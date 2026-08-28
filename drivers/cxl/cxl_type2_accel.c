// SPDX-License-Identifier: GPL-2.0-only
/*
 * CXL Type 2 Accelerator Driver
 * Supports CXL Type 2 devices with CXL.cache capabilities
 *
 * Copyright(c) 2025 Victor Yang
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/compat.h>
#include <linux/fs.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/overflow.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/uaccess.h>
#include <uapi/linux/cxl_type2_accel.h>

#include "cxlmem.h"
#include "cxlpci.h"
#include "cxlcache.h"
#include "cxl.h"
#include "capcxl_identity.h"
#include "tmatmul_identity.h"

#define CXL_TYPE2_VENDOR_ID	0x8086
#define CXL_TYPE2_DEVICE_ID_QEMU	0x0d92
#define CXL_TYPE2_DEVICE_ID_IA780I	0x0ddb

#define CAPCXL_ID_OFFSET		0x1c0ff0
#define CAPCXL_ID_MAP_SIZE		0x10
#define CAPCXL_HPA_BASE		0x180000000000ULL
#define CAPCXL_MEMORY_SIZE		(4ULL * SZ_1G)

enum cxl_type2_match_role {
	CXL_TYPE2_MATCH_GENERIC,
	CXL_TYPE2_MATCH_IA780I_ACCEL,
	CXL_TYPE2_MATCH_IA780I_MEM,
};

#define TMATMUL_DEV_ID			TMATMUL_ID_VALUE
#define TMATMUL_CSR_BASE		0x1C0000
#define TMATMUL_CSR_SIZE		0x2000
#define TMATMUL_INSTANCE_BASE		0x100
#define TMATMUL_INSTANCE_STRIDE		0x80

#define TMATMUL_REG_DEV_ID		0x000
#define TMATMUL_REG_NUM_INSTANCES	0x004
#define TMATMUL_REG_DIM_D		0x008
#define TMATMUL_REG_DDR_DATA_WIDTH	0x00C
#define TMATMUL_REG_DDR_PROBE_ADDR	0x010
#define TMATMUL_REG_DDR_PROBE_WDATA	0x014
#define TMATMUL_REG_DDR_PROBE_CTRL	0x018
#define TMATMUL_REG_DDR_PROBE_RDATA	0x01C
#define TMATMUL_REG_DDR_PROBE_STATUS	0x024
#define TMATMUL_REG_MC_STATUS		0x018

#define TMATMUL_DDR_PROBE_START		BIT(0)
#define TMATMUL_DDR_PROBE_WRITE		BIT(1)
#define TMATMUL_DDR_PROBE_STATUS_BUSY	0x1
#define TMATMUL_DDR_PROBE_STATUS_DONE	0x2
#define TMATMUL_DDR_PROBE_STATUS_ERROR	0x3
#define TMATMUL_DDR_PROBE_WORD_BYTES	sizeof(u32)
#define TMATMUL_DDR_PROBE_LINE_BYTES	64
#define TMATMUL_DDR_PROBE_TIMEOUT_MS	250

#define TMATMUL_INST_STALL_STATUS	0x00
#define TMATMUL_INST_STALL_CLEAR	0x04
#define TMATMUL_INST_RST_TRIGGER	0x08
#define TMATMUL_INST_RST_STATUS		0x0C
#define TMATMUL_INST_INSTR_SRC_LO	0x10
#define TMATMUL_INST_INSTR_SRC_HI	0x14
#define TMATMUL_INST_INSTR_LEN		0x18
#define TMATMUL_INST_INSTR_START	0x1C
#define TMATMUL_INST_DBG_INSTR_CNT	0x40
#define TMATMUL_INST_EXEC_STATUS	0x60

#define TMATMUL_DMA_IDLE		0x00
#define TMATMUL_DMA_RUNNING		0x01
#define TMATMUL_DMA_DONE		0x02
#define TMATMUL_DMA_ERROR		0xFF

#define TMATMUL_DDR_INSTR_DPA_DEFAULT	0x00600000ULL
#define TMATMUL_PROGRAM_BYTES		(8 * 16)

static bool enable_cache;
module_param(enable_cache, bool, 0644);
MODULE_PARM_DESC(enable_cache,
		 "Enable CXL.cache registration (default false; CXL-Secured uses CXL.mem)");

static bool enable_memdev;
module_param(enable_memdev, bool, 0644);
MODULE_PARM_DESC(enable_memdev,
		 "Register CXL memdev/HDM/DAX path (default false; CSR-only boot-safe mode)");

static bool allow_uncommitted_hdm;
module_param(allow_uncommitted_hdm, bool, 0644);
MODULE_PARM_DESC(allow_uncommitted_hdm,
		 "Allow memdev registration when HDM commit readback fails (unsafe; default false)");

static unsigned long long type2_hpa_base = CAPCXL_HPA_BASE;
module_param(type2_hpa_base, ullong, 0644);
MODULE_PARM_DESC(type2_hpa_base,
		 "HPA base programmed into the generic Type-2 endpoint HDM decoder");

static bool use_dvsec_hdm = true;
module_param(use_dvsec_hdm, bool, 0644);
MODULE_PARM_DESC(use_dvsec_hdm,
		 "Model Type-2 memory from active DVSEC ranges instead of component HDM registers");

static unsigned long long tmatmul_program_dpa =
	TMATMUL_DDR_INSTR_DPA_DEFAULT;
module_param(tmatmul_program_dpa, ullong, 0444);
MODULE_PARM_DESC(tmatmul_program_dpa,
		 "DPA of the fixed 128-byte RUN_CSR_ONLY instruction program");

static int type2_parse_capacity(struct cxl_memdev_state *mds,
				const struct cxl_mbox_identify *id)
{
	struct device *dev = mds->cxlds.dev;
	u64 total_units = le64_to_cpu(id->total_capacity);
	u64 volatile_units = le64_to_cpu(id->volatile_capacity);
	u64 persistent_units = le64_to_cpu(id->persistent_capacity);
	u64 align_units = le64_to_cpu(id->partition_align);
	u64 total, volatile_bytes, persistent, align, partitioned;

	if (!total_units || total_units == U64_MAX ||
	    volatile_units == U64_MAX || persistent_units == U64_MAX ||
	    align_units == U64_MAX) {
		dev_err(dev,
			"invalid Identify capacity: total=%#llx volatile=%#llx persistent=%#llx align=%#llx (256 MiB units)\n",
			total_units, volatile_units, persistent_units,
			align_units);
		return -EINVAL;
	}

	if (check_mul_overflow(total_units, CXL_CAPACITY_MULTIPLIER,
			       &total) ||
	    check_mul_overflow(volatile_units, CXL_CAPACITY_MULTIPLIER,
			       &volatile_bytes) ||
	    check_mul_overflow(persistent_units, CXL_CAPACITY_MULTIPLIER,
			       &persistent) ||
	    check_mul_overflow(align_units, CXL_CAPACITY_MULTIPLIER,
			       &align)) {
		dev_err(dev,
			"Identify capacity overflows bytes: total=%#llx volatile=%#llx persistent=%#llx align=%#llx\n",
			total_units, volatile_units, persistent_units,
			align_units);
		return -EOVERFLOW;
	}

	if ((!volatile_bytes && !persistent) ||
	    volatile_bytes > total || persistent > total ||
	    check_add_overflow(volatile_bytes, persistent, &partitioned) ||
	    partitioned > total) {
		dev_err(dev,
			"inconsistent Identify capacity: total=%#llx volatile=%#llx persistent=%#llx bytes\n",
			total, volatile_bytes, persistent);
		return -EINVAL;
	}

	mds->total_bytes = total;
	mds->volatile_only_bytes = volatile_bytes;
	mds->persistent_only_bytes = persistent;
	mds->active_volatile_bytes = volatile_bytes;
	mds->active_persistent_bytes = persistent;
	mds->partition_align_bytes = align;
	memcpy(mds->firmware_version, id->fw_revision,
	       sizeof(id->fw_revision));

	dev_info(dev,
		 "mailbox Identify capacity: total=%#llx volatile=%#llx persistent=%#llx bytes\n",
		 total, volatile_bytes, persistent);
	return 0;
}

static int type2_read_dvsec_range(struct cxl_memdev_state *mds,
				  u64 *mapped_base, u64 *capacity)
{
	struct cxl_dev_state *cxlds = &mds->cxlds;
	struct pci_dev *pdev = to_pci_dev(cxlds->dev);
	u64 first_base = 0, next_base = 0, total = 0;
	bool found = false;
	int i;

	if (!cxlds->cxl_dvsec) {
		dev_err(cxlds->dev,
			"CXL Device DVSEC is missing; capacity cannot be verified\n");
		return -ENODEV;
	}

	for (i = 0; i < CXL_DVSEC_RANGE_MAX; i++) {
		u64 base, end, size;
		u32 base_high, base_low, size_high, size_low;

		if (pci_read_config_dword(pdev, cxlds->cxl_dvsec +
					 CXL_DVSEC_RANGE_SIZE_HIGH(i),
					 &size_high) ||
		    pci_read_config_dword(pdev, cxlds->cxl_dvsec +
					 CXL_DVSEC_RANGE_SIZE_LOW(i),
					 &size_low) ||
		    pci_read_config_dword(pdev, cxlds->cxl_dvsec +
					 CXL_DVSEC_RANGE_BASE_HIGH(i),
					 &base_high) ||
		    pci_read_config_dword(pdev, cxlds->cxl_dvsec +
					 CXL_DVSEC_RANGE_BASE_LOW(i),
					 &base_low))
			return -EIO;

		if (!(size_low & CXL_DVSEC_MEM_INFO_VALID))
			continue;

		size = ((u64)size_high << 32) |
		       (size_low & CXL_DVSEC_MEM_SIZE_LOW_MASK);
		if (!size) {
			dev_err(cxlds->dev,
				"DVSEC range %d is valid but advertises zero capacity (high=%#x low=%#x)\n",
				i, size_high, size_low);
			return -EINVAL;
		}

		base = ((u64)base_high << 32) |
		       (base_low & CXL_DVSEC_MEM_BASE_LOW_MASK);
		if (!base) {
			dev_err(cxlds->dev,
				"DVSEC range %d is valid but has no programmed HPA base\n",
				i);
			return -EINVAL;
		}
		if (check_add_overflow(base, size, &end))
			return -EOVERFLOW;

		if (!found)
			first_base = base;
		else if (base != next_base) {
			dev_err(cxlds->dev,
				"DVSEC ranges are not contiguous: range %d starts at %#llx, expected %#llx\n",
				i, base, next_base);
			return -EINVAL;
		}

		if (check_add_overflow(total, size, &total))
			return -EOVERFLOW;
		next_base = end;
		found = true;
	}

	if (!found) {
		dev_err(cxlds->dev,
			"CXL Device DVSEC has no valid memory range\n");
		return -ENODEV;
	}

	*mapped_base = first_base;
	*capacity = total;
	return 0;
}

static int type2_read_identify(struct cxl_memdev_state *mds,
			       struct cxl_mbox_identify *id)
{
	struct cxl_dev_state *cxlds = &mds->cxlds;
	struct device *dev = cxlds->dev;
	void __iomem *mbox = cxlds->regs.mbox;
	void __iomem *memdev = cxlds->regs.memdev;
	u64 md_status, cmd_reg, status_reg;
	u32 cap, ctrl, payload_exp;
	size_t payload_size, out_len;
	u16 return_code;
	int rc;

	if (!mbox || !memdev) {
		dev_err(dev,
			"memory-device mailbox registers are not mapped\n");
		return -ENODEV;
	}

	md_status = readq(memdev + CXLMDEV_STATUS_OFFSET);
	if ((md_status & (CXLMDEV_DEV_FATAL | CXLMDEV_FW_HALT)) ||
	    !CXLMDEV_READY(md_status) ||
	    !(md_status & CXLMDEV_MBOX_IF_READY)) {
		dev_err(dev,
			"memory device is not ready for Identify (status=%#llx)\n",
			md_status);
		return -ENODEV;
	}

	cap = readl(mbox + CXLDEV_MBOX_CAPS_OFFSET);
	payload_exp = FIELD_GET(CXLDEV_MBOX_CAP_PAYLOAD_SIZE_MASK, cap);
	payload_size = 1ULL << payload_exp;
	if (payload_size < sizeof(*id)) {
		dev_err(dev,
			"mailbox payload is too small for Identify (%zu bytes)\n",
			payload_size);
		return -ENXIO;
	}

	rc = readl_poll_timeout(mbox + CXLDEV_MBOX_CTRL_OFFSET, ctrl,
				!(ctrl & CXLDEV_MBOX_CTRL_DOORBELL),
				1000, 2 * 1000 * 1000);
	if (rc) {
		dev_err(dev, "timeout waiting for mailbox idle\n");
		return rc;
	}

	cmd_reg = FIELD_PREP(CXLDEV_MBOX_CMD_COMMAND_OPCODE_MASK,
			     CXL_MBOX_OP_IDENTIFY);
	writeq(cmd_reg, mbox + CXLDEV_MBOX_CMD_OFFSET);
	writel(CXLDEV_MBOX_CTRL_DOORBELL,
	       mbox + CXLDEV_MBOX_CTRL_OFFSET);

	rc = readl_poll_timeout(mbox + CXLDEV_MBOX_CTRL_OFFSET, ctrl,
				!(ctrl & CXLDEV_MBOX_CTRL_DOORBELL),
				1000, 2 * 1000 * 1000);
	if (rc) {
		dev_err(dev, "mailbox Identify timed out\n");
		return rc;
	}

	status_reg = readq(mbox + CXLDEV_MBOX_STATUS_OFFSET);
	return_code = FIELD_GET(CXLDEV_MBOX_STATUS_RET_CODE_MASK, status_reg);
	if ((status_reg & CXLDEV_MBOX_STATUS_BG_CMD) ||
	    return_code != CXL_MBOX_CMD_RC_SUCCESS) {
		dev_err(dev,
			"mailbox Identify failed (status=%#llx return_code=%u)\n",
			status_reg, return_code);
		return -EIO;
	}

	cmd_reg = readq(mbox + CXLDEV_MBOX_CMD_OFFSET);
	out_len = FIELD_GET(CXLDEV_MBOX_CMD_PAYLOAD_LENGTH_MASK, cmd_reg);
	if (out_len < sizeof(*id) || out_len > payload_size) {
		dev_err(dev,
			"mailbox Identify returned invalid payload length %zu\n",
			out_len);
		return -EPROTO;
	}

	memcpy_fromio(id, mbox + CXLDEV_MBOX_PAYLOAD_OFFSET, sizeof(*id));
	return 0;
}

static int cxl_type2_identify_capacity(struct cxl_memdev_state *mds,
					u64 *mapped_base,
					u64 *mapped_capacity)
{
	struct cxl_mbox_identify first = {};
	struct cxl_mbox_identify second = {};
	u64 dvsec_capacity, active_capacity;
	int rc;

	rc = type2_read_identify(mds, &first);
	if (rc)
		return rc;

	rc = type2_read_identify(mds, &second);
	if (rc)
		return rc;

	if (memcmp(&first, &second, sizeof(first))) {
		dev_err(mds->cxlds.dev,
			"mailbox Identify payload is not repeatable; refusing capacity discovery\n");
		return -EIO;
	}

	rc = type2_parse_capacity(mds, &first);
	if (rc)
		return rc;

	rc = type2_read_dvsec_range(mds, mapped_base, &dvsec_capacity);
	if (rc)
		return rc;

	if (check_add_overflow(mds->active_volatile_bytes,
			       mds->active_persistent_bytes,
			       &active_capacity))
		return -EOVERFLOW;

	/*
	 * Identify reports installed media, while the active DVSEC ranges
	 * describe the subset currently mapped into HPA space.
	 */
	if (dvsec_capacity > active_capacity) {
		dev_err(mds->cxlds.dev,
			"DVSEC capacity %#llx exceeds active mailbox capacity %#llx bytes\n",
			dvsec_capacity, active_capacity);
		return -EINVAL;
	}
	if (dvsec_capacity != mds->total_bytes)
		dev_info(mds->cxlds.dev,
			 "active DVSEC maps %#llx of %#llx mailbox bytes\n",
			 dvsec_capacity, mds->total_bytes);

	*mapped_capacity = dvsec_capacity;
	mds->cxlds.media_ready = true;
	return 0;
}

#define TYPE2_MAX_SOFT_RESERVED_RANGES	8

struct type2_soft_reserved_range {
	resource_size_t start;
	resource_size_t end;
	unsigned long flags;
	unsigned long desc;
	struct resource *parent;
};

struct type2_soft_reserved_scan {
	struct type2_soft_reserved_range range[TYPE2_MAX_SOFT_RESERVED_RANGES];
	unsigned int nr;
};

struct type2_soft_reserved_merge {
	struct device *dev;
	struct resource *parent;
	struct resource wrapper;
	struct resource *original[TYPE2_MAX_SOFT_RESERVED_RANGES];
	unsigned int nr_original;
	unsigned int nr_removed;
	bool inserted;
};

static DEFINE_MUTEX(type2_resource_fixup_lock);

static int type2_collect_soft_reserved(struct resource *res, void *data)
{
	struct type2_soft_reserved_scan *scan = data;
	struct type2_soft_reserved_range *range;

	if (scan->nr == ARRAY_SIZE(scan->range))
		return -E2BIG;

	range = &scan->range[scan->nr++];
	range->start = res->start;
	range->end = res->end;
	range->flags = res->flags;
	range->desc = res->desc;
	range->parent = res->parent;
	return 0;
}

static int
type2_reinsert_soft_reserved_locked(struct type2_soft_reserved_merge *merge)
{
	int first_rc = 0;
	unsigned int i;

	for (i = 0; i < merge->nr_removed; i++) {
		struct resource *res = merge->original[i];
		int rc;

		if (res->parent == &merge->wrapper)
			continue;
		if (res->parent) {
			dev_crit(merge->dev,
				 "Soft Reserved resource %pR was unexpectedly reparented\n",
				 res);
			if (!first_rc)
				first_rc = -EBUSY;
			continue;
		}

		rc = request_resource(&merge->wrapper, res);
		if (rc) {
			dev_crit(merge->dev,
				 "failed to restore Soft Reserved resource %pR: %d\n",
				 res, rc);
			if (!first_rc)
				first_rc = rc;
		}
	}

	return first_rc;
}

static int type2_remove_wrapper_locked(struct type2_soft_reserved_merge *merge)
{
	int rc;

	if (!merge->inserted)
		return 0;

	rc = type2_reinsert_soft_reserved_locked(merge);
	if (rc)
		return rc;

	rc = remove_resource(&merge->wrapper);
	if (rc)
		return rc;

	merge->inserted = false;
	return 0;
}

static int
__type2_restore_soft_reserved(struct type2_soft_reserved_merge *merge)
{
	int rc;

	if (!merge->inserted)
		return 0;

	if (merge->wrapper.parent != merge->parent ||
	    merge->wrapper.child) {
		dev_err(merge->dev,
			"cannot restore split Soft Reserved resources while normalized range is owned or reparented\n");
		return -EBUSY;
	}

	rc = type2_remove_wrapper_locked(merge);
	if (rc)
		return rc;

	dev_info(merge->dev,
		 "restored %u split Soft Reserved resources after DAX teardown\n",
		 merge->nr_original);
	kfree(merge);
	return 0;
}

static void type2_restore_soft_reserved(void *data)
{
	struct type2_soft_reserved_merge *merge = data;
	int rc;

	mutex_lock(&type2_resource_fixup_lock);
	rc = __type2_restore_soft_reserved(merge);
	if (rc)
		dev_crit(merge->dev,
			 "leaving normalized Soft Reserved resource allocated after teardown failure: %d\n",
			 rc);
	mutex_unlock(&type2_resource_fixup_lock);
}

static int type2_coalesce_soft_reserved(struct device *dev, u64 mapped_base,
					 u64 mapped_capacity)
{
	struct type2_soft_reserved_scan scan = {};
	struct type2_soft_reserved_merge *merge;
	struct resource *parent;
	struct resource *res;
	resource_size_t start, end;
	unsigned int i;
	u64 end64;
	int rc;

	if (!mapped_capacity || mapped_capacity > SIZE_MAX ||
	    check_add_overflow(mapped_base, mapped_capacity - 1, &end64) ||
	    (u64)(resource_size_t)mapped_base != mapped_base ||
	    (u64)(resource_size_t)end64 != end64)
		return -EOVERFLOW;

	start = mapped_base;
	end = end64;

	mutex_lock(&type2_resource_fixup_lock);

	rc = region_intersects(start, mapped_capacity, IORESOURCE_SYSTEM_RAM,
			       IORES_DESC_NONE);
	if (rc != REGION_DISJOINT) {
		dev_err(dev,
			"refusing DVSEC resource fixup: System RAM intersects %pa-%pa\n",
			&start, &end);
		rc = -EBUSY;
		goto out_unlock;
	}

	rc = walk_iomem_res_desc(IORES_DESC_SOFT_RESERVED, IORESOURCE_MEM,
				 start, end, &scan,
				 type2_collect_soft_reserved);
	if (rc && (scan.nr || rc != -EINVAL)) {
		dev_err(dev,
			"failed to enumerate Soft Reserved resources for %pa-%pa: %d\n",
			&start, &end, rc);
		goto out_unlock;
	}
	if (!scan.nr) {
		rc = 0;
		goto out_unlock;
	}

	if (scan.range[0].start != start ||
	    scan.range[scan.nr - 1].end != end) {
		dev_err(dev,
			"Soft Reserved resources do not cover the complete DVSEC range %pa-%pa\n",
			&start, &end);
		rc = -EINVAL;
		goto out_unlock;
	}

	for (i = 0; i < scan.nr; i++) {
		struct type2_soft_reserved_range *range = &scan.range[i];

		if (range->desc != IORES_DESC_SOFT_RESERVED ||
		    (range->flags & IORESOURCE_TYPE_BITS) != IORESOURCE_MEM ||
		    (range->flags & IORESOURCE_BUSY) ||
		    !range->parent ||
		    (i && (scan.range[i - 1].end == RESOURCE_SIZE_MAX ||
			   scan.range[i - 1].end + 1 != range->start)) ||
		    (i && (range->flags != scan.range[0].flags ||
			   range->desc != scan.range[0].desc ||
			   range->parent != scan.range[0].parent))) {
			dev_err(dev,
				"DVSEC Soft Reserved resources are not safe, identical, and contiguous\n");
			rc = -EINVAL;
			goto out_unlock;
		}
	}

	parent = scan.range[0].parent;
	if (parent->start != start || parent->end != end ||
	    resource_type(parent) != IORESOURCE_MEM ||
	    parent->desc != IORES_DESC_CXL ||
	    (parent->flags & IORESOURCE_BUSY)) {
		rc = -EINVAL;
		dev_err(dev,
			"no exact non-busy CXL window owns DVSEC range %pa-%pa: %d\n",
			&start, &end, rc);
		goto out_unlock;
	}

	if (scan.nr == 1) {
		rc = 0;
		goto out_unlock;
	}

	merge = kzalloc(sizeof(*merge), GFP_KERNEL);
	if (!merge) {
		rc = -ENOMEM;
		goto out_unlock;
	}
	merge->dev = dev;
	merge->parent = parent;
	merge->wrapper = (struct resource) {
		.name = "CXL Type-2 normalized Soft Reserved",
		.start = start,
		.end = end,
		.flags = scan.range[0].flags,
		.desc = IORES_DESC_SOFT_RESERVED,
	};

	rc = insert_resource(parent, &merge->wrapper);
	if (rc) {
		dev_err(dev,
			"failed to insert normalized Soft Reserved resource: %d\n",
			rc);
		kfree(merge);
		goto out_unlock;
	}
	merge->inserted = true;

	res = merge->wrapper.child;
	for (i = 0; i < scan.nr; i++) {
		struct type2_soft_reserved_range *range = &scan.range[i];

		if (!res || res->parent != &merge->wrapper ||
		    res->start != range->start || res->end != range->end ||
		    res->flags != range->flags || res->desc != range->desc ||
		    res->child) {
			dev_err(dev,
				"Soft Reserved resource %u changed or has an active child; refusing fixup\n",
				i);
			rc = -EBUSY;
			goto rollback;
		}
		merge->original[merge->nr_original++] = res;
		res = res->sibling;
	}
	if (res) {
		dev_err(dev,
			"normalized Soft Reserved resource captured unexpected children\n");
		rc = -EBUSY;
		goto rollback;
	}

	for (i = 0; i < merge->nr_original; i++) {
		rc = remove_resource(merge->original[i]);
		if (rc)
			goto rollback;
		merge->nr_removed++;
	}

	if (merge->wrapper.child) {
		rc = -EAGAIN;
		goto rollback;
	}

	rc = devm_add_action(dev, type2_restore_soft_reserved, merge);
	if (rc) {
		int restore_rc = __type2_restore_soft_reserved(merge);

		if (restore_rc)
			dev_crit(dev,
				 "failed to restore resources after devres registration error: %d\n",
				 restore_rc);
		goto out_unlock;
	}

	dev_info(dev,
		 "coalesced %u adjacent Soft Reserved resources into %pR for native DVSEC DAX\n",
		 scan.nr, &merge->wrapper);
	rc = 0;
	goto out_unlock;

rollback:
	{
		int restore_rc = type2_remove_wrapper_locked(merge);

		if (restore_rc)
			dev_crit(dev,
				 "Soft Reserved rollback was incomplete: %d\n",
				 restore_rc);
		else
			kfree(merge);
	}
out_unlock:
	mutex_unlock(&type2_resource_fixup_lock);
	return rc;
}

/*
 * Userspace places its instruction program at this device-physical-address
 * offset within the dax window.  The driver only needs to point the CSR
 * fetch engine at it; the actual bytes are written by userspace.
 */
struct cxl_type2_tmatmul_dev {
	struct pci_dev *pdev;
	void __iomem *csr;
	struct miscdevice miscdev;
	struct mutex lock;
};

static u32 tmatmul_rd32(struct cxl_type2_tmatmul_dev *tmatmul, u32 off)
{
	return readl(tmatmul->csr + off);
}

static void tmatmul_wr32(struct cxl_type2_tmatmul_dev *tmatmul, u32 off, u32 val)
{
	writel(val, tmatmul->csr + off);
}

static u32 tmatmul_inst_off(u32 inst, u32 off)
{
	return TMATMUL_INSTANCE_BASE + inst * TMATMUL_INSTANCE_STRIDE + off;
}

static int tmatmul_launch_csr_only(struct cxl_type2_tmatmul_dev *tmatmul,
				   struct cxl_type2_tmatmul_csr_run *run)
{
	unsigned long deadline;
	u32 timeout_ms = run->timeout_ms ?: 5000;
	u32 num_instances;
	u32 instr_before;
	u32 status;
	int rc = 0;

	if (run->flags)
		return -EINVAL;

	if (!IS_ALIGNED(tmatmul_program_dpa, 64) ||
	    tmatmul_program_dpa > U64_MAX - TMATMUL_PROGRAM_BYTES)
		return -EINVAL;

	num_instances = tmatmul_rd32(tmatmul, TMATMUL_REG_NUM_INSTANCES);
	if (!num_instances)
		return -ENODEV;

	run->dim_d = tmatmul_rd32(tmatmul, TMATMUL_REG_DIM_D);

	mutex_lock(&tmatmul->lock);

	status = tmatmul_rd32(tmatmul,
			      tmatmul_inst_off(0, TMATMUL_INST_RST_STATUS));
	if (status & 0x1) {
		rc = -EBUSY;
		goto out_unlock;
	}

	/*
	 * Do not pulse the per-instance reset between runs. The FPGA's
	 * toggle-based instr_dma_start synchronizer resets only its destination
	 * state on an instance reset; its CSR-source toggle remains live. After
	 * the first launch, resetting only the destination synthesizes a second
	 * edge and replays the previous DMA descriptor.
	 *
	 * A cold engine is (IDLE, not stalled), while a completed engine is
	 * (DONE, stalled). Reject every other combination. The instruction
	 * counter is cumulative without a reset, so report the per-run delta.
	 */
	status = tmatmul_rd32(tmatmul,
			      tmatmul_inst_off(0, TMATMUL_INST_EXEC_STATUS));
	if (status) {
		rc = -EIO;
		goto out_unlock;
	}

	status = tmatmul_rd32(tmatmul,
			      tmatmul_inst_off(0, TMATMUL_INST_INSTR_START)) & 0xff;
	if (status == TMATMUL_DMA_IDLE) {
		if (tmatmul_rd32(tmatmul,
				tmatmul_inst_off(0, TMATMUL_INST_STALL_STATUS))) {
			rc = -EIO;
			goto out_unlock;
		}
	} else if (status == TMATMUL_DMA_DONE) {
		if (!tmatmul_rd32(tmatmul,
				 tmatmul_inst_off(0, TMATMUL_INST_STALL_STATUS))) {
			rc = -EBUSY;
			goto out_unlock;
		}
	} else if (status == TMATMUL_DMA_RUNNING) {
		rc = -EBUSY;
		goto out_unlock;
	} else {
		rc = -EIO;
		goto out_unlock;
	}

	instr_before = tmatmul_rd32(tmatmul,
				   tmatmul_inst_off(0,
						    TMATMUL_INST_DBG_INSTR_CNT));

	tmatmul_wr32(tmatmul, tmatmul_inst_off(0, TMATMUL_INST_STALL_CLEAR), 1);
	rc = readl_poll_timeout(tmatmul->csr +
			tmatmul_inst_off(0, TMATMUL_INST_STALL_STATUS),
			status, !status, 10, 100000);
	if (rc)
		goto out_unlock;

	tmatmul_wr32(tmatmul, tmatmul_inst_off(0, TMATMUL_INST_INSTR_SRC_LO),
		     lower_32_bits(tmatmul_program_dpa));
	tmatmul_wr32(tmatmul, tmatmul_inst_off(0, TMATMUL_INST_INSTR_SRC_HI),
		     upper_32_bits(tmatmul_program_dpa));
	tmatmul_wr32(tmatmul, tmatmul_inst_off(0, TMATMUL_INST_INSTR_LEN),
		     TMATMUL_PROGRAM_BYTES);
	tmatmul_wr32(tmatmul, tmatmul_inst_off(0, TMATMUL_INST_INSTR_START), 1);

	deadline = jiffies + msecs_to_jiffies(timeout_ms);
	do {
		run->dma_status = tmatmul_rd32(tmatmul,
			tmatmul_inst_off(0, TMATMUL_INST_INSTR_START)) & 0xff;
		run->stall_status = tmatmul_rd32(tmatmul,
			tmatmul_inst_off(0, TMATMUL_INST_STALL_STATUS));
		status = tmatmul_rd32(tmatmul,
				      tmatmul_inst_off(0, TMATMUL_INST_DBG_INSTR_CNT));
		run->instr_count = status - instr_before;

		if (run->stall_status) {
			run->result_flags |= CXL_TYPE2_TMATMUL_RESULT_STALLED;
			goto out_unlock;
		}
		if (run->dma_status == TMATMUL_DMA_ERROR) {
			run->result_flags |= CXL_TYPE2_TMATMUL_RESULT_DMA_ERROR;
			rc = -EIO;
			goto out_unlock;
		}

		usleep_range(1000, 2000);
	} while (time_before(jiffies, deadline));

	rc = -ETIMEDOUT;

out_unlock:
	mutex_unlock(&tmatmul->lock);
	return rc;
}

static int tmatmul_probe_wait(struct cxl_type2_tmatmul_dev *tmatmul)
{
	unsigned long deadline = jiffies +
		msecs_to_jiffies(TMATMUL_DDR_PROBE_TIMEOUT_MS);
	u32 status;

	/*
	 * The CSR-to-DDR probe status is level-held. Give the pulse synchronizer
	 * time to observe the new request before polling, matching the userspace
	 * BAR smoke that proved this path on IA-780I.
	 */
	usleep_range(5000, 6000);
	do {
		status = tmatmul_rd32(tmatmul, TMATMUL_REG_DDR_PROBE_STATUS);
		switch (status & 0x3) {
		case TMATMUL_DDR_PROBE_STATUS_DONE:
			return 0;
		case TMATMUL_DDR_PROBE_STATUS_ERROR:
			return -EIO;
		case TMATMUL_DDR_PROBE_STATUS_BUSY:
		default:
			usleep_range(1000, 2000);
			break;
		}
	} while (time_before(jiffies, deadline));

	return -ETIMEDOUT;
}

static int tmatmul_probe_write_dword(struct cxl_type2_tmatmul_dev *tmatmul,
				     u64 offset, u32 value)
{
	if (offset > U32_MAX)
		return -ERANGE;

	tmatmul_wr32(tmatmul, TMATMUL_REG_DDR_PROBE_ADDR,
		     lower_32_bits(offset));
	tmatmul_wr32(tmatmul, TMATMUL_REG_DDR_PROBE_WDATA, value);
	tmatmul_wr32(tmatmul, TMATMUL_REG_DDR_PROBE_CTRL,
		     TMATMUL_DDR_PROBE_START | TMATMUL_DDR_PROBE_WRITE);

	return tmatmul_probe_wait(tmatmul);
}

static int tmatmul_probe_read_dword(struct cxl_type2_tmatmul_dev *tmatmul,
				    u64 offset, u32 *value)
{
	int rc;

	if (offset > U32_MAX)
		return -ERANGE;

	tmatmul_wr32(tmatmul, TMATMUL_REG_DDR_PROBE_ADDR,
		     lower_32_bits(offset));
	tmatmul_wr32(tmatmul, TMATMUL_REG_DDR_PROBE_CTRL,
		     TMATMUL_DDR_PROBE_START);

	rc = tmatmul_probe_wait(tmatmul);
	if (rc)
		return rc;

	*value = tmatmul_rd32(tmatmul, TMATMUL_REG_DDR_PROBE_RDATA);
	return 0;
}

static int tmatmul_mem_io_write(struct cxl_type2_tmatmul_dev *tmatmul,
				u64 offset, const u8 *buf, u32 size)
{
	u32 pos = 0;

	/*
	 * The IA-780I probe path is 32-bit, but the DDR side is a 512-bit line.
	 * Write zero dwords before non-zero dwords within each DDR line so a
	 * sparse-write implementation that temporarily drives zero on untouched
	 * lanes cannot erase payload words written earlier in the line.
	 */
	while (pos < size) {
		u32 line_rem = TMATMUL_DDR_PROBE_LINE_BYTES -
			((offset + pos) & (TMATMUL_DDR_PROBE_LINE_BYTES - 1));
		u32 line_len = min(line_rem, size - pos);
		u32 phase;

		for (phase = 0; phase < 2; phase++) {
			u32 i;

			for (i = 0; i < line_len; i += sizeof(u32)) {
				u32 word;
				int rc;

				memcpy(&word, buf + pos + i, sizeof(word));
				if ((word != 0) != phase)
					continue;

				rc = tmatmul_probe_write_dword(tmatmul,
							       offset + pos + i,
							       word);
				if (rc)
					return rc;
			}
		}

		pos += line_len;
	}

	return 0;
}

static int tmatmul_mem_io_read(struct cxl_type2_tmatmul_dev *tmatmul,
			       u64 offset, u8 *buf, u32 size)
{
	u32 pos;

	for (pos = 0; pos < size; pos += sizeof(u32)) {
		u32 word;
		int rc = tmatmul_probe_read_dword(tmatmul, offset + pos, &word);

		if (rc)
			return rc;
		memcpy(buf + pos, &word, sizeof(word));
	}

	return 0;
}

static int tmatmul_mem_io(struct cxl_type2_tmatmul_dev *tmatmul,
			  struct cxl_type2_mem_req *req)
{
	void __user *uptr;
	u8 *buf;
	int rc;

	if (req->flags || req->reserved0)
		return -EINVAL;
	if (!req->user_ptr)
		return -EINVAL;
	if (!req->size || req->size > CXL_TYPE2_MEM_REQ_MAX_BYTES)
		return -EINVAL;
	if (!IS_ALIGNED(req->offset, TMATMUL_DDR_PROBE_WORD_BYTES) ||
	    !IS_ALIGNED(req->size, TMATMUL_DDR_PROBE_WORD_BYTES))
		return -EINVAL;
	if (req->offset > U32_MAX || req->size > U32_MAX - req->offset + 1)
		return -ERANGE;
	if (req->hpa_size && req->offset + req->size > req->hpa_size)
		return -ERANGE;
	if (req->op != CXL_TYPE2_MEM_REQ_READ &&
	    req->op != CXL_TYPE2_MEM_REQ_WRITE)
		return -EINVAL;

	uptr = u64_to_user_ptr(req->user_ptr);
	buf = kvzalloc(req->size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (req->op == CXL_TYPE2_MEM_REQ_WRITE) {
		if (copy_from_user(buf, uptr, req->size)) {
			rc = -EFAULT;
			goto out;
		}

		mutex_lock(&tmatmul->lock);
		rc = tmatmul_mem_io_write(tmatmul, req->offset, buf, req->size);
		mutex_unlock(&tmatmul->lock);
	} else {
		mutex_lock(&tmatmul->lock);
		rc = tmatmul_mem_io_read(tmatmul, req->offset, buf, req->size);
		mutex_unlock(&tmatmul->lock);
		if (rc)
			goto out;
		if (copy_to_user(uptr, buf, req->size))
			rc = -EFAULT;
	}

out:
	kvfree(buf);
	return rc;
}

static long cxl_type2_tmatmul_ioctl(struct file *file, unsigned int cmd,
				    unsigned long arg)
{
	struct miscdevice *miscdev = file->private_data;
	struct cxl_type2_tmatmul_dev *tmatmul =
		container_of(miscdev, struct cxl_type2_tmatmul_dev, miscdev);
	void __user *argp = (void __user *)arg;
	struct cxl_type2_tmatmul_info info;
	struct cxl_type2_tmatmul_csr_run run;
	struct cxl_type2_mem_req mem_req;
	int rc;

	switch (cmd) {
	case CXL_TYPE2_TMATMUL_GET_INFO:
		memset(&info, 0, sizeof(info));
		info.version = CXL_TYPE2_TMATMUL_UAPI_VERSION;
		info.dev_id = tmatmul_rd32(tmatmul, TMATMUL_REG_DEV_ID);
		info.num_instances = tmatmul_rd32(tmatmul,
						  TMATMUL_REG_NUM_INSTANCES);
		info.dim_d = tmatmul_rd32(tmatmul, TMATMUL_REG_DIM_D);
		info.ddr_data_width = tmatmul_rd32(tmatmul,
						   TMATMUL_REG_DDR_DATA_WIDTH);
		info.mc_status = tmatmul_rd32(tmatmul, TMATMUL_REG_MC_STATUS);
		if (copy_to_user(argp, &info, sizeof(info)))
			return -EFAULT;
		return 0;

	case CXL_TYPE2_TMATMUL_RUN_CSR_ONLY:
		if (copy_from_user(&run, argp, sizeof(run)))
			return -EFAULT;
		run.dma_status = 0;
		run.stall_status = 0;
		run.instr_count = 0;
		run.dim_d = 0;
		run.result_flags = 0;
		rc = tmatmul_launch_csr_only(tmatmul, &run);
		if (copy_to_user(argp, &run, sizeof(run)))
			return -EFAULT;
		return rc;

	case CXL_TYPE2_MEM_IO:
		if (copy_from_user(&mem_req, argp, sizeof(mem_req)))
			return -EFAULT;
		return tmatmul_mem_io(tmatmul, &mem_req);

	default:
		return -ENOTTY;
	}
}

static int cxl_type2_tmatmul_release(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct cxl_type2_tmatmul_dev *tmatmul =
		container_of(miscdev, struct cxl_type2_tmatmul_dev, miscdev);
	u32 dma_status, exec_status, instr_count, reset_status, stall_status;
	const u32 rearm_period = 2 * (TMATMUL_PROGRAM_BYTES / 16);
	int rc;

	mutex_lock(&tmatmul->lock);
	reset_status = tmatmul_rd32(tmatmul,
			tmatmul_inst_off(0, TMATMUL_INST_RST_STATUS));
	exec_status = tmatmul_rd32(tmatmul,
			tmatmul_inst_off(0, TMATMUL_INST_EXEC_STATUS));
	dma_status = tmatmul_rd32(tmatmul,
			tmatmul_inst_off(0, TMATMUL_INST_INSTR_START)) & 0xff;
	stall_status = tmatmul_rd32(tmatmul,
			tmatmul_inst_off(0, TMATMUL_INST_STALL_STATUS));
	instr_count = tmatmul_rd32(tmatmul,
			tmatmul_inst_off(0, TMATMUL_INST_DBG_INSTR_CNT));

	/*
	 * The start synchronizer uses a source-domain toggle. Resetting after an
	 * odd launch count clears only the destination toggle and fabricates a
	 * second start edge. After an even number of complete 8-slot programs,
	 * both toggles are zero again, so an instance reset is safe and rearms
	 * the engine for the next control-device session.
	 *
	 * Never reset a running, partial, errored, or otherwise ambiguous state.
	 * That preserves the fail-closed recovery boundary after a timeout.
	 */
	if (!reset_status && !exec_status &&
	    dma_status == TMATMUL_DMA_DONE && stall_status &&
	    instr_count && !(instr_count % rearm_period)) {
		tmatmul_wr32(tmatmul,
			     tmatmul_inst_off(0, TMATMUL_INST_RST_TRIGGER), 1);
		msleep(50);
		rc = readl_poll_timeout(tmatmul->csr +
				tmatmul_inst_off(0, TMATMUL_INST_DBG_INSTR_CNT),
				instr_count, !instr_count, 10, 100000);

		reset_status = tmatmul_rd32(tmatmul,
				tmatmul_inst_off(0, TMATMUL_INST_RST_STATUS));
		exec_status = tmatmul_rd32(tmatmul,
				tmatmul_inst_off(0, TMATMUL_INST_EXEC_STATUS));
		dma_status = tmatmul_rd32(tmatmul,
				tmatmul_inst_off(0, TMATMUL_INST_INSTR_START)) & 0xff;
		stall_status = tmatmul_rd32(tmatmul,
				tmatmul_inst_off(0, TMATMUL_INST_STALL_STATUS));
		if (rc || reset_status || exec_status ||
		    dma_status != TMATMUL_DMA_IDLE || stall_status) {
			dev_warn(&tmatmul->pdev->dev,
				 "tmatmul even-toggle rearm failed: rc=%d reset=%u exec=%u dma=%#x stall=%u instr=%u\n",
				 rc, reset_status, exec_status, dma_status,
				 stall_status, instr_count);
		} else {
			dev_dbg(&tmatmul->pdev->dev,
				"tmatmul rearmed after even-toggle session\n");
		}
	}
	mutex_unlock(&tmatmul->lock);
	return 0;
}

static const struct file_operations cxl_type2_tmatmul_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= cxl_type2_tmatmul_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
	.release	= cxl_type2_tmatmul_release,
	.llseek		= noop_llseek,
};

static void cxl_type2_tmatmul_misc_deregister(void *data)
{
	misc_deregister(data);
}

static int cxl_type2_tmatmul_cold_rearm(struct cxl_type2_tmatmul_dev *tmatmul)
{
	u32 dma_status, exec_status, instr_count, reset_status, stall_status;

	reset_status = tmatmul_rd32(tmatmul,
			tmatmul_inst_off(0, TMATMUL_INST_RST_STATUS));
	exec_status = tmatmul_rd32(tmatmul,
			tmatmul_inst_off(0, TMATMUL_INST_EXEC_STATUS));
	dma_status = tmatmul_rd32(tmatmul,
			tmatmul_inst_off(0, TMATMUL_INST_INSTR_START)) & 0xff;
	stall_status = tmatmul_rd32(tmatmul,
			tmatmul_inst_off(0, TMATMUL_INST_STALL_STATUS));
	instr_count = tmatmul_rd32(tmatmul,
			tmatmul_inst_off(0, TMATMUL_INST_DBG_INSTR_CNT));

	if (reset_status || exec_status || dma_status != TMATMUL_DMA_IDLE ||
	    stall_status || instr_count) {
		dev_err(&tmatmul->pdev->dev,
			"refusing probe-time tmatmul reset from non-cold state: reset=%u exec=%u dma=%#x stall=%u instr=%u\n",
			reset_status, exec_status, dma_status, stall_status,
			instr_count);
		return -EBUSY;
	}

	/*
	 * A host reboot can reset the visible CSR state without guaranteeing that
	 * every instance-local datapath has observed reset. Pulse the instance
	 * reset once before exposing the miscdevice, then wait long enough to catch
	 * a mismatched pulse-sync toggle replay. Registration is fail-closed unless
	 * the complete instance remains cold.
	 */
	tmatmul_wr32(tmatmul,
		     tmatmul_inst_off(0, TMATMUL_INST_RST_TRIGGER), 1);
	msleep(100);

	reset_status = tmatmul_rd32(tmatmul,
			tmatmul_inst_off(0, TMATMUL_INST_RST_STATUS));
	exec_status = tmatmul_rd32(tmatmul,
			tmatmul_inst_off(0, TMATMUL_INST_EXEC_STATUS));
	dma_status = tmatmul_rd32(tmatmul,
			tmatmul_inst_off(0, TMATMUL_INST_INSTR_START)) & 0xff;
	stall_status = tmatmul_rd32(tmatmul,
			tmatmul_inst_off(0, TMATMUL_INST_STALL_STATUS));
	instr_count = tmatmul_rd32(tmatmul,
			tmatmul_inst_off(0, TMATMUL_INST_DBG_INSTR_CNT));
	if (reset_status || exec_status || dma_status != TMATMUL_DMA_IDLE ||
	    stall_status || instr_count) {
		dev_err(&tmatmul->pdev->dev,
			"probe-time tmatmul reset did not remain cold: reset=%u exec=%u dma=%#x stall=%u instr=%u\n",
			reset_status, exec_status, dma_status, stall_status,
			instr_count);
		return -EIO;
	}

	dev_info(&tmatmul->pdev->dev,
		 "tmatmul instance cold-rearmed before userspace exposure\n");
	return 0;
}

static int cxl_type2_tmatmul_init_from(struct pci_dev *owner,
				       struct pci_dev *csr_pdev)
{
	struct cxl_type2_tmatmul_dev *tmatmul;
	resource_size_t bar0_start, bar0_len;
	const char *name;
	u32 dev_id;
	int rc;

	bar0_start = pci_resource_start(csr_pdev, 0);
	bar0_len = pci_resource_len(csr_pdev, 0);
	if (!bar0_start || bar0_len < TMATMUL_CSR_BASE + TMATMUL_CSR_SIZE) {
		dev_dbg(&owner->dev, "%s BAR0 too small for tmatmul CSR window\n",
			 pci_name(csr_pdev));
		return -ENODEV;
	}

	tmatmul = devm_kzalloc(&owner->dev, sizeof(*tmatmul), GFP_KERNEL);
	if (!tmatmul)
		return -ENOMEM;

	tmatmul->pdev = csr_pdev;
	tmatmul->csr = devm_ioremap(&owner->dev,
				    bar0_start + TMATMUL_CSR_BASE,
				    TMATMUL_CSR_SIZE);
	if (!tmatmul->csr)
		return -ENOMEM;

	dev_id = tmatmul_rd32(tmatmul, TMATMUL_REG_DEV_ID);
	if (dev_id != TMATMUL_DEV_ID) {
		dev_dbg(&owner->dev,
			 "%s has no tmatmul CSR at BAR0+0x%x (dev_id=0x%08x)\n",
			 pci_name(csr_pdev), TMATMUL_CSR_BASE, dev_id);
		return -ENODEV;
	}

	mutex_init(&tmatmul->lock);

	rc = cxl_type2_tmatmul_cold_rearm(tmatmul);
	if (rc)
		return rc;

	name = devm_kasprintf(&owner->dev, GFP_KERNEL,
			      "cxl_tmatmul%02x%02x%x", owner->bus->number,
			      PCI_SLOT(owner->devfn), PCI_FUNC(owner->devfn));
	if (!name)
		return -ENOMEM;

	tmatmul->miscdev.minor = MISC_DYNAMIC_MINOR;
	tmatmul->miscdev.name = name;
	tmatmul->miscdev.fops = &cxl_type2_tmatmul_fops;
	tmatmul->miscdev.parent = &owner->dev;

	rc = misc_register(&tmatmul->miscdev);
	if (rc) {
		dev_warn(&owner->dev, "failed to register tmatmul miscdev: %d\n",
			 rc);
		return rc;
	}

	rc = devm_add_action_or_reset(&owner->dev,
				      cxl_type2_tmatmul_misc_deregister,
				      &tmatmul->miscdev);
	if (rc)
		return rc;

	dev_info(&owner->dev,
		 "tmatmul ready: /dev/%s CSR=%s BAR0+0x%x\n",
		 name, pci_name(csr_pdev), TMATMUL_CSR_BASE);
	return 0;
}

static u32 cxl_type2_tmatmul_read_id(struct pci_dev *pdev)
{
	resource_size_t bar0_start = pci_resource_start(pdev, 0);
	resource_size_t bar0_len = pci_resource_len(pdev, 0);
	void __iomem *csr;
	u32 dev_id = 0;

	if (!bar0_start || bar0_len < TMATMUL_CSR_BASE + sizeof(dev_id))
		return 0;

	csr = ioremap(bar0_start + TMATMUL_CSR_BASE, sizeof(dev_id));
	if (!csr)
		return 0;
	dev_id = readl(csr);
	iounmap(csr);
	return dev_id;
}

static int cxl_type2_tmatmul_init(struct pci_dev *owner)
{
	struct pci_dev *sibling = NULL;
	struct pci_dev *provider = NULL;
	enum tmatmul_csr_source source;
	u32 current_id, sibling_id = 0;
	int rc;

	current_id = cxl_type2_tmatmul_read_id(owner);
	if (PCI_FUNC(owner->devfn) == 0) {
		sibling = pci_get_slot(owner->bus,
				       PCI_DEVFN(PCI_SLOT(owner->devfn), 1));
		if (sibling)
			sibling_id = cxl_type2_tmatmul_read_id(sibling);
	}

	source = tmatmul_csr_source_for_pair(current_id, sibling_id);
	if (source == TMATMUL_CSR_CURRENT)
		provider = owner;
	else if (source == TMATMUL_CSR_SIBLING)
		provider = sibling;
	else
		dev_info(&owner->dev,
			 "tmatmul CSR not present on PF0 or sibling PF1\n");

	rc = provider ? cxl_type2_tmatmul_init_from(owner, provider) : 0;
	if (sibling)
		pci_dev_put(sibling);

	return rc;
}

/**
 * DOC: CXL Type 2 Accelerator Driver
 *
 * This driver supports CXL Type 2 accelerator devices such as GPUs and
 * FPGAs that use the CXL.cache protocol for CPU-device coherency and
 * CXL.mem for device-attached memory.
 */

static int cxl_type2_setup_regs(struct pci_dev *pdev, enum cxl_regloc_type type,
				struct cxl_register_map *map)
{
	int rc;

	rc = cxl_find_regblock(pdev, type, map);
	if (rc)
		return rc;

	return cxl_setup_regs(map);
}

static int capcxl_read_identity(struct pci_dev *pf0, u64 *magic, u64 *caps)
{
	void __iomem *identity;

	if (pci_resource_len(pf0, 0) < CAPCXL_ID_OFFSET + CAPCXL_ID_MAP_SIZE)
		return -ENODEV;

	identity = pci_iomap_range(pf0, 0, CAPCXL_ID_OFFSET,
				  CAPCXL_ID_MAP_SIZE);
	if (!identity)
		return -ENOMEM;

	*magic = readq(identity);
	*caps = readq(identity + sizeof(*magic));
	pci_iounmap(pf0, identity);
	return 0;
}

static void capcxl_fill_pci_identity(struct pci_dev *pdev,
				     struct capcxl_pci_identity *identity)
{
	*identity = (struct capcxl_pci_identity) {
		.vendor = pdev->vendor,
		.device = pdev->device,
		.function = PCI_FUNC(pdev->devfn),
		.class_code = pdev->class,
		.revision = pdev->revision,
	};
}

static enum capcxl_role capcxl_detect_role(struct pci_dev *pdev,
					   struct pci_dev **pf0_out,
					   struct pci_dev **pf1_out)
{
	struct capcxl_pci_identity pf0_identity, pf1_identity;
	struct pci_dev *pf0, *pf1;
	enum capcxl_role role;
	u64 magic, caps;
	int rc;

	*pf0_out = NULL;
	*pf1_out = NULL;
	if (pdev->vendor != CAPCXL_VENDOR_ID ||
	    pdev->device != CAPCXL_DEVICE_ID)
		return CAPCXL_ROLE_NONE;

	pf0 = pci_get_slot(pdev->bus,
			   PCI_DEVFN(PCI_SLOT(pdev->devfn), 0));
	pf1 = pci_get_slot(pdev->bus,
			   PCI_DEVFN(PCI_SLOT(pdev->devfn), 1));
	if (!pf0 || !pf1)
		goto not_capcxl;

	/* Balance this temporary enable even when PF0 is already enabled. */
	rc = pci_enable_device_mem(pf0);
	if (rc)
		goto not_capcxl;
	rc = capcxl_read_identity(pf0, &magic, &caps);
	pci_disable_device(pf0);
	if (rc)
		goto not_capcxl;

	capcxl_fill_pci_identity(pf0, &pf0_identity);
	capcxl_fill_pci_identity(pf1, &pf1_identity);
	role = capcxl_role_for_pair(magic, caps, PCI_FUNC(pdev->devfn),
				    &pf0_identity, &pf1_identity);
	if (role == CAPCXL_ROLE_NONE)
		goto not_capcxl;

	*pf0_out = pf0;
	*pf1_out = pf1;
	return role;

not_capcxl:
	if (pf0)
		pci_dev_put(pf0);
	if (pf1)
		pci_dev_put(pf1);
	return CAPCXL_ROLE_NONE;
}

static int capcxl_require_rch(struct pci_dev *pdev)
{
	struct cxl_dport *dport = NULL;
	struct cxl_port *port;
	bool rch;

	port = cxl_pci_find_port(pdev, &dport);
	if (!port)
		return -EPROBE_DEFER;

	rch = dport && dport->rch;
	put_device(&port->dev);
	if (!rch) {
		dev_err(&pdev->dev,
			"CapCXL requires the RCH root topology; load cxl_acpi with rch_parent_uid=3\n");
		return -ENODEV;
	}

	return 0;
}

/*
 * IA-780I splits the Type-2 device across two functions: PF0 carries the
 * host-facing CXL stack plus the tmatmul CSRs in BAR0, and PF1 carries the AFU
 * data path including the CXL.mem responder.  This driver only binds PF0, but
 * the AFU on PF1 must have Memory Space + Bus Master enabled before the device
 * will acknowledge CXL.mem traffic to the HPA range advertised by the HDM
 * decoder.  Without PF1 enabled, userspace writes to the devdax mmap land on
 * an inert responder; on this platform that surfaces as an uncorrectable
 * error and brings the host down rather than returning an I/O error.
 * Enabling PF1 here removes the userspace ordering requirement.
 */
static void cxl_type2_enable_pf1(struct pci_dev *pf0)
{
	struct pci_dev *pf1;
	u16 cmd, want;

	pf1 = pci_get_slot(pf0->bus, PCI_DEVFN(PCI_SLOT(pf0->devfn), 1));
	if (!pf1) {
		dev_warn(&pf0->dev,
			 "PF1 not enumerated; AFU CXL.mem path will be inactive\n");
		return;
	}

	want = PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER |
	       PCI_COMMAND_PARITY | PCI_COMMAND_SERR;
	pci_read_config_word(pf1, PCI_COMMAND, &cmd);
	if ((cmd & want) != want) {
		pci_write_config_word(pf1, PCI_COMMAND, cmd | want);
		pci_read_config_word(pf1, PCI_COMMAND, &cmd);
	}

	dev_info(&pf0->dev, "PF1 %s enabled (COMMAND=0x%04x)\n",
		 pci_name(pf1), cmd);

	pci_dev_put(pf1);
}

/* Force-commit Decoder 0 before memdev registration triggers port probing. */
static int cxl_type2_force_commit_hdm(struct pci_dev *reg_pdev,
				      struct device *owner, u64 base_pa,
				      u64 size, bool hostonly)
{
	struct cxl_register_map comp_map = {};
	void __iomem *comp_base, *cap_base, *hdm_base = NULL;
	u64 rb_base, rb_size;
	u32 cap_hdr, global_ctrl, ctrl, lo, hi;
	int array_size, i, rc;

	rc = cxl_find_regblock(reg_pdev, CXL_REGLOC_RBI_COMPONENT, &comp_map);
	if (rc || comp_map.resource == CXL_RESOURCE_NONE)
		return rc ?: -ENODEV;

	comp_base = devm_ioremap(owner, comp_map.resource,
				 comp_map.max_size);
	if (IS_ERR_OR_NULL(comp_base))
		return -ENOMEM;

	/* Walk CXL.cm capability array (block + 0x1000) to find HDM cap (id=5). */
	cap_base = comp_base + 0x1000;
	cap_hdr = readl(cap_base);
	array_size = (cap_hdr >> 20) & 0xfff;
	for (i = 0; i < array_size && i < 32; i++) {
		u32 entry = readl(cap_base + 4 + i * 4);
		u16 cap_id = entry & 0xffff;
		u32 cap_off = (entry >> 20) & 0xfff;

		if (cap_id == 5) {
			hdm_base = cap_base + cap_off;
			break;
		}
	}
	if (!hdm_base) {
		dev_warn(owner,
			 "HDM Decoder capability not found in component regs\n");
		return -ENODEV;
	}

	/* Enable HDM decoding globally (RMW to preserve other bits). */
	global_ctrl = readl(hdm_base + CXL_HDM_DECODER_CTRL_OFFSET);
	if (!(global_ctrl & CXL_HDM_DECODER_ENABLE)) {
		writel(global_ctrl | CXL_HDM_DECODER_ENABLE,
		       hdm_base + CXL_HDM_DECODER_CTRL_OFFSET);
		global_ctrl = readl(hdm_base + CXL_HDM_DECODER_CTRL_OFFSET);
	}

	/*
	 * Program Decoder 0 base/size.
	 *
	 * Note: this HDM IP hardwires the HOSTONLY bit (BIT(12) of the per-
	 * decoder ctrl register) to 0, so we can't influence target_type via
	 * the HW path.  The fixup that flips target_type to HOSTONLYMEM lives
	 * in cxl_type2_fixup_classmem_target_type(), called after the memdev
	 * is registered.
	 */
	writel(lower_32_bits(base_pa),
	       hdm_base + CXL_HDM_DECODER0_BASE_LOW_OFFSET(0));
	writel(upper_32_bits(base_pa),
	       hdm_base + CXL_HDM_DECODER0_BASE_HIGH_OFFSET(0));
	writel(lower_32_bits(size),
	       hdm_base + CXL_HDM_DECODER0_SIZE_LOW_OFFSET(0));
	writel(upper_32_bits(size),
	       hdm_base + CXL_HDM_DECODER0_SIZE_HIGH_OFFSET(0));

	writel((hostonly ? CXL_HDM_DECODER0_CTRL_HOSTONLY : 0) |
	       CXL_HDM_DECODER0_CTRL_COMMIT,
	       hdm_base + CXL_HDM_DECODER0_CTRL_OFFSET(0));
	msleep(100);

	global_ctrl = readl(hdm_base + CXL_HDM_DECODER_CTRL_OFFSET);
	ctrl = readl(hdm_base + CXL_HDM_DECODER0_CTRL_OFFSET(0));
	lo = readl(hdm_base + CXL_HDM_DECODER0_BASE_LOW_OFFSET(0));
	hi = readl(hdm_base + CXL_HDM_DECODER0_BASE_HIGH_OFFSET(0));
	rb_base = ((u64)hi << 32) | lo;
	lo = readl(hdm_base + CXL_HDM_DECODER0_SIZE_LOW_OFFSET(0));
	hi = readl(hdm_base + CXL_HDM_DECODER0_SIZE_HIGH_OFFSET(0));
	rb_size = ((u64)hi << 32) | lo;

	dev_info(owner,
		 "HDM Decoder 0 force-commit readback: global_ctrl=0x%x ctrl=0x%x base=0x%llx size=0x%llx\n",
		 global_ctrl, ctrl, rb_base, rb_size);

	if (!(global_ctrl & CXL_HDM_DECODER_ENABLE) ||
	    !(ctrl & CXL_HDM_DECODER0_CTRL_COMMITTED) ||
	    (!!(ctrl & CXL_HDM_DECODER0_CTRL_HOSTONLY) != hostonly) ||
	    rb_base != base_pa || rb_size != size) {
		dev_err(owner,
			"HDM Decoder 0 did not latch requested range base=0x%llx size=0x%llx; refusing unsafe memdev registration\n",
			base_pa, size);
		return -EIO;
	}

	return 0;
}

static int capcxl_probe_type2(struct pci_dev *pdev)
{
	struct cxl_dev_state *cxlds;
	struct cxl_cachedev *cxlcd;
	int rc;

	rc = pcim_enable_device(pdev);
	if (rc)
		return rc;
	pci_set_master(pdev);

	rc = capcxl_require_rch(pdev);
	if (rc)
		return rc;

	cxlds = devm_kzalloc(&pdev->dev, sizeof(*cxlds), GFP_KERNEL);
	if (!cxlds)
		return -ENOMEM;

	cxlds->dev = &pdev->dev;
	cxlds->serial = pci_get_dsn(pdev);
	cxlds->cxl_dvsec = pci_find_dvsec_capability(
		pdev, PCI_VENDOR_ID_CXL, CXL_DVSEC_PCIE_DEVICE);
	cxlds->type = CXL_DEVTYPE_DEVMEM;
	cxlds->rcd = true;
	cxlds->media_ready = true;
	cxlds->reg_map.host = &pdev->dev;
	cxlds->reg_map.resource = CXL_RESOURCE_NONE;
	cxlds->cstate.size = 128 * SZ_1M;
	cxlds->cstate.unit = 64;
	cxlds->cstate.snoop_id = CXL_SNOOP_ID_NO_ID;
	cxlds->cstate.cache_id = CXL_CACHE_ID_NO_ID;
	pci_set_drvdata(pdev, cxlds);

	cxlcd = devm_cxl_add_cachedev(&pdev->dev, cxlds);
	if (IS_ERR(cxlcd))
		return dev_err_probe(&pdev->dev, PTR_ERR(cxlcd),
				     "failed to register CapCXL Type-2 cachedev\n");

	dev_info(&pdev->dev,
		 "CapCXL identity verified: PF0 initialized as Type-2 cachedev cache%d\n",
		 cxlcd->id);
	return 0;
}

static int capcxl_probe_type3(struct pci_dev *pdev, struct pci_dev *pf0)
{
	struct cxl_dpa_info dpa_info = {};
	struct cxl_memdev_state *mds;
	struct cxl_dev_state *cxlds;
	struct cxl_memdev *cxlmd;
	int rc;

	rc = pcim_enable_device(pdev);
	if (rc)
		return rc;
	pci_set_master(pdev);

	rc = capcxl_require_rch(pdev);
	if (rc)
		return rc;

	mds = cxl_memdev_state_create(&pdev->dev);
	if (IS_ERR(mds))
		return PTR_ERR(mds);
	cxlds = &mds->cxlds;
	cxlds->serial = pci_get_dsn(pdev);
	cxlds->cxl_dvsec = pci_find_dvsec_capability(
		pdev, PCI_VENDOR_ID_CXL, CXL_DVSEC_PCIE_DEVICE);
	cxlds->type = CXL_DEVTYPE_CLASSMEM;
	cxlds->rcd = true;
	cxlds->media_ready = true;

	rc = cxl_type2_setup_regs(pf0, CXL_REGLOC_RBI_COMPONENT,
				  &cxlds->reg_map);
	if (rc)
		return dev_err_probe(&pdev->dev, rc,
				     "PF0 shared component registers unavailable\n");
	if (!cxlds->reg_map.component_map.hdm_decoder.valid)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "PF0 shared component block has no HDM decoder\n");

	/* The physical locator is PF0; all persistent mappings belong to PF1. */
	cxlds->reg_map.host = &pdev->dev;
	cxlds->skip_dvsec_range_decode = true;
	cxlds->hostonly_hdm_decoder = true;
	mds->total_bytes = CAPCXL_MEMORY_SIZE;
	mds->volatile_only_bytes = CAPCXL_MEMORY_SIZE;
	mds->active_volatile_bytes = CAPCXL_MEMORY_SIZE;

	rc = cxl_mem_dpa_fetch(mds, &dpa_info);
	if (rc)
		return rc;
	rc = cxl_dpa_setup(cxlds, &dpa_info);
	if (rc)
		return rc;

	rc = cxl_type2_force_commit_hdm(pf0, &pdev->dev,
					CAPCXL_HPA_BASE, CAPCXL_MEMORY_SIZE,
					true);
	if (rc)
		return rc;

	pci_set_drvdata(pdev, cxlds);
	cxlmd = devm_cxl_add_memdev(&pdev->dev, cxlds);
	if (IS_ERR(cxlmd))
		return dev_err_probe(&pdev->dev, PTR_ERR(cxlmd),
				     "failed to register CapCXL Type-3 memdev\n");

	dev_info(&pdev->dev,
		 "CapCXL identity verified: PF1 initialized as Type-3 memdev mem%d (4 GiB volatile, shared PF0 HDM)\n",
		 cxlmd->id);
	return 0;
}

static int cxl_type2_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct pci_dev *capcxl_pf0, *capcxl_pf1;
	struct cxl_register_map map;
	struct cxl_memdev_state *mds;
	struct cxl_dev_state *cxlds;
	struct cxl_cachedev *cxlcd;
	struct cxl_memdev *cxlmd;
	enum cxl_type2_match_role match = id ? id->driver_data :
		CXL_TYPE2_MATCH_GENERIC;
	enum capcxl_role capcxl_role;
	int rc;
	u64 mapped_base, mapped_capacity;
	u16 dvsec;

	dev_info(&pdev->dev, "CXL Type 2 Accelerator driver probing function %d\n",
		 PCI_FUNC(pdev->devfn));

	capcxl_role = capcxl_detect_role(pdev, &capcxl_pf0, &capcxl_pf1);
	if (capcxl_role != CAPCXL_ROLE_NONE) {
		if (capcxl_role == CAPCXL_ROLE_TYPE2)
			rc = capcxl_probe_type2(pdev);
		else
			rc = capcxl_probe_type3(pdev, capcxl_pf0);

		pci_dev_put(capcxl_pf0);
		pci_dev_put(capcxl_pf1);
		return rc;
	}

	/*
	 * IA-780I PF1 advertises the CXL memory-device class, but the FPGA
	 * image exposes the CXL DVSEC/register-locator path through PF0. Bind
	 * PF1 here as the board-specific AFU memory function so it is not left
	 * driverless after generic cxl_pci rejects it for missing DVSEC. PF0
	 * remains responsible for CXL register discovery, memdev registration,
	 * and the tmatmul CSR miscdevice.
	 */
	if (match == CXL_TYPE2_MATCH_IA780I_MEM) {
		rc = pcim_enable_device(pdev);
		if (rc)
			return rc;
		pci_set_master(pdev);

		dev_info(&pdev->dev,
			 "IA-780I CXL memory-class PF bound; PF0 owns CXL DVSEC/register-locator handling\n");
		return 0;
	}

	if (match == CXL_TYPE2_MATCH_IA780I_ACCEL &&
	    PCI_FUNC(pdev->devfn) == 1) {
		rc = pcim_enable_device(pdev);
		if (rc)
			return rc;
		pci_set_master(pdev);
		dev_info(&pdev->dev,
			 "IA-780I tmatmul companion PF bound; PF0 owns CXL/DAX and the misc device\n");
		return 0;
	}

	if (PCI_FUNC(pdev->devfn) != 0)
		return -ENODEV;

	rc = pcim_enable_device(pdev);
	if (rc)
		return rc;
	pci_set_master(pdev);
	cxl_type2_enable_pf1(pdev);

	/* Allocate cxl_memdev_state (embeds cxl_dev_state at offset 0) */
	mds = devm_kzalloc(&pdev->dev, sizeof(*mds), GFP_KERNEL);
	if (!mds)
		return -ENOMEM;

	cxlds = &mds->cxlds;
	cxlds->dev = &pdev->dev;
	cxlds->serial = pci_get_dsn(pdev);
	cxlds->cxl_dvsec = pci_find_dvsec_capability(pdev, PCI_VENDOR_ID_CXL,
						      CXL_DVSEC_PCIE_DEVICE);
	cxlds->type = CXL_DEVTYPE_CLASSMEM;
	cxlds->media_ready = false;
	cxlds->dvsec_hdm_devmem = use_dvsec_hdm;

	/*
	 * Try to discover component registers with HDM decoder capability.
	 * If found, store in cxlds->reg_map so the endpoint probe uses
	 * real HDM decoder registers with hardware commit callbacks.
	 * Otherwise, fall back to DVSEC range emulation (CXL_RESOURCE_NONE).
	 */
	rc = cxl_type2_setup_regs(pdev, CXL_REGLOC_RBI_COMPONENT,
				  &cxlds->reg_map);
	if (rc || !cxlds->reg_map.component_map.hdm_decoder.valid) {
		dev_dbg(&pdev->dev,
			"No HDM decoder in component regs, using DVSEC emulation\n");
		cxlds->reg_map.host = &pdev->dev;
		cxlds->reg_map.resource = CXL_RESOURCE_NONE;
	} else {
		dev_info(&pdev->dev,
			 "HDM decoder found in component registers\n");
	}

	/* Detect RCH (Restricted CXL Host) / RCiEP topology */
	if (pci_pcie_type(pdev) == PCI_EXP_TYPE_RC_END) {
		cxlds->rcd = true;
		dev_info(&pdev->dev, "RCiEP detected, using RCH topology\n");
	}

	/* Check for CXL capabilities */
	dvsec = cxlds->cxl_dvsec;
	if (!dvsec &&
	    cxlds->reg_map.resource != CXL_RESOURCE_NONE &&
	    cxlds->reg_map.component_map.hdm_decoder.valid) {
		cxlds->skip_dvsec_range_decode = true;
		dev_warn(&pdev->dev,
			 "CXL Device DVSEC missing; using component HDM decoder registers\n");
	}
	if (dvsec) {
		u16 cap, ctrl;

			pci_read_config_word(pdev, dvsec + CXL_DVSEC_CAP_OFFSET, &cap);
			pci_read_config_word(pdev, dvsec + CXL_DVSEC_CTRL_OFFSET, &ctrl);

			dev_info(&pdev->dev, "CXL DVSEC: cap=0x%04x ctrl=0x%04x\n",
				 cap, ctrl);

			if (cxlds->reg_map.resource != CXL_RESOURCE_NONE &&
			    cxlds->reg_map.component_map.hdm_decoder.valid &&
			    (!(cap & CXL_DVSEC_MEM_CAPABLE) ||
			     !(cap & CXL_DVSEC_HDM_COUNT_MASK))) {
				cxlds->skip_dvsec_range_decode = true;
				dev_warn(&pdev->dev,
					 "CXL Device DVSEC lacks MemCapable/HDMCount; using component HDM decoder registers\n");
			}

			/*
			 * Enable all advertised CXL capabilities.
			 * Type 2 accelerators typically need CXL.cache for
		 * CPU-device coherency, CXL.io for MMIO, and CXL.mem
		 * for device-attached memory.
		 */

		/* CXL.cache */
		if (cap & CXL_DVSEC_CACHE_CAPABLE) {
			if (enable_cache) {
				ctrl |= CXL_DVSEC_CACHE_ENABLE;
				dev_info(&pdev->dev, "CXL.cache capable - enabling\n");
			} else {
				ctrl &= ~CXL_DVSEC_CACHE_ENABLE;
				dev_info(&pdev->dev,
					 "CXL.cache capable - disabled by enable_cache=0\n");
			}
		} else {
			dev_warn(&pdev->dev, "Device does not have CXL.cache capability\n");
		}

		/* CXL.io */
		if (cap & CXL_DVSEC_IO_CAPABLE) {
			ctrl |= CXL_DVSEC_IO_ENABLE;
			dev_info(&pdev->dev, "CXL.io capable\n");
		}

		/* CXL.mem */
		if (cap & CXL_DVSEC_MEM_CAPABLE) {
			ctrl |= CXL_DVSEC_MEM_ENABLE;
			dev_info(&pdev->dev, "CXL.mem capable\n");
		}

		/*
		 * CXL Viral - error containment.
		 * Per CXL 2.0 spec, viral prevents corrupted data from
		 * propagating when an uncorrectable error is detected.
		 * Force-enable on Type 2 accelerators even if CXLCap
		 * doesn't advertise it (e.g. QEMU device model) since
		 * the control bit may still be functional.
		 */
		ctrl |= CXL_DVSEC_VIRAL_ENABLE;

		/* Write all enables at once */
		pci_write_config_word(pdev, dvsec + CXL_DVSEC_CTRL_OFFSET, ctrl);

		/* Verify */
		pci_read_config_word(pdev, dvsec + CXL_DVSEC_CTRL_OFFSET, &ctrl);
		dev_info(&pdev->dev,
			 "CXL DVSEC ctrl=0x%04x: Cache%c IO%c Mem%c Viral%c\n",
			 ctrl,
			 (ctrl & CXL_DVSEC_CACHE_ENABLE) ? '+' : '-',
			 (ctrl & CXL_DVSEC_IO_ENABLE)    ? '+' : '-',
			 (ctrl & CXL_DVSEC_MEM_ENABLE)   ? '+' : '-',
			 (ctrl & CXL_DVSEC_VIRAL_ENABLE)  ? '+' : '-');
		/*
		 * Force-activate CXL.mem ranges.  On Type 2 devices the
		 * MEM_ACTIVE bit in each DVSEC Range Size Low register
		 * gates ip2hdm_reset_n inside the CXL IP.  Without a
		 * host-side CFMWS the normal cxl_mem flow never sets it,
		 * so we do it here.
		 */
		for (int i = 0; i < CXL_DVSEC_RANGE_MAX; i++) {
			u32 range_lo;

			pci_read_config_dword(pdev,
				dvsec + CXL_DVSEC_RANGE_SIZE_LOW(i),
				&range_lo);
			if (range_lo & CXL_DVSEC_MEM_INFO_VALID) {
				range_lo |= CXL_DVSEC_MEM_ACTIVE;
				pci_write_config_dword(pdev,
					dvsec + CXL_DVSEC_RANGE_SIZE_LOW(i),
					range_lo);
				pci_read_config_dword(pdev,
					dvsec + CXL_DVSEC_RANGE_SIZE_LOW(i),
					&range_lo);
				dev_info(&pdev->dev,
					 "DVSEC Range %d: mem_active=%d (size_lo=0x%08x)\n",
					 i, !!(range_lo & CXL_DVSEC_MEM_ACTIVE),
					 range_lo);
			}
		}
	} else {
		dev_warn(&pdev->dev, "No CXL DVSEC found\n");
	}

	/*
	 * Map RAS registers from component register block.
	 * If cxlds->reg_map already has valid component registers (HDM
	 * decoder path), reuse it. Otherwise do a separate discovery.
	 */
	if (cxlds->reg_map.resource != CXL_RESOURCE_NONE) {
		rc = cxl_map_component_regs(&cxlds->reg_map,
					    &cxlds->regs.component,
					    BIT(CXL_CM_CAP_CAP_ID_RAS));
		if (rc)
			dev_dbg(&pdev->dev,
				"RAS capability not available: %d\n", rc);
		else
			dev_info(&pdev->dev, "Component registers mapped\n");
	} else {
		struct cxl_register_map comp_map;

		rc = cxl_type2_setup_regs(pdev, CXL_REGLOC_RBI_COMPONENT,
					  &comp_map);
		if (rc) {
			dev_dbg(&pdev->dev,
				"Component register discovery: %d\n", rc);
		} else {
			rc = cxl_map_component_regs(&comp_map,
						    &cxlds->regs.component,
						    BIT(CXL_CM_CAP_CAP_ID_RAS));
			if (rc)
				dev_dbg(&pdev->dev,
					"RAS capability not available: %d\n",
					rc);
			else
				dev_info(&pdev->dev,
					 "Component registers mapped\n");
		}
	}

	/* Try to find and map device registers */
	rc = cxl_type2_setup_regs(pdev, CXL_REGLOC_RBI_MEMDEV, &map);
	if (!rc) {
		rc = cxl_map_device_regs(&map, &cxlds->regs.device_regs);
		if (!rc)
			dev_info(&pdev->dev, "Device registers mapped\n");
		else
			dev_dbg(&pdev->dev, "Device register mapping failed: %d\n", rc);
	} else {
		dev_dbg(&pdev->dev, "No device registers advertised (expected for Type 2)\n");
	}

	pci_set_drvdata(pdev, cxlds);

	if (enable_cache) {
		/* Set cache state defaults for Type 2 accelerator */
		cxlds->cstate.size = 128 * 1024 * 1024;  /* 128MB default cache */
		cxlds->cstate.unit = 64;                  /* 64-byte cache lines */
		cxlds->cstate.snoop_id = CXL_SNOOP_ID_NO_ID;
		cxlds->cstate.cache_id = CXL_CACHE_ID_NO_ID;
		dev_info(&pdev->dev, "Cache configured: %llu MB, %u-byte lines\n",
			 cxlds->cstate.size / (1024 * 1024), cxlds->cstate.unit);

		/* Register as a CXL cache device */
		cxlcd = devm_cxl_add_cachedev(&pdev->dev, cxlds);
		if (IS_ERR(cxlcd)) {
			rc = PTR_ERR(cxlcd);
			dev_info(&pdev->dev, "Cache device registration: %d\n", rc);
			/* Don't fail - device is still usable for CXL.mem / MMIO. */
		} else {
			dev_info(&pdev->dev, "CXL cache device registered successfully\n");
		}
	} else {
		dev_info(&pdev->dev,
			 "CXL.cache registration skipped (enable_cache=0; CXL-Secured uses CXL.mem)\n");
	}

	if (!enable_memdev) {
		dev_info(&pdev->dev,
			 "CXL memdev/HDM/DAX registration skipped (enable_memdev=0; CSR-only boot-safe mode)\n");
		rc = cxl_type2_tmatmul_init(pdev);
		if (rc)
			dev_warn(&pdev->dev, "tmatmul init failed: %d\n", rc);

		dev_info(&pdev->dev,
			 "CXL Type 2 Accelerator driver probed in CSR-only mode\n");
		return 0;
	}

	rc = cxl_type2_identify_capacity(mds, &mapped_base,
					 &mapped_capacity);
	if (rc) {
		dev_err(&pdev->dev,
			"CXL memdev/HDM/DAX registration skipped: endpoint Identify did not report usable capacity (%d)\n",
			rc);
		rc = cxl_type2_tmatmul_init(pdev);
		if (rc)
			dev_warn(&pdev->dev,
				 "tmatmul init failed: %d\n", rc);

		dev_info(&pdev->dev,
			 "CXL Type 2 Accelerator driver probed in CSR-only mode\n");
		return 0;
	}

	/* Set up DPA partitions (must happen before devm_cxl_add_memdev) */
	{
		struct cxl_dpa_info dpa_info = { 0 };

		rc = cxl_mem_dpa_fetch(mds, &dpa_info);
		if (rc)
			dev_dbg(&pdev->dev, "DPA fetch failed: %d\n", rc);
		else {
			rc = cxl_dpa_setup(cxlds, &dpa_info);
			if (rc)
				dev_dbg(&pdev->dev, "DPA setup failed: %d\n", rc);
		}
	}

	if (use_dvsec_hdm) {
		rc = type2_coalesce_soft_reserved(&pdev->dev, mapped_base,
						  mapped_capacity);
		if (rc) {
			dev_err(&pdev->dev,
				"CXL memdev/HDM/DAX registration skipped: native DVSEC resource ownership is unsafe (%d)\n",
				rc);
			rc = cxl_type2_tmatmul_init(pdev);
			if (rc)
				dev_warn(&pdev->dev,
					 "tmatmul init failed: %d\n", rc);

			dev_info(&pdev->dev,
				 "CXL Type 2 Accelerator driver probed in CSR-only mode\n");
			return 0;
		}
		dev_info(&pdev->dev,
			 "using active DVSEC range %#llx-%#llx for software Type-2 endpoint decoder\n",
			 mapped_base, mapped_base + mapped_capacity - 1);
	} else {
		/*
		 * Keep the hardware-commit path for platforms where the component
		 * decoder is writable and matches the host route.
		 */
		rc = cxl_type2_force_commit_hdm(pdev, &pdev->dev,
						type2_hpa_base,
						mapped_capacity, false);
		if (rc && !allow_uncommitted_hdm) {
			dev_err(&pdev->dev,
				"CXL memdev/HDM/DAX registration skipped after HDM commit failure (set allow_uncommitted_hdm=1 only for unsafe debug; do not online memory)\n");
			rc = cxl_type2_tmatmul_init(pdev);
			if (rc)
				dev_warn(&pdev->dev,
					 "tmatmul init failed: %d\n", rc);

			dev_info(&pdev->dev,
				 "CXL Type 2 Accelerator driver probed in CSR-only mode\n");
			return 0;
		}
		if (rc)
			dev_warn(&pdev->dev,
				 "allow_uncommitted_hdm=1: registering memdev despite failed HDM commit; do not online memory\n");
	}

	/* Register as a CXL memory device for decoder/region/DAX flow */
	cxlmd = devm_cxl_add_memdev(&pdev->dev, cxlds);
	if (IS_ERR(cxlmd)) {
		rc = PTR_ERR(cxlmd);
		dev_info(&pdev->dev, "Memory device registration: %d\n", rc);
	} else {
		dev_info(&pdev->dev, "CXL memory device registered (mem%d)\n",
			 cxlmd->id);
	}

	rc = cxl_type2_tmatmul_init(pdev);
	if (rc)
		dev_warn(&pdev->dev, "tmatmul init failed: %d\n", rc);

	dev_info(&pdev->dev, "CXL Type 2 Accelerator driver probed successfully\n");
	return 0;
}

static void cxl_type2_remove(struct pci_dev *pdev)
{
	dev_info(&pdev->dev, "CXL Type 2 Accelerator driver removed\n");
}

static const struct pci_device_id cxl_type2_pci_ids[] = {
	/* QEMU CXL Type 2 device */
	{ PCI_DEVICE(CXL_TYPE2_VENDOR_ID, CXL_TYPE2_DEVICE_ID_QEMU) },
	/* Intel IA-780i Agilex 7 CXL Type 2 accelerator PF */
	{
		.vendor = CXL_TYPE2_VENDOR_ID,
		.device = CXL_TYPE2_DEVICE_ID_IA780I,
		.subvendor = PCI_ANY_ID,
		.subdevice = PCI_ANY_ID,
		.class = PCI_CLASS_ACCELERATOR_PROCESSING << 8,
		.class_mask = 0xffffff,
		.driver_data = CXL_TYPE2_MATCH_IA780I_ACCEL,
	},
	/* Intel IA-780i Agilex 7 CXL memory-class AFU PF */
	{
		.vendor = CXL_TYPE2_VENDOR_ID,
		.device = CXL_TYPE2_DEVICE_ID_IA780I,
		.subvendor = PCI_ANY_ID,
		.subdevice = PCI_ANY_ID,
		.class = PCI_CLASS_MEMORY_CXL << 8 | CXL_MEMORY_PROGIF,
		.class_mask = 0xffffff,
		.driver_data = CXL_TYPE2_MATCH_IA780I_MEM,
	},
	{ }
};
MODULE_DEVICE_TABLE(pci, cxl_type2_pci_ids);

static struct pci_driver cxl_type2_driver = {
	.name		= "cxl_type2_accel",
	.id_table	= cxl_type2_pci_ids,
	.probe		= cxl_type2_probe,
	.remove		= cxl_type2_remove,
};

module_pci_driver(cxl_type2_driver);

MODULE_DESCRIPTION("CXL Type 2 Accelerator Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Victor Yang");
MODULE_IMPORT_NS("CXL");
