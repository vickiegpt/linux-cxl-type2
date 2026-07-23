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
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/uaccess.h>
#include <uapi/linux/cxl_type2_accel.h>

#include "cxlmem.h"
#include "cxlpci.h"
#include "cxlcache.h"
#include "cxl.h"

#define CXL_TYPE2_VENDOR_ID	0x8086
#define CXL_TYPE2_DEVICE_ID_QEMU	0x0d92
#define CXL_TYPE2_DEVICE_ID_IA780I	0x0ddb

#define TMATMUL_DEV_ID			0x544D4D31 /* "TMM1" */
#define TMATMUL_CSR_BASE		0x1C0000
#define TMATMUL_CSR_SIZE		0x2000
#define TMATMUL_INSTANCE_BASE		0x100
#define TMATMUL_INSTANCE_STRIDE		0x80

#define TMATMUL_REG_DEV_ID		0x000
#define TMATMUL_REG_NUM_INSTANCES	0x004
#define TMATMUL_REG_DIM_D		0x008
#define TMATMUL_REG_DDR_DATA_WIDTH	0x00C
#define TMATMUL_REG_MC_STATUS		0x018

#define TMATMUL_INST_STALL_STATUS	0x00
#define TMATMUL_INST_STALL_CLEAR	0x04
#define TMATMUL_INST_RST_TRIGGER	0x08
#define TMATMUL_INST_INSTR_SRC_LO	0x10
#define TMATMUL_INST_INSTR_SRC_HI	0x14
#define TMATMUL_INST_INSTR_LEN		0x18
#define TMATMUL_INST_INSTR_START	0x1C
#define TMATMUL_INST_DBG_INSTR_CNT	0x40

#define TMATMUL_DMA_IDLE		0x00
#define TMATMUL_DMA_RUNNING		0x01
#define TMATMUL_DMA_DONE		0x02
#define TMATMUL_DMA_ERROR		0xFF

/*
 * Userspace places its instruction program at this device-physical-address
 * offset within the dax window.  The driver only needs to point the CSR
 * fetch engine at it; the actual bytes are written by userspace.
 */
#define TMATMUL_DDR_INSTR_DPA		0x00300000ULL
#define TMATMUL_PROGRAM_BYTES		(6 * 16)

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
	int rc = 0;

	if (run->flags)
		return -EINVAL;

	num_instances = tmatmul_rd32(tmatmul, TMATMUL_REG_NUM_INSTANCES);
	if (!num_instances)
		return -ENODEV;

	run->dim_d = tmatmul_rd32(tmatmul, TMATMUL_REG_DIM_D);

	mutex_lock(&tmatmul->lock);

	tmatmul_wr32(tmatmul, tmatmul_inst_off(0, TMATMUL_INST_STALL_CLEAR), 1);
	tmatmul_wr32(tmatmul, tmatmul_inst_off(0, TMATMUL_INST_RST_TRIGGER), 1);
	msleep(50);

	tmatmul_wr32(tmatmul, tmatmul_inst_off(0, TMATMUL_INST_INSTR_SRC_LO),
		     lower_32_bits(TMATMUL_DDR_INSTR_DPA));
	tmatmul_wr32(tmatmul, tmatmul_inst_off(0, TMATMUL_INST_INSTR_SRC_HI),
		     upper_32_bits(TMATMUL_DDR_INSTR_DPA));
	tmatmul_wr32(tmatmul, tmatmul_inst_off(0, TMATMUL_INST_INSTR_LEN),
		     TMATMUL_PROGRAM_BYTES);
	tmatmul_wr32(tmatmul, tmatmul_inst_off(0, TMATMUL_INST_INSTR_START), 1);

	deadline = jiffies + msecs_to_jiffies(timeout_ms);
	do {
		run->dma_status = tmatmul_rd32(tmatmul,
			tmatmul_inst_off(0, TMATMUL_INST_INSTR_START)) & 0xff;
		run->stall_status = tmatmul_rd32(tmatmul,
			tmatmul_inst_off(0, TMATMUL_INST_STALL_STATUS));
		run->instr_count = tmatmul_rd32(tmatmul,
			tmatmul_inst_off(0, TMATMUL_INST_DBG_INSTR_CNT));

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

static long cxl_type2_tmatmul_ioctl(struct file *file, unsigned int cmd,
				    unsigned long arg)
{
	struct miscdevice *miscdev = file->private_data;
	struct cxl_type2_tmatmul_dev *tmatmul =
		container_of(miscdev, struct cxl_type2_tmatmul_dev, miscdev);
	void __user *argp = (void __user *)arg;
	struct cxl_type2_tmatmul_info info;
	struct cxl_type2_tmatmul_csr_run run;
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

	default:
		return -ENOTTY;
	}
}

static const struct file_operations cxl_type2_tmatmul_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= cxl_type2_tmatmul_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
	.llseek		= noop_llseek,
};

static void cxl_type2_tmatmul_misc_deregister(void *data)
{
	misc_deregister(data);
}

static int cxl_type2_tmatmul_init(struct pci_dev *pdev)
{
	struct cxl_type2_tmatmul_dev *tmatmul;
	resource_size_t bar0_start, bar0_len;
	const char *name;
	u32 dev_id;
	int rc;

	bar0_start = pci_resource_start(pdev, 0);
	bar0_len = pci_resource_len(pdev, 0);
	if (!bar0_start || bar0_len < TMATMUL_CSR_BASE + TMATMUL_CSR_SIZE) {
		dev_dbg(&pdev->dev, "BAR0 too small for tmatmul CSR window\n");
		return 0;
	}

	tmatmul = devm_kzalloc(&pdev->dev, sizeof(*tmatmul), GFP_KERNEL);
	if (!tmatmul)
		return -ENOMEM;

	tmatmul->pdev = pdev;
	tmatmul->csr = devm_ioremap(&pdev->dev, bar0_start + TMATMUL_CSR_BASE,
				    TMATMUL_CSR_SIZE);
	if (!tmatmul->csr)
		return -ENOMEM;

	dev_id = tmatmul_rd32(tmatmul, TMATMUL_REG_DEV_ID);
	if (dev_id != TMATMUL_DEV_ID) {
		dev_info(&pdev->dev,
			 "tmatmul CSR not present at BAR0+0x%x (dev_id=0x%08x)\n",
			 TMATMUL_CSR_BASE, dev_id);
		return 0;
	}

	mutex_init(&tmatmul->lock);

	name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "cxl_tmatmul%02x%02x%x",
			      pdev->bus->number, PCI_SLOT(pdev->devfn),
			      PCI_FUNC(pdev->devfn));
	if (!name)
		return -ENOMEM;

	tmatmul->miscdev.minor = MISC_DYNAMIC_MINOR;
	tmatmul->miscdev.name = name;
	tmatmul->miscdev.fops = &cxl_type2_tmatmul_fops;
	tmatmul->miscdev.parent = &pdev->dev;

	rc = misc_register(&tmatmul->miscdev);
	if (rc) {
		dev_warn(&pdev->dev, "failed to register tmatmul miscdev: %d\n",
			 rc);
		return 0;
	}

	rc = devm_add_action_or_reset(&pdev->dev,
				      cxl_type2_tmatmul_misc_deregister,
				      &tmatmul->miscdev);
	if (rc)
		return rc;

	dev_info(&pdev->dev,
		 "tmatmul ready: /dev/%s CSR=BAR0+0x%x\n",
		 name, TMATMUL_CSR_BASE);
	return 0;
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

static u64 cxl_type2_qemu_mem_size(struct pci_dev *pdev, int dvsec)
{
	u64 capacity = 0;
	int i;

	for (i = 0; i < CXL_DVSEC_RANGE_MAX; i++) {
		u64 size;
		u32 high, low;

		if (pci_read_config_dword(
			    pdev, dvsec + CXL_DVSEC_RANGE_SIZE_HIGH(i), &high) ||
		    pci_read_config_dword(
			    pdev, dvsec + CXL_DVSEC_RANGE_SIZE_LOW(i), &low))
			continue;
		if (!(low & CXL_DVSEC_MEM_INFO_VALID))
			continue;

		size = (u64)high << 32;
		size |= low & CXL_DVSEC_MEM_SIZE_LOW_MASK;
		capacity = max(capacity, size);
	}

	return capacity;
}

static int cxl_type2_match_root_decoder(struct device *dev, const void *data)
{
	const u64 *capacity = data;
	struct cxl_decoder *cxld;

	if (!is_root_decoder(dev))
		return 0;

	cxld = to_cxl_decoder(dev);
	return cxld->flags & CXL_DECODER_F_RAM &&
	       range_len(&cxld->hpa_range) >= *capacity;
}

static int cxl_type2_qemu_hdm_range(struct pci_dev *pdev, u64 capacity,
				    u64 *base, u64 *size)
{
	struct device *dev;
	struct cxl_decoder *cxld;

	dev = bus_find_device(&cxl_bus_type, NULL, &capacity,
			      cxl_type2_match_root_decoder);
	if (!dev)
		return -ENODEV;

	cxld = to_cxl_decoder(dev);
	*base = cxld->hpa_range.start;
	*size = min_t(u64, capacity, range_len(&cxld->hpa_range));
	dev_info(&pdev->dev, "using %s HPA range %#llx-%#llx\n",
		 dev_name(dev), *base, *base + *size - 1);
	put_device(dev);

	return 0;
}

/*
 * Force-commit Decoder 0 of the device's HDM block with the given HPA range.
 * Sets the HOSTONLY bit so the CXL core classifies the endpoint decoder as
 * CXL_DECODER_HOSTONLYMEM — matching the platform's CFMWS root decoder type.
 * Without HOSTONLY the endpoint decoder is treated as DEVMEM (2) and the
 * cxl_region_attach check rejects it against the HOSTONLYMEM (3) region.
 *
 * Must run BEFORE devm_cxl_add_memdev so the endpoint port probe sees the
 * corrected ctrl state on its first read of the decoder.
 */
static void cxl_type2_force_commit_hdm(struct pci_dev *pdev, u64 base_pa,
				       u64 size)
{
	struct cxl_register_map comp_map = {};
	void __iomem *comp_base, *cap_base, *hdm_base = NULL;
	u32 cap_hdr, global_ctrl, ctrl;
	int array_size, i, rc;

	rc = cxl_find_regblock(pdev, CXL_REGLOC_RBI_COMPONENT, &comp_map);
	if (rc || comp_map.resource == CXL_RESOURCE_NONE)
		return;

	comp_base = devm_ioremap(&pdev->dev, comp_map.resource,
				 comp_map.max_size);
	if (IS_ERR_OR_NULL(comp_base))
		return;

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
		dev_warn(&pdev->dev,
			 "HDM Decoder capability not found in component regs\n");
		return;
	}

	ctrl = readl(hdm_base + CXL_HDM_DECODER0_CTRL_OFFSET(0));
	if (ctrl & CXL_HDM_DECODER0_CTRL_COMMITTED) {
		u64 firmware_base;
		u64 firmware_size;

		firmware_base =
			(u64)readl(hdm_base +
				   CXL_HDM_DECODER0_BASE_HIGH_OFFSET(0)) << 32;
		firmware_base |= readl(
			hdm_base + CXL_HDM_DECODER0_BASE_LOW_OFFSET(0));
		firmware_size =
			(u64)readl(hdm_base +
				   CXL_HDM_DECODER0_SIZE_HIGH_OFFSET(0)) << 32;
		firmware_size |= readl(
			hdm_base + CXL_HDM_DECODER0_SIZE_LOW_OFFSET(0));
		dev_info(&pdev->dev,
			 "preserving firmware HDM Decoder 0: base=0x%llx size=0x%llx ctrl=0x%x\n",
			 firmware_base, firmware_size, ctrl);
		return;
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

	writel(CXL_HDM_DECODER0_CTRL_COMMIT,
	       hdm_base + CXL_HDM_DECODER0_CTRL_OFFSET(0));
	msleep(100);

	ctrl = readl(hdm_base + CXL_HDM_DECODER0_CTRL_OFFSET(0));
	dev_info(&pdev->dev,
		 "HDM Decoder 0 force-committed: global_ctrl=0x%x ctrl=0x%x base=0x%llx size=0x%llx\n",
		 global_ctrl, ctrl, base_pa, size);
}

static int cxl_type2_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct cxl_register_map map;
	struct cxl_memdev_state *mds;
	struct cxl_dev_state *cxlds;
	struct cxl_cachedev *cxlcd;
	struct cxl_memdev *cxlmd;
	u64 hdm_base = 0x180000000000ULL;
	u64 hdm_size = 4ULL * SZ_1G;
	int rc;
	u16 dvsec;

	dev_info(&pdev->dev, "CXL Type 2 Accelerator driver probing function %d\n",
		 PCI_FUNC(pdev->devfn));

	/*
	 * IA-780I function 1 hosts the AFU side. Do not bind or touch its
	 * BARs from this driver; PF1 setup is limited to explicit config-space
	 * writes from user space.
	 */
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
	cxlds->media_ready = true;

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
			ctrl |= CXL_DVSEC_CACHE_ENABLE;
			dev_info(&pdev->dev, "CXL.cache capable\n");
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
		/* Don't fail - device is still usable for MMIO testing */
	} else {
		dev_info(&pdev->dev, "CXL cache device registered successfully\n");
	}

	if (pdev->device == CXL_TYPE2_DEVICE_ID_QEMU) {
		hdm_size = cxl_type2_qemu_mem_size(pdev, dvsec);
		if (!hdm_size)
			return dev_err_probe(&pdev->dev, -ENODEV,
					     "QEMU DVSEC has no memory capacity\n");

		rc = cxl_type2_qemu_hdm_range(pdev, hdm_size, &hdm_base,
					      &hdm_size);
		if (rc)
			return dev_err_probe(&pdev->dev, rc,
					     "no suitable CFMWS root decoder\n");

	}

	mds->total_bytes = hdm_size;
	mds->volatile_only_bytes = hdm_size;
	mds->active_volatile_bytes = hdm_size;

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

	/*
	 * Force-commit HDM Decoder 0 BEFORE registering the memdev so the
	 * endpoint port probe (triggered inside devm_cxl_add_memdev) reads
	 * a non-zero HPA range from the device.
	 *
	 * Real IA-780I hardware retains its platform-specific default window.
	 * The QEMU device instead derives capacity from DVSEC and selects the
	 * first suitable CFMWS root decoder installed by ACPI.
	 */
	cxl_type2_force_commit_hdm(pdev, hdm_base, hdm_size);

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
	/* Intel IA-780i Agilex 7 CXL Type 2 */
	{ PCI_DEVICE(CXL_TYPE2_VENDOR_ID, CXL_TYPE2_DEVICE_ID_IA780I) },
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
