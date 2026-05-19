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

static int cxl_type2_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct cxl_register_map map;
	struct cxl_memdev_state *mds;
	struct cxl_dev_state *cxlds;
	struct cxl_cachedev *cxlcd;
	struct cxl_memdev *cxlmd;
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

	/* Set device memory size (4GB, matches QEMU mem-size parameter) */
	mds->total_bytes = 4ULL * SZ_1G;
	mds->volatile_only_bytes = 4ULL * SZ_1G;
	mds->active_volatile_bytes = 4ULL * SZ_1G;

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

	/*
	 * Force-commit HDM decoder 0 to release ip2hdm_reset_n on the
	 * device SIP.  On platforms where the BIOS CEDT has no CFMWS entry
	 * for this device's root port the standard cxl region flow cannot
	 * commit the endpoint decoder.  We work around this by directly
	 * programming the device's HDM decoder component registers.
	 *
	 * Component register block location is discovered from the Register
	 * Locator DVSEC.  The HDM decoder capability offset within the block
	 * is found by walking the CXL capability array header.
	 */
	{
		struct cxl_register_map comp_map = {};
		void __iomem *comp_base;

		rc = cxl_find_regblock(pdev, CXL_REGLOC_RBI_COMPONENT, &comp_map);
		if (!rc && comp_map.resource != CXL_RESOURCE_NONE) {
			comp_base = devm_ioremap(&pdev->dev,
					comp_map.resource, comp_map.max_size);
			if (!IS_ERR(comp_base)) {
				/* Walk CXL capability array to find HDM Decoder (id=5)
				 * CXL spec: capability header is at block + 0x1000 */
				void __iomem *cap_base = comp_base + 0x1000;
				void __iomem *hdm_base = NULL;
				u32 cap_hdr = readl(cap_base);
				int array_size = (cap_hdr >> 20) & 0xfff;
				int i;

				for (i = 0; i < array_size && i < 32; i++) {
					u32 entry = readl(cap_base + 4 + i * 4);
					u16 cap_id = entry & 0xffff;
					u32 cap_off = (entry >> 20) & 0xfff;
					/* HDM Decoder cap_id = 5 (CXL_CM_CAP_CAP_ID_HDM) */
					if (cap_id == 5) {
						hdm_base = cap_base + cap_off;
						break;
					}
				}

				if (hdm_base) {
					u32 global_ctrl, ctrl;
					u64 base_pa = 0x4080000000ULL;
					u64 size = mds->total_bytes;

					/*
					 * Enable HDM decoding globally before
					 * programming Decoder 0.  cxl_hdm_decode_init()
					 * does this in the canonical flow; without it
					 * Decoder 0 reads back as committed but the
					 * device does not actually decode CXL.mem
					 * traffic, and host writes to the HPA window
					 * fault on the device side.  Read-modify-write
					 * to preserve other bits in the global control.
					 */
					global_ctrl = readl(hdm_base + CXL_HDM_DECODER_CTRL_OFFSET);
					if (!(global_ctrl & CXL_HDM_DECODER_ENABLE)) {
						writel(global_ctrl | CXL_HDM_DECODER_ENABLE,
						       hdm_base + CXL_HDM_DECODER_CTRL_OFFSET);
						global_ctrl = readl(hdm_base + CXL_HDM_DECODER_CTRL_OFFSET);
					}

					/* DECODER0_BASE */
					writel(lower_32_bits(base_pa), hdm_base + 0x10);
					writel(upper_32_bits(base_pa), hdm_base + 0x14);
					/* DECODER0_SIZE */
					writel(lower_32_bits(size), hdm_base + 0x18);
					writel(upper_32_bits(size), hdm_base + 0x1C);
					/* DECODER0_CTRL: commit (bit 9) */
					writel(1 << 9, hdm_base + 0x20);
					msleep(100);

					ctrl = readl(hdm_base + 0x20);
					dev_info(&pdev->dev,
						 "HDM Decoder 0 force-committed: global_ctrl=0x%x ctrl=0x%x base=0x%llx size=0x%llx\n",
						 global_ctrl, ctrl, base_pa, size);
				} else {
					dev_warn(&pdev->dev,
						 "HDM Decoder capability not found in component regs\n");
				}
			}
		}
	}

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
