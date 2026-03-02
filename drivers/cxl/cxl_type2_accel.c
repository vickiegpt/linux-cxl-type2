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

#include "cxlmem.h"
#include "cxlpci.h"
#include "cxlcache.h"
#include "cxl.h"

#define CXL_TYPE2_VENDOR_ID	0x8086
#define CXL_TYPE2_DEVICE_ID_QEMU	0x0d92
#define CXL_TYPE2_DEVICE_ID_IA780I	0x0ddb

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

static int cxl_type2_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct cxl_register_map map;
	struct cxl_memdev_state *mds;
	struct cxl_dev_state *cxlds;
	struct cxl_cachedev *cxlcd;
	struct cxl_memdev *cxlmd;
	int rc;
	u16 dvsec;

	dev_info(&pdev->dev, "CXL Type 2 Accelerator driver probing\n");

	rc = pcim_enable_device(pdev);
	if (rc)
		return rc;
	pci_set_master(pdev);

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
	 * Initialize reg_map to CXL_RESOURCE_NONE so that
	 * cxl_port_setup_endpoint() propagates NONE to the endpoint
	 * port, triggering DVSEC range emulation in devm_cxl_setup_hdm()
	 * instead of looking for (nonexistent) HDM decoder registers.
	 */
	cxlds->reg_map.host = &pdev->dev;
	cxlds->reg_map.resource = CXL_RESOURCE_NONE;

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
	} else {
		dev_warn(&pdev->dev, "No CXL DVSEC found\n");
	}

	/*
	 * Find and map component registers for RAS into a local map.
	 *
	 * Do NOT store this in cxlds->reg_map. The QEMU Type 2 device
	 * exposes component registers (for RAS) but does not implement
	 * HDM decoder capability in them. If cxlds->reg_map is populated
	 * with these registers, cxl_port_setup_endpoint() will copy them
	 * to the endpoint port's reg_map, and devm_cxl_setup_hdm() will
	 * find component registers without HDM decoder and fail with
	 * -ENODEV instead of falling back to DVSEC range emulation.
	 *
	 * By keeping cxlds->reg_map at its default (CXL_RESOURCE_NONE),
	 * the endpoint probe takes the DVSEC range emulation path.
	 */
	{
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

	/* Set device memory size (16GB, matches DVSEC Range1) */
	mds->total_bytes = 16ULL * SZ_1G;
	mds->volatile_only_bytes = 16ULL * SZ_1G;
	mds->active_volatile_bytes = 16ULL * SZ_1G;

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
