// SPDX-License-Identifier: GPL-2.0-only
/*
 * Zettai CXL Type-2 control driver.
 *
 * This binds the QEMU/Zettai 7a74:a123 PCI function and exposes a small
 * userspace ABI for the tmatmul CSR block and CXL.mem HPA requests. It is a
 * bring-up driver for CXLMemSim/QEMU experiments, not a full FMAPI switch CCI
 * implementation.
 */

#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/uaccess.h>
#include <uapi/linux/cxl_type2_accel.h>
#include <asm/cacheflush.h>

#define CXL_SWITCH_CCI_BAR 0

#define TMATMUL_DEV_ID                  0x544D4D31 /* "TMM1" */
#define TMATMUL_CSR_BASE                0x1C0000
#define TMATMUL_CSR_SIZE                0x2000
#define TMATMUL_INSTANCE_BASE           0x100
#define TMATMUL_INSTANCE_STRIDE         0x80

#define TMATMUL_REG_DEV_ID              0x000
#define TMATMUL_REG_NUM_INSTANCES       0x004
#define TMATMUL_REG_DIM_D               0x008
#define TMATMUL_REG_DDR_DATA_WIDTH      0x00C
#define TMATMUL_REG_MC_STATUS           0x018

#define TMATMUL_INST_STALL_STATUS       0x00
#define TMATMUL_INST_STALL_CLEAR        0x04
#define TMATMUL_INST_RST_TRIGGER        0x08
#define TMATMUL_INST_INSTR_SRC_LO       0x10
#define TMATMUL_INST_INSTR_SRC_HI       0x14
#define TMATMUL_INST_INSTR_LEN          0x18
#define TMATMUL_INST_INSTR_START        0x1C
#define TMATMUL_INST_DBG_INSTR_CNT      0x40

#define TMATMUL_DMA_ERROR               0xFF

#define TMATMUL_DDR_MATRIX_ADDR         0x00000000ULL
#define TMATMUL_DDR_INPUT_ADDR          0x00100000ULL
#define TMATMUL_DDR_OUTPUT_ADDR         0x00200000ULL
#define TMATMUL_DDR_INSTR_ADDR          0x00300000ULL

#define TMATMUL_PROGRAM_BYTES           (6 * 16)
#define TMATMUL_MAX_MATRIX_BYTES        (64ULL * SZ_1M)
#define TMATMUL_UNCACHED_MEMREMAP_FLAGS (MEMREMAP_WT | MEMREMAP_WC)
#define TMATMUL_WB_MEMREMAP_FLAGS       MEMREMAP_WB

static u64 zettai_cxlmem_hpa_base;
module_param(zettai_cxlmem_hpa_base, ullong, 0644);
MODULE_PARM_DESC(zettai_cxlmem_hpa_base,
		 "Default CXL.mem host physical base for Zettai ioctls");

static u64 zettai_cxlmem_hpa_size = 16ULL * SZ_1G;
module_param(zettai_cxlmem_hpa_size, ullong, 0644);
MODULE_PARM_DESC(zettai_cxlmem_hpa_size,
		 "Default CXL.mem host physical window size for Zettai ioctls");

struct cxl_switch_cci {
	struct pci_dev *pdev;
	void __iomem *regs;
	resource_size_t bar_len;
	void __iomem *tmatmul_csr;
	bool tmatmul_present;
	struct miscdevice miscdev;
	struct mutex lock; /* serializes tmatmul launch and CXL.mem access */
	u64 default_hpa_base;
	u64 default_hpa_size;
};

static void cxl_switch_cci_misc_deregister(void *data)
{
	misc_deregister(data);
}

static ssize_t zettai_switch_cci_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct cxl_switch_cci *swcci = pci_get_drvdata(pdev);

	return sysfs_emit(buf,
			  "vendor=0x%04x\n"
			  "device=0x%04x\n"
			  "bar0_size=0x%llx\n"
			  "tmatmul_present=%u\n"
			  "cxlmem_hpa_base=0x%llx\n"
			  "cxlmem_hpa_size=0x%llx\n"
			  "miscdev=%s\n",
			  pdev->vendor, pdev->device,
			  (unsigned long long)swcci->bar_len,
			  swcci->tmatmul_present,
			  (unsigned long long)swcci->default_hpa_base,
			  (unsigned long long)swcci->default_hpa_size,
			  swcci->miscdev.name ?: "none");
}
static DEVICE_ATTR_RO(zettai_switch_cci);

static u32 zettai_tmatmul_rd32(struct cxl_switch_cci *swcci, u32 off)
{
	if (!swcci->tmatmul_csr)
		return 0;

	return readl(swcci->tmatmul_csr + off);
}

static void zettai_tmatmul_wr32(struct cxl_switch_cci *swcci, u32 off, u32 val)
{
	writel(val, swcci->tmatmul_csr + off);
}

static u32 zettai_tmatmul_inst_off(u32 inst, u32 off)
{
	return TMATMUL_INSTANCE_BASE + inst * TMATMUL_INSTANCE_STRIDE + off;
}

static bool zettai_cxlmem_range_ok(u64 hpa_base, u64 hpa_size, u64 off, u64 len)
{
	if (!hpa_base || !hpa_size || !len)
		return false;
	if (off > hpa_size || len > hpa_size - off)
		return false;
	if (hpa_base + off < hpa_base)
		return false;
	if (hpa_base + off + len < hpa_base + off)
		return false;

	return true;
}

static void *zettai_cxlmem_memremap(u64 hpa_base, u64 hpa_size, u64 off,
				    size_t len, bool *needs_flush)
{
	void *addr;

	if (!zettai_cxlmem_range_ok(hpa_base, hpa_size, off, len))
		return NULL;

	if (needs_flush)
		*needs_flush = false;

	addr = memremap(hpa_base + off, len, TMATMUL_UNCACHED_MEMREMAP_FLAGS);
	if (addr)
		return addr;

	addr = memremap(hpa_base + off, len, TMATMUL_WB_MEMREMAP_FLAGS);
	if (addr && needs_flush)
		*needs_flush = true;

	return addr;
}

static void zettai_cxlmem_flush_mapping(void *addr, size_t len, bool needs_flush)
{
	if (needs_flush)
		clflush_cache_range(addr, len);
}

static void zettai_cxlmem_order_write(void)
{
	/* Ensure CXL.mem stores complete before device DMA observes data. */
	wmb();
}

static void zettai_cxlmem_order_read(void)
{
	/* Ensure CXL.mem data is visible before checking or copying it. */
	rmb();
}

static int zettai_cxlmem_zero_hpa(u64 hpa_base, u64 hpa_size, u64 off,
				  size_t len)
{
	void *addr;
	bool needs_flush;

	addr = zettai_cxlmem_memremap(hpa_base, hpa_size, off, len,
				      &needs_flush);
	if (!addr)
		return -ENOMEM;

	memset(addr, 0, len);
	zettai_cxlmem_order_write();
	zettai_cxlmem_flush_mapping(addr, len, needs_flush);
	zettai_cxlmem_order_write();
	memunmap(addr);
	return 0;
}

static int zettai_cxlmem_pattern_hpa(u64 hpa_base, u64 hpa_size, u64 off,
				     size_t len, u8 pattern)
{
	void *addr;
	bool needs_flush;

	addr = zettai_cxlmem_memremap(hpa_base, hpa_size, off, len,
				      &needs_flush);
	if (!addr)
		return -ENOMEM;

	memset(addr, pattern, len);
	zettai_cxlmem_order_write();
	zettai_cxlmem_flush_mapping(addr, len, needs_flush);
	zettai_cxlmem_order_write();
	memunmap(addr);
	return 0;
}

static int zettai_cxlmem_write_hpa(u64 hpa_base, u64 hpa_size, u64 off,
				   const void *src, size_t len)
{
	void *addr;
	bool needs_flush;

	addr = zettai_cxlmem_memremap(hpa_base, hpa_size, off, len,
				      &needs_flush);
	if (!addr)
		return -ENOMEM;

	memcpy(addr, src, len);
	zettai_cxlmem_order_write();
	zettai_cxlmem_flush_mapping(addr, len, needs_flush);
	zettai_cxlmem_order_write();
	memunmap(addr);
	return 0;
}

static int zettai_tmatmul_fill_input_vector(u64 hpa_base, u64 hpa_size,
					    u64 off, u32 dim_d)
{
	__le16 *vec;
	u32 i;
	size_t len = dim_d * sizeof(*vec);
	bool needs_flush;

	vec = zettai_cxlmem_memremap(hpa_base, hpa_size, off, len,
				     &needs_flush);
	if (!vec)
		return -ENOMEM;

	for (i = 0; i < dim_d; i++)
		vec[i] = cpu_to_le16(0x0100);

	zettai_cxlmem_order_write();
	zettai_cxlmem_flush_mapping(vec, len, needs_flush);
	zettai_cxlmem_order_write();
	memunmap(vec);
	return 0;
}

static int zettai_tmatmul_output_is_zero(u64 hpa_base, u64 hpa_size, u64 off,
					 size_t len)
{
	u8 *addr;
	size_t i;
	bool needs_flush;

	addr = zettai_cxlmem_memremap(hpa_base, hpa_size, off, len,
				      &needs_flush);
	if (!addr)
		return -ENOMEM;

	zettai_cxlmem_flush_mapping(addr, len, needs_flush);
	zettai_cxlmem_order_read();
	for (i = 0; i < len; i++) {
		if (addr[i]) {
			memunmap(addr);
			return -EIO;
		}
	}

	memunmap(addr);
	return 0;
}

static void zettai_tmatmul_encode_instr(u8 *dst, u32 fu, u32 op, u32 vy,
					u32 vb, u32 va, u32 ls, u32 tm,
					u32 rms, u64 addr)
{
	__le64 *word = (__le64 *)dst;
	u64 hi = 0;

	hi |= (u64)(rms & 0x7);
	hi |= (u64)(tm & 0x3) << 3;
	hi |= (u64)(ls & 0x3) << 5;
	hi |= (u64)(va & 0x7) << 7;
	hi |= (u64)(vb & 0x7) << 10;
	hi |= (u64)(vy & 0x7) << 13;
	hi |= (u64)(op & 0xf) << 16;
	hi |= (u64)(fu & 0x7) << 20;

	word[0] = cpu_to_le64(addr);
	word[1] = cpu_to_le64(hi);
}

static void zettai_tmatmul_build_smoke_program(u8 program[TMATMUL_PROGRAM_BYTES])
{
	u8 *p = program;

	zettai_tmatmul_encode_instr(p + 0 * 16, 0x1, 0, 0, 0, 0, 0x1, 0, 0,
				    TMATMUL_DDR_INPUT_ADDR);
	zettai_tmatmul_encode_instr(p + 1 * 16, 0x3, 0, 0, 0, 0, 0, 0x1, 0, 0);
	zettai_tmatmul_encode_instr(p + 2 * 16, 0x3, 0, 0, 0, 0, 0, 0x2, 0,
				    TMATMUL_DDR_MATRIX_ADDR);
	zettai_tmatmul_encode_instr(p + 3 * 16, 0x3, 0, 1, 1, 1, 0, 0x3, 0, 0);
	zettai_tmatmul_encode_instr(p + 4 * 16, 0x1, 0, 1, 1, 1, 0x2, 0, 0,
				    TMATMUL_DDR_OUTPUT_ADDR);
	zettai_tmatmul_encode_instr(p + 5 * 16, 0x5, 0, 0, 0, 0, 0, 0, 0, 0);
}

static int zettai_tmatmul_upload_smoke_payload(u64 hpa_base, u64 hpa_size,
					       u32 dim_d)
{
	u8 program[TMATMUL_PROGRAM_BYTES];
	u64 matrix_len;
	size_t vector_len;
	int rc;

	matrix_len = (u64)dim_d * dim_d / 4;
	vector_len = dim_d * sizeof(__le16);
	if (!dim_d || matrix_len > TMATMUL_MAX_MATRIX_BYTES)
		return -EINVAL;

	rc = zettai_cxlmem_zero_hpa(hpa_base, hpa_size, TMATMUL_DDR_MATRIX_ADDR,
				    matrix_len);
	if (rc)
		return rc;

	rc = zettai_tmatmul_fill_input_vector(hpa_base, hpa_size,
					      TMATMUL_DDR_INPUT_ADDR, dim_d);
	if (rc)
		return rc;

	rc = zettai_cxlmem_pattern_hpa(hpa_base, hpa_size,
				       TMATMUL_DDR_OUTPUT_ADDR, vector_len,
				       0xa5);
	if (rc)
		return rc;

	zettai_tmatmul_build_smoke_program(program);
	return zettai_cxlmem_write_hpa(hpa_base, hpa_size,
				       TMATMUL_DDR_INSTR_ADDR, program,
				       sizeof(program));
}

static int zettai_tmatmul_launch_smoke(struct cxl_switch_cci *swcci,
				       struct cxl_type2_tmatmul_run *run)
{
	unsigned long deadline;
	u64 hpa_base = run->hpa_base ?: swcci->default_hpa_base;
	u64 hpa_size = run->hpa_size ?: swcci->default_hpa_size;
	u32 timeout_ms = run->timeout_ms ?: 5000;
	u32 dim_d, num_instances;
	u32 instr_count_off;
	u32 instr_start_off;
	u32 stall_status_off;
	size_t output_len;
	int rc = 0;

	if (!swcci->tmatmul_present)
		return -ENODEV;
	if (!(run->flags & CXL_TYPE2_TMATMUL_RUN_SMOKE))
		return -EINVAL;

	num_instances = zettai_tmatmul_rd32(swcci, TMATMUL_REG_NUM_INSTANCES);
	if (!num_instances)
		return -ENODEV;

	dim_d = zettai_tmatmul_rd32(swcci, TMATMUL_REG_DIM_D);
	output_len = dim_d * sizeof(__le16);
	run->dim_d = dim_d;

	if (!zettai_cxlmem_range_ok(hpa_base, hpa_size,
				    TMATMUL_DDR_OUTPUT_ADDR, output_len))
		return -EINVAL;

	instr_count_off = zettai_tmatmul_inst_off(0, TMATMUL_INST_DBG_INSTR_CNT);
	instr_start_off = zettai_tmatmul_inst_off(0, TMATMUL_INST_INSTR_START);
	stall_status_off = zettai_tmatmul_inst_off(0, TMATMUL_INST_STALL_STATUS);

	mutex_lock(&swcci->lock);

	rc = zettai_tmatmul_upload_smoke_payload(hpa_base, hpa_size, dim_d);
	if (rc)
		goto out_unlock;

	zettai_tmatmul_wr32(swcci,
			    zettai_tmatmul_inst_off(0,
						    TMATMUL_INST_STALL_CLEAR),
			    1);
	zettai_tmatmul_wr32(swcci,
			    zettai_tmatmul_inst_off(0,
						    TMATMUL_INST_RST_TRIGGER),
			    1);
	msleep(50);

	zettai_tmatmul_wr32(swcci,
			    zettai_tmatmul_inst_off(0,
						    TMATMUL_INST_INSTR_SRC_LO),
			    lower_32_bits(TMATMUL_DDR_INSTR_ADDR));
	zettai_tmatmul_wr32(swcci,
			    zettai_tmatmul_inst_off(0,
						    TMATMUL_INST_INSTR_SRC_HI),
			    upper_32_bits(TMATMUL_DDR_INSTR_ADDR));
	zettai_tmatmul_wr32(swcci,
			    zettai_tmatmul_inst_off(0,
						    TMATMUL_INST_INSTR_LEN),
			    TMATMUL_PROGRAM_BYTES);
	zettai_tmatmul_wr32(swcci,
			    zettai_tmatmul_inst_off(0,
						    TMATMUL_INST_INSTR_START),
			    1);

	deadline = jiffies + msecs_to_jiffies(timeout_ms);
	do {
		run->dma_status = zettai_tmatmul_rd32(swcci,
						      instr_start_off) & 0xff;
		run->stall_status = zettai_tmatmul_rd32(swcci,
							stall_status_off);
		run->instr_count = zettai_tmatmul_rd32(swcci,
						       instr_count_off);

		if (run->stall_status) {
			run->result_flags |= CXL_TYPE2_TMATMUL_RESULT_STALLED;
			rc = zettai_tmatmul_output_is_zero(hpa_base, hpa_size,
							   TMATMUL_DDR_OUTPUT_ADDR,
							   output_len);
			if (!rc)
				run->result_flags |=
					CXL_TYPE2_TMATMUL_RESULT_OUTPUT_ZERO;
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
	mutex_unlock(&swcci->lock);
	return rc;
}

static int zettai_cxlmem_io(struct cxl_switch_cci *swcci,
			    struct cxl_type2_mem_req *req)
{
	u64 hpa_base = req->hpa_base ?: swcci->default_hpa_base;
	u64 hpa_size = req->hpa_size ?: swcci->default_hpa_size;
	void __user *user_ptr = (void __user *)(uintptr_t)req->user_ptr;
	void *addr;
	bool needs_flush;
	int rc = 0;

	if (!req->size || req->size > CXL_TYPE2_MEM_REQ_MAX_BYTES ||
	    req->flags || !user_ptr)
		return -EINVAL;

	addr = zettai_cxlmem_memremap(hpa_base, hpa_size, req->offset,
				      req->size, &needs_flush);
	if (!addr)
		return -ENOMEM;

	switch (req->op) {
	case CXL_TYPE2_MEM_REQ_READ:
		zettai_cxlmem_flush_mapping(addr, req->size, needs_flush);
		zettai_cxlmem_order_read();
		if (copy_to_user(user_ptr, addr, req->size))
			rc = -EFAULT;
		break;
	case CXL_TYPE2_MEM_REQ_WRITE:
		if (copy_from_user(addr, user_ptr, req->size)) {
			rc = -EFAULT;
			break;
		}
		zettai_cxlmem_order_write();
		zettai_cxlmem_flush_mapping(addr, req->size, needs_flush);
		zettai_cxlmem_order_write();
		break;
	default:
		rc = -EINVAL;
		break;
	}

	memunmap(addr);
	return rc;
}

static long cxl_switch_cci_ioctl(struct file *file, unsigned int cmd,
				 unsigned long arg)
{
	struct miscdevice *miscdev = file->private_data;
	struct cxl_switch_cci *swcci = container_of(miscdev,
						    struct cxl_switch_cci,
						    miscdev);
	void __user *argp = (void __user *)arg;
	struct cxl_type2_tmatmul_info info;
	struct cxl_type2_tmatmul_run run;
	struct cxl_type2_mem_req mem_req;
	int rc;

	switch (cmd) {
	case CXL_TYPE2_TMATMUL_GET_INFO:
		memset(&info, 0, sizeof(info));
		info.version = CXL_TYPE2_TMATMUL_UAPI_VERSION;
		info.dev_id = zettai_tmatmul_rd32(swcci, TMATMUL_REG_DEV_ID);
		if (swcci->tmatmul_present) {
			info.num_instances = zettai_tmatmul_rd32(swcci, TMATMUL_REG_NUM_INSTANCES);
			info.dim_d = zettai_tmatmul_rd32(swcci, TMATMUL_REG_DIM_D);
			info.ddr_data_width =
				zettai_tmatmul_rd32(swcci, TMATMUL_REG_DDR_DATA_WIDTH);
			info.mc_status = zettai_tmatmul_rd32(swcci, TMATMUL_REG_MC_STATUS);
		}
		info.default_hpa_base = swcci->default_hpa_base;
		info.default_hpa_size = swcci->default_hpa_size;
		if (copy_to_user(argp, &info, sizeof(info)))
			return -EFAULT;
		return 0;

	case CXL_TYPE2_TMATMUL_RUN:
		if (copy_from_user(&run, argp, sizeof(run)))
			return -EFAULT;
		run.dma_status = 0;
		run.stall_status = 0;
		run.instr_count = 0;
		run.result_flags = 0;
		rc = zettai_tmatmul_launch_smoke(swcci, &run);
		if (copy_to_user(argp, &run, sizeof(run)))
			return -EFAULT;
		return rc;

	case CXL_TYPE2_MEM_IO:
		if (copy_from_user(&mem_req, argp, sizeof(mem_req)))
			return -EFAULT;
		return zettai_cxlmem_io(swcci, &mem_req);

	default:
		return -ENOTTY;
	}
}

static const struct file_operations cxl_switch_cci_fops = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = cxl_switch_cci_ioctl,
	.compat_ioctl   = compat_ptr_ioctl,
	.llseek         = noop_llseek,
};

static int cxl_switch_cci_probe(struct pci_dev *pdev,
				const struct pci_device_id *id)
{
	struct cxl_switch_cci *swcci;
	const char *name;
	u32 tmatmul_dev_id = 0;
	int rc;

	rc = pcim_enable_device(pdev);
	if (rc)
		return rc;

	rc = pcim_iomap_regions(pdev, BIT(CXL_SWITCH_CCI_BAR),
				dev_name(&pdev->dev));
	if (rc)
		return rc;

	swcci = devm_kzalloc(&pdev->dev, sizeof(*swcci), GFP_KERNEL);
	if (!swcci)
		return -ENOMEM;

	swcci->pdev = pdev;
	swcci->regs = pcim_iomap_table(pdev)[CXL_SWITCH_CCI_BAR];
	swcci->bar_len = pci_resource_len(pdev, CXL_SWITCH_CCI_BAR);
	swcci->default_hpa_base = zettai_cxlmem_hpa_base;
	swcci->default_hpa_size = zettai_cxlmem_hpa_size;
	mutex_init(&swcci->lock);
	pci_set_drvdata(pdev, swcci);

	if (swcci->bar_len >= TMATMUL_CSR_BASE + TMATMUL_CSR_SIZE) {
		swcci->tmatmul_csr = swcci->regs + TMATMUL_CSR_BASE;
		tmatmul_dev_id = zettai_tmatmul_rd32(swcci,
						     TMATMUL_REG_DEV_ID);
		swcci->tmatmul_present = tmatmul_dev_id == TMATMUL_DEV_ID;
	}

	name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "zettai_cxl%02x%02x%x",
			      pdev->bus->number, PCI_SLOT(pdev->devfn),
			      PCI_FUNC(pdev->devfn));
	if (!name)
		return -ENOMEM;

	swcci->miscdev.minor = MISC_DYNAMIC_MINOR;
	swcci->miscdev.name = name;
	swcci->miscdev.fops = &cxl_switch_cci_fops;
	swcci->miscdev.parent = &pdev->dev;

	rc = misc_register(&swcci->miscdev);
	if (rc)
		return rc;

	rc = devm_add_action_or_reset(&pdev->dev,
				      cxl_switch_cci_misc_deregister,
				      &swcci->miscdev);
	if (rc)
		return rc;

	rc = device_create_file(&pdev->dev, &dev_attr_zettai_switch_cci);
	if (rc)
		return rc;

	dev_info(&pdev->dev,
		 "Zettai CXL device ready: /dev/%s BAR0=0x%llx tmatmul=%u dev_id=0x%08x hpa_base=0x%llx hpa_size=0x%llx\n",
		 name, (unsigned long long)swcci->bar_len,
		 swcci->tmatmul_present, tmatmul_dev_id,
		 (unsigned long long)swcci->default_hpa_base,
		 (unsigned long long)swcci->default_hpa_size);

	return 0;
}

static void cxl_switch_cci_remove(struct pci_dev *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_zettai_switch_cci);
}

static const struct pci_device_id cxl_switch_cci_pci_tbl[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_ZETTAI,
		     PCI_DEVICE_ID_ZETTAI_CXL_SWITCH_CCI) },
	{ }
};
MODULE_DEVICE_TABLE(pci, cxl_switch_cci_pci_tbl);

static struct pci_driver cxl_switch_cci_driver = {
	.name = "cxl_switch_cci",
	.id_table = cxl_switch_cci_pci_tbl,
	.probe = cxl_switch_cci_probe,
	.remove = cxl_switch_cci_remove,
};

module_pci_driver(cxl_switch_cci_driver);

MODULE_DESCRIPTION("Zettai CXL Type-2 control driver");
MODULE_LICENSE("GPL");
