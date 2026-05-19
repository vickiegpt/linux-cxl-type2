/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_CXL_TYPE2_ACCEL_H
#define _UAPI_LINUX_CXL_TYPE2_ACCEL_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define CXL_TYPE2_TMATMUL_UAPI_VERSION	2

/*
 * Result flags returned by CXL_TYPE2_TMATMUL_RUN_CSR_ONLY.
 *
 * STALLED is the success indicator: the device reached the trailing 'stall'
 * instruction of the user-provided program.  Userspace verifies the output
 * region of its own mmap to confirm the result.
 */
#define CXL_TYPE2_TMATMUL_RESULT_STALLED	(1U << 0)
#define CXL_TYPE2_TMATMUL_RESULT_DMA_ERROR	(1U << 2)

struct cxl_type2_tmatmul_info {
	__u32 version;
	__u32 dev_id;
	__u32 num_instances;
	__u32 dim_d;
	__u32 ddr_data_width;
	__u32 mc_status;
	__u32 reserved0;
	__u64 reserved1[2];	/* was default_hpa_base/_size in v1 */
	__u64 reserved2[4];
};

struct cxl_type2_tmatmul_csr_run {
	/* in */
	__u32 timeout_ms;
	__u32 flags;		/* reserved, must be 0 */

	/* out (updated on both success and error) */
	__u32 dma_status;
	__u32 stall_status;
	__u32 instr_count;
	__u32 dim_d;
	__u32 result_flags;
	__u32 reserved0;
	__u64 reserved1[4];
};

#define CXL_TYPE2_TMATMUL_IOC_MAGIC		0xCE
#define CXL_TYPE2_TMATMUL_GET_INFO		\
	_IOR(CXL_TYPE2_TMATMUL_IOC_MAGIC, 0x00, struct cxl_type2_tmatmul_info)
/* ioctl 0x01 was CXL_TYPE2_TMATMUL_RUN in v1; retired with no replacement. */
#define CXL_TYPE2_TMATMUL_RUN_CSR_ONLY		\
	_IOWR(CXL_TYPE2_TMATMUL_IOC_MAGIC, 0x02, struct cxl_type2_tmatmul_csr_run)

#endif /* _UAPI_LINUX_CXL_TYPE2_ACCEL_H */
