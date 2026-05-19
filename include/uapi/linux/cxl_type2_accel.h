/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_CXL_TYPE2_ACCEL_H
#define _UAPI_LINUX_CXL_TYPE2_ACCEL_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define CXL_TYPE2_TMATMUL_UAPI_VERSION	1

/*
 * Run the built-in smoke program:
 *   ldv input -> tmatmul import -> tmatmul go -> export -> sv output -> stall
 *
 * The driver writes a zero ternary matrix, a fixed-point 1.0 input vector, and
 * an output sentinel into CXL.mem before launching the instruction stream.
 */
#define CXL_TYPE2_TMATMUL_RUN_SMOKE		(1U << 0)

#define CXL_TYPE2_TMATMUL_RESULT_STALLED	(1U << 0)
#define CXL_TYPE2_TMATMUL_RESULT_OUTPUT_ZERO	(1U << 1)
#define CXL_TYPE2_TMATMUL_RESULT_DMA_ERROR	(1U << 2)

struct cxl_type2_tmatmul_info {
	__u32 version;
	__u32 dev_id;
	__u32 num_instances;
	__u32 dim_d;
	__u32 ddr_data_width;
	__u32 mc_status;
	__u32 reserved0;
	__u64 default_hpa_base;
	__u64 default_hpa_size;
	__u64 reserved1[4];
};

struct cxl_type2_tmatmul_run {
	/* Inputs. hpa_base/hpa_size may be zero to use module defaults. */
	__u64 hpa_base;
	__u64 hpa_size;
	__u32 timeout_ms;
	__u32 flags;

	/* Outputs, updated even when the ioctl returns an error. */
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
#define CXL_TYPE2_TMATMUL_RUN			\
	_IOWR(CXL_TYPE2_TMATMUL_IOC_MAGIC, 0x01, struct cxl_type2_tmatmul_run)

#endif /* _UAPI_LINUX_CXL_TYPE2_ACCEL_H */
