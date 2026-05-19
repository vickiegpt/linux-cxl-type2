# tmatmul CSR-only ioctl + userspace devdax buffer — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rewrite the tmatmul smoke-test so the kernel never accesses CXL.mem. Userspace owns the buffer via a devdax mmap; the new `RUN_CSR_ONLY` ioctl only sequences CSRs.

**Architecture:** UAPI is bumped to v2 with a single new ioctl and the old `RUN` ioctl deleted entirely. The driver loses ~250 lines of CXL.mem-touching code. The userspace tool auto-discovers `/dev/daxN.M` for this device, mmaps the full window, lays out matrix/input/output/program at fixed DPAs, flushes CPU caches with `_mm_clflush`+`_mm_sfence`, kicks the ioctl, and verifies output in its own mmap.

**Tech Stack:** Linux 6.18-rc5 kernel (out-of-tree CXL Type-2 driver), C99, libc, `daxctl` utility (one-time setup), x86_64 SSE intrinsics (`<emmintrin.h>`/`<xmmintrin.h>`) for cacheline ops.

**Spec:** `docs/superpowers/specs/2026-05-19-tmatmul-csr-only-design.md`

---

## File structure

| File | Action | Responsibility |
|---|---|---|
| `include/uapi/linux/cxl_type2_accel.h` | rewrite | UAPI: GET_INFO ioctl, RUN_CSR_ONLY ioctl, types |
| `drivers/cxl/cxl_type2_accel.c` | edit | Kernel driver: PCI probe, miscdev, CSR-only RUN handler |
| `tools/testing/cxl/tmatmul_type2_run.c` | rewrite | Userspace tool: dax discover, mmap, layout, ioctl, verify |

No new files. No directory layout changes.

---

## Task 1: Replace UAPI header

**Files:**
- Rewrite: `include/uapi/linux/cxl_type2_accel.h`

- [ ] **Step 1: Rewrite the header**

Overwrite the entire file with:

```c
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
```

- [ ] **Step 2: Verify the header parses standalone**

Run:
```bash
cd /home/victoryang00/cxl && \
  gcc -fsyntax-only -c -x c - <<< '#include "include/uapi/linux/cxl_type2_accel.h"
int main(void){return 0;}' \
  -I include/uapi
```

Expected: no output (success). Any error: re-check the header for typos.

- [ ] **Step 3: Commit**

```bash
cd /home/victoryang00/cxl
git add include/uapi/linux/cxl_type2_accel.h
git commit -m "$(cat <<'EOF'
cxl/type2_accel: uapi v2 — drop legacy RUN, add RUN_CSR_ONLY

Bump CXL_TYPE2_TMATMUL_UAPI_VERSION to 2.  Delete the legacy RUN ioctl
(0x01) and its struct/flags.  Add RUN_CSR_ONLY (0x02) which sequences the
device's CSRs only; userspace owns the CXL.mem buffer via a devdax mmap.
Drop default_hpa_base/_size from the info struct (now reserved).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: Update kernel driver to RUN_CSR_ONLY

**Files:**
- Modify: `drivers/cxl/cxl_type2_accel.c`

- [ ] **Step 1: Remove the cacheflush include**

Delete the line:
```c
#include <asm/cacheflush.h>
```

(it's the last `#include` in the header block, just before `#include "cxlmem.h"`).

- [ ] **Step 2: Remove the hpa module parameters and their MODULE_PARM_DESC**

Find the block starting:
```c
static u64 tmatmul_hpa_base;
module_param(tmatmul_hpa_base, ullong, 0644);
...
static u64 tmatmul_hpa_size = 16ULL * SZ_1G;
module_param(tmatmul_hpa_size, ullong, 0644);
MODULE_PARM_DESC(tmatmul_hpa_size,
	"Default CXL.mem host physical window size for tmatmul ioctls");
```

Delete that entire block (lines 68–76 in the current file).

- [ ] **Step 3: Drop default_hpa_* fields from the device struct**

Replace:
```c
struct cxl_type2_tmatmul_dev {
	struct pci_dev *pdev;
	void __iomem *csr;
	struct miscdevice miscdev;
	struct mutex lock;
	u64 default_hpa_base;
	u64 default_hpa_size;
};
```

with:
```c
struct cxl_type2_tmatmul_dev {
	struct pci_dev *pdev;
	void __iomem *csr;
	struct miscdevice miscdev;
	struct mutex lock;
};
```

- [ ] **Step 4: Replace the address/program macro block**

Find:
```c
#define TMATMUL_DDR_MATRIX_ADDR		0x00000000ULL
#define TMATMUL_DDR_INPUT_ADDR		0x00100000ULL
#define TMATMUL_DDR_OUTPUT_ADDR		0x00200000ULL
#define TMATMUL_DDR_INSTR_ADDR		0x00300000ULL

#define TMATMUL_PROGRAM_BYTES		(6 * 16)
#define TMATMUL_MAX_MATRIX_BYTES	(64ULL * SZ_1M)
#define TMATMUL_UNCACHED_MEMREMAP_FLAGS	(MEMREMAP_WT | MEMREMAP_WC)
#define TMATMUL_WB_MEMREMAP_FLAGS	MEMREMAP_WB
```

Replace with just:
```c
/*
 * Userspace places its instruction program at this device-physical-address
 * offset within the dax window.  The driver only needs to point the CSR
 * fetch engine at it; the actual bytes are written by userspace.
 */
#define TMATMUL_DDR_INSTR_DPA		0x00300000ULL
#define TMATMUL_PROGRAM_BYTES		(6 * 16)
```

- [ ] **Step 5: Delete the CXL.mem helper functions**

Delete these functions in their entirety:
- `tmatmul_range_ok` (~12 lines)
- `tmatmul_memremap` (~20 lines)
- `tmatmul_flush_mapping` (~5 lines)
- `tmatmul_zero_hpa` (~16 lines)
- `tmatmul_pattern_hpa` (~16 lines)
- `tmatmul_write_hpa` (~16 lines)
- `tmatmul_fill_input_vector` (~21 lines)
- `tmatmul_output_is_zero` (~22 lines)
- `tmatmul_encode_instr` (~18 lines)
- `tmatmul_build_smoke_program` (~14 lines)
- `tmatmul_upload_smoke_payload` (~31 lines)
- `tmatmul_launch_smoke` (~79 lines)

All sit between the `tmatmul_inst_off` helper and the `cxl_type2_tmatmul_ioctl` function. Result: nothing between `tmatmul_inst_off` and the new `tmatmul_launch_csr_only` function added next step.

- [ ] **Step 6: Add the new CSR-only handler**

Right after `tmatmul_inst_off`, paste:

```c
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
```

- [ ] **Step 7: Replace the ioctl dispatcher body**

Find `cxl_type2_tmatmul_ioctl`. Replace the entire function body (everything between the opening `{` and closing `}`) with:

```c
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
```

- [ ] **Step 8: Update the tmatmul_init log message and drop default_hpa_* assignment**

Find inside `cxl_type2_tmatmul_init`:
```c
	mutex_init(&tmatmul->lock);
	tmatmul->default_hpa_base = tmatmul_hpa_base;
	tmatmul->default_hpa_size = tmatmul_hpa_size;
```

Replace with:
```c
	mutex_init(&tmatmul->lock);
```

Find the trailing `dev_info` call:
```c
	dev_info(&pdev->dev,
		 "tmatmul ready: /dev/%s CSR=BAR0+0x%x hpa_base=0x%llx hpa_size=0x%llx\n",
		 name, TMATMUL_CSR_BASE, tmatmul->default_hpa_base,
		 tmatmul->default_hpa_size);
```

Replace with:
```c
	dev_info(&pdev->dev,
		 "tmatmul ready: /dev/%s CSR=BAR0+0x%x\n",
		 name, TMATMUL_CSR_BASE);
```

- [ ] **Step 9: Build the module**

Run:
```bash
cd /home/victoryang00/cxl && make modules -j"$(nproc)" 2>&1 | tail -20
```

Expected: `LD [M] drivers/cxl/cxl_type2_accel.ko` and `BTF [M] drivers/cxl/cxl_type2_accel.ko` at the end, no errors. If a previously-built `cxl_type2_accel.o` is cached, it'll be rebuilt; you must see a `CC [M]` line for it.

If the build fails with "unused variable" or similar, re-check: every helper deleted in Step 5 must have no remaining callers, and every macro deleted in Step 4 must have no remaining references.

- [ ] **Step 10: Commit**

```bash
cd /home/victoryang00/cxl
git add drivers/cxl/cxl_type2_accel.c
git commit -m "$(cat <<'EOF'
cxl/type2_accel: drop CXL.mem helpers; add RUN_CSR_ONLY handler

Removes the smoke payload upload path (memremap + memset to a hard-coded
HPA) that was corrupting kernel RAM when the target range happened to be
promoted to System RAM via dax kmem.  The driver now only sequences the
device's CSRs; userspace is responsible for laying out the buffer in
device-attached memory through its own devdax mmap.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: Userspace — replace arg parsing and add dax discovery

**Files:**
- Modify: `tools/testing/cxl/tmatmul_type2_run.c`

This task replaces just the header includes, defaults, `usage()`, `parse_u64`, and adds discovery helpers. The `main()` body is left as-is from Task 2 (broken — won't compile yet, that's fine; Task 5 finishes it). To keep intermediate-state damage minimal we'll do Task 3, 4, and 5 in close succession.

- [ ] **Step 1: Rewrite the file's preamble**

Replace everything from the top of the file through (and including) the existing `parse_u64()` function with:

```c
// SPDX-License-Identifier: GPL-2.0
/*
 * Userspace runner for the cxl_type2_accel tmatmul RUN_CSR_ONLY ioctl.
 *
 * Build from the kernel tree:
 *   gcc -O2 -Wall -Wextra -msse2 -D__EXPORTED_HEADERS__ -Iinclude/uapi \
 *       tools/testing/cxl/tmatmul_type2_run.c -o /tmp/tmatmul_type2_run
 *
 * Pre-test setup (one-time per boot):
 *   sudo modprobe device_dax
 *   sudo daxctl reconfigure-device dax0.0 --mode=devdax --force
 *
 * Run:
 *   sudo /tmp/tmatmul_type2_run                 # auto-discover dax
 *   sudo /tmp/tmatmul_type2_run --dax /dev/dax0.0
 */

#define _GNU_SOURCE
#include <dirent.h>
#include <emmintrin.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <libgen.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <xmmintrin.h>

#include <linux/cxl_type2_accel.h>

#define DEFAULT_DEV		"/dev/cxl_tmatmulad000"
#define DEFAULT_TIMEOUT_MS	10000U

#define TMATMUL_DPA_MATRIX	0x00000000ULL
#define TMATMUL_DPA_INPUT	0x00100000ULL
#define TMATMUL_DPA_OUTPUT	0x00200000ULL
#define TMATMUL_DPA_PROGRAM	0x00300000ULL
#define TMATMUL_PROGRAM_BYTES	(6 * 16)

#define INPUT_Q88_ONE		0x0100	/* Q8.8 1.0 */
#define OUTPUT_SENTINEL		0xa5

static void usage(const char *prog)
{
	fprintf(stderr,
		"Usage: %s [options]\n"
		"\n"
		"Options:\n"
		"  --dev PATH        tmatmul misc device (default %s)\n"
		"  --dax PATH        devdax device for buffer (default: auto-discover)\n"
		"  --timeout-ms MSEC kernel poll timeout (default %u)\n"
		"  --help            show this help\n",
		prog, DEFAULT_DEV, DEFAULT_TIMEOUT_MS);
}

static uint64_t parse_u64(const char *s, const char *what)
{
	char *end = NULL;
	uint64_t val;

	errno = 0;
	val = strtoull(s, &end, 0);
	if (errno || !end || *end) {
		fprintf(stderr, "invalid %s: %s\n", what, s);
		exit(EXIT_FAILURE);
	}

	return val;
}
```

- [ ] **Step 2: Add dax discovery helpers**

Immediately after `parse_u64`, paste:

```c
static int read_sysfs_line(const char *path, char *buf, size_t bufsz)
{
	FILE *f = fopen(path, "r");
	if (!f)
		return -1;
	if (!fgets(buf, (int)bufsz, f)) {
		fclose(f);
		return -1;
	}
	fclose(f);
	/* strip trailing newline */
	size_t n = strlen(buf);
	while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r'))
		buf[--n] = '\0';
	return 0;
}

static int resolve_pci_path(const char *dev_path, char *out, size_t outsz)
{
	char link[PATH_MAX];
	const char *base = strrchr(dev_path, '/');
	base = base ? base + 1 : dev_path;
	snprintf(link, sizeof(link), "/sys/class/misc/%s/device", base);
	ssize_t n = readlink(link, out, outsz - 1);
	if (n < 0)
		return -1;
	out[n] = '\0';

	/* Resolve to canonical form. realpath() handles ../ chains. */
	char canon[PATH_MAX];
	if (out[0] != '/') {
		char tmp[PATH_MAX];
		snprintf(tmp, sizeof(tmp), "/sys/class/misc/%s/%s", base, out);
		if (!realpath(tmp, canon))
			return -1;
	} else {
		if (!realpath(out, canon))
			return -1;
	}
	if (strlen(canon) >= outsz)
		return -1;
	strcpy(out, canon);
	return 0;
}

/* Append "candidates" with /dev/daxN.M for each dax device whose backing
 * memdev is parented by pci_path.  Returns number found (-1 on error).
 * Caller passes in a pre-sized array.
 */
static int find_dax_for_pci(const char *pci_path,
			    char candidates[][PATH_MAX], int max)
{
	int n_found = 0;
	DIR *mem_d = opendir("/sys/bus/cxl/devices");
	if (!mem_d)
		return -1;
	struct dirent *e;
	while ((e = readdir(mem_d))) {
		if (strncmp(e->d_name, "mem", 3) != 0)
			continue;

		/* Confirm this memdev belongs to our PCI device. */
		char mem_pci[PATH_MAX], mem_canon[PATH_MAX];
		snprintf(mem_pci, sizeof(mem_pci),
			 "/sys/bus/cxl/devices/%s/device", e->d_name);
		if (!realpath(mem_pci, mem_canon))
			continue;
		if (strncmp(mem_canon, pci_path, strlen(pci_path)) != 0)
			continue;

		/* Walk regions that include this memdev. */
		char memdir[PATH_MAX];
		snprintf(memdir, sizeof(memdir),
			 "/sys/bus/cxl/devices/%s", e->d_name);
		DIR *region_d = opendir(memdir);
		if (!region_d)
			continue;
		struct dirent *r;
		while ((r = readdir(region_d))) {
			if (strncmp(r->d_name, "region", 6) != 0)
				continue;
			char region_path[PATH_MAX];
			snprintf(region_path, sizeof(region_path),
				 "%s/%s", memdir, r->d_name);
			char region_canon[PATH_MAX];
			if (!realpath(region_path, region_canon))
				continue;

			/* Under the region, find a dax_region child with a dax* device. */
			DIR *rdir = opendir(region_canon);
			if (!rdir)
				continue;
			struct dirent *dr;
			while ((dr = readdir(rdir))) {
				if (strncmp(dr->d_name, "dax_region", 10) != 0)
					continue;
				char drpath[PATH_MAX];
				snprintf(drpath, sizeof(drpath),
					 "%s/%s", region_canon, dr->d_name);
				DIR *dd = opendir(drpath);
				if (!dd)
					continue;
				struct dirent *de;
				while ((de = readdir(dd))) {
					if (strncmp(de->d_name, "dax", 3) != 0
					    || strchr(de->d_name, '.') == NULL)
						continue;
					if (n_found < max) {
						snprintf(candidates[n_found],
							 PATH_MAX, "/dev/%s",
							 de->d_name);
						n_found++;
					}
				}
				closedir(dd);
			}
			closedir(rdir);
		}
		closedir(region_d);
	}
	closedir(mem_d);
	return n_found;
}

static int discover_dax(const char *dev_path, char *out, size_t outsz)
{
	char pci_path[PATH_MAX];
	if (resolve_pci_path(dev_path, pci_path, sizeof(pci_path)) < 0) {
		fprintf(stderr,
			"could not resolve PCI device for %s: %s\n",
			dev_path, strerror(errno));
		return -1;
	}

	char candidates[8][PATH_MAX];
	int n = find_dax_for_pci(pci_path, candidates, 8);
	if (n < 0) {
		fprintf(stderr, "error walking sysfs: %s\n", strerror(errno));
		return -1;
	}
	if (n == 0) {
		fprintf(stderr,
			"no devdax device found for %s\n"
			"\n"
			"Required setup:\n"
			"    sudo modprobe device_dax\n"
			"    sudo daxctl reconfigure-device dax0.0 --mode=devdax --force\n"
			"    ls /dev/dax*\n",
			dev_path);
		return -1;
	}
	if (n > 1) {
		fprintf(stderr, "multiple devdax candidates:\n");
		for (int i = 0; i < n; i++)
			fprintf(stderr, "    %s\n", candidates[i]);
		fprintf(stderr, "pass --dax PATH to choose one\n");
		return -1;
	}
	if (strlen(candidates[0]) >= outsz)
		return -1;
	strcpy(out, candidates[0]);
	return 0;
}

static int read_dax_size(const char *dax_path, uint64_t *size_out)
{
	const char *base = strrchr(dax_path, '/');
	base = base ? base + 1 : dax_path;
	char sysfs[PATH_MAX];
	snprintf(sysfs, sizeof(sysfs),
		 "/sys/bus/dax/devices/%s/size", base);
	char line[64];
	if (read_sysfs_line(sysfs, line, sizeof(line)) < 0) {
		fprintf(stderr, "cannot read %s: %s\n", sysfs, strerror(errno));
		return -1;
	}
	*size_out = parse_u64(line, "dax size");
	return 0;
}
```

- [ ] **Step 3: Verify the tool still compiles (it won't yet — that's OK)**

The file still references `cxl_type2_tmatmul_info`, `cxl_type2_tmatmul_run`, `CXL_TYPE2_TMATMUL_GET_INFO`, `CXL_TYPE2_TMATMUL_RUN`, `CXL_TYPE2_TMATMUL_RUN_SMOKE`, etc. in the old `main()` body. We expect compile errors. That's fine — Task 5 rewrites `main()`.

For now just commit the partial work; it'll fail to link but be in a meaningful intermediate state.

Run:
```bash
cd /home/victoryang00/cxl && \
  gcc -O2 -Wall -Wextra -msse2 -D__EXPORTED_HEADERS__ -Iinclude/uapi \
      tools/testing/cxl/tmatmul_type2_run.c -o /tmp/tmatmul_type2_run 2>&1 | head -20
```

Expected: errors about undefined `cxl_type2_tmatmul_run`, `CXL_TYPE2_TMATMUL_RUN`, `CXL_TYPE2_TMATMUL_RUN_SMOKE`, etc. These come from the legacy `main()` body left in place — they'll be fixed in Task 5.

- [ ] **Step 4: Commit (intermediate state)**

```bash
cd /home/victoryang00/cxl
git add tools/testing/cxl/tmatmul_type2_run.c
git commit -m "$(cat <<'EOF'
tools/cxl/tmatmul_type2_run: add dax discovery helpers (WIP)

Replace argument parsing and add devdax auto-discovery walking
/sys/bus/cxl/devices/mem*/region*/dax_region*/dax*.  main() still
references the deleted v1 ioctl — compile is broken intentionally
until follow-up commits land the encoder and the new flow.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: Userspace — payload encoding and layout helpers

**Files:**
- Modify: `tools/testing/cxl/tmatmul_type2_run.c`

- [ ] **Step 1: Add the program encoder and layout helpers**

Immediately after `read_dax_size` from Task 3, paste:

```c
/*
 * Bit layout of one instruction word (mirrors kernel tmatmul_encode_instr).
 *
 *   word[0] = addr (little-endian)
 *   word[1] = bits[2:0]=rms, [4:3]=tm, [6:5]=ls,
 *             [9:7]=va,  [12:10]=vb, [15:13]=vy,
 *             [19:16]=op, [22:20]=fu
 */
static void encode_instr(uint8_t *dst, uint32_t fu, uint32_t op,
			 uint32_t vy, uint32_t vb, uint32_t va,
			 uint32_t ls, uint32_t tm, uint32_t rms,
			 uint64_t addr)
{
	uint64_t hi = 0;
	hi |= (uint64_t)(rms & 0x7);
	hi |= (uint64_t)(tm  & 0x3) << 3;
	hi |= (uint64_t)(ls  & 0x3) << 5;
	hi |= (uint64_t)(va  & 0x7) << 7;
	hi |= (uint64_t)(vb  & 0x7) << 10;
	hi |= (uint64_t)(vy  & 0x7) << 13;
	hi |= (uint64_t)(op  & 0xf) << 16;
	hi |= (uint64_t)(fu  & 0x7) << 20;

	uint64_t addr_le = htole64(addr);
	uint64_t hi_le   = htole64(hi);
	memcpy(dst + 0, &addr_le, 8);
	memcpy(dst + 8, &hi_le,   8);
}

static void encode_smoke_program(uint8_t prog[TMATMUL_PROGRAM_BYTES])
{
	encode_instr(prog + 0 * 16, 0x1, 0, 0, 0, 0, 0x1, 0, 0,
		     TMATMUL_DPA_INPUT);
	encode_instr(prog + 1 * 16, 0x3, 0, 0, 0, 0, 0,   0x1, 0, 0);
	encode_instr(prog + 2 * 16, 0x3, 0, 0, 0, 0, 0,   0x2, 0,
		     TMATMUL_DPA_MATRIX);
	encode_instr(prog + 3 * 16, 0x3, 0, 1, 1, 1, 0,   0x3, 0, 0);
	encode_instr(prog + 4 * 16, 0x1, 0, 1, 1, 1, 0x2, 0,   0,
		     TMATMUL_DPA_OUTPUT);
	encode_instr(prog + 5 * 16, 0x5, 0, 0, 0, 0, 0,   0,   0, 0);
}

static void fill_input_vector(uint8_t *base, uint32_t dim_d)
{
	uint16_t v = (uint16_t)htole16(INPUT_Q88_ONE);
	uint16_t *vec = (uint16_t *)base;
	for (uint32_t i = 0; i < dim_d; i++)
		vec[i] = v;
}

/* Evict the touched cachelines so the device will see the writes when it
 * fetches from CXL.mem.  Range is [start, start+len), aligned externally
 * to 64-byte boundaries by the caller.
 */
static void flush_range(volatile void *start, size_t len)
{
	const size_t LINE = 64;
	uintptr_t a = (uintptr_t)start & ~(LINE - 1);
	uintptr_t end = ((uintptr_t)start + len + LINE - 1) & ~(LINE - 1);
	for (; a < end; a += LINE)
		_mm_clflush((const void *)a);
	_mm_sfence();
}

/* Counterpart to flush_range for reads: clflush evicts any cached copy of
 * the line, so the next load comes from device memory.  lfence orders the
 * subsequent loads after the flush.
 */
static void invalidate_range(volatile void *start, size_t len)
{
	const size_t LINE = 64;
	uintptr_t a = (uintptr_t)start & ~(LINE - 1);
	uintptr_t end = ((uintptr_t)start + len + LINE - 1) & ~(LINE - 1);
	for (; a < end; a += LINE)
		_mm_clflush((const void *)a);
	_mm_lfence();
}
```

(Note: `htole16` / `htole64` come from `<endian.h>`. The existing `#include` list doesn't have it. Add it.)

- [ ] **Step 2: Add the endian.h include**

In the `#include` block near the top, add (after `<dirent.h>` for alphabetic order):

```c
#include <endian.h>
```

- [ ] **Step 3: Compile-check (still expected to fail on `main()`)**

Run:
```bash
cd /home/victoryang00/cxl && \
  gcc -O2 -Wall -Wextra -msse2 -D__EXPORTED_HEADERS__ -Iinclude/uapi \
      tools/testing/cxl/tmatmul_type2_run.c -o /tmp/tmatmul_type2_run 2>&1 | head -20
```

Expected: errors are still about the obsolete identifiers in `main()`. The new helper functions should produce no errors themselves. If you see errors about `_mm_clflush`, `_mm_sfence`, `_mm_lfence`: confirm `-msse2` is in CFLAGS and `<emmintrin.h>` is included (`<xmmintrin.h>` provides `_mm_sfence`; `<emmintrin.h>` provides `_mm_clflush` and `_mm_lfence`).

- [ ] **Step 4: Commit**

```bash
cd /home/victoryang00/cxl
git add tools/testing/cxl/tmatmul_type2_run.c
git commit -m "$(cat <<'EOF'
tools/cxl/tmatmul_type2_run: add encoder, layout, and clflush helpers (WIP)

Port the smoke program encoder verbatim from the kernel and add CPU cache
flush wrappers around _mm_clflush/_mm_sfence/_mm_lfence for moving data
between the host CPU caches and the device's CXL.mem responder.  main()
still references v1 — fixed in the next commit.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: Userspace — rewrite main flow

**Files:**
- Modify: `tools/testing/cxl/tmatmul_type2_run.c`

- [ ] **Step 1: Replace the print helpers and main**

Find `static void print_info(...)` and replace from there through the end of `main()` (i.e., everything below the helper functions you've added) with:

```c
static void print_info(const struct cxl_type2_tmatmul_info *info)
{
	printf("info:\n");
	printf("  version:        %u\n", info->version);
	printf("  dev_id:         0x%08x\n", info->dev_id);
	printf("  num_instances:  %u\n", info->num_instances);
	printf("  dim_d:          %u\n", info->dim_d);
	printf("  ddr_data_width: %u\n", info->ddr_data_width);
	printf("  mc_status:      0x%08x\n", info->mc_status);
}

static void print_run(const struct cxl_type2_tmatmul_csr_run *run)
{
	printf("run:\n");
	printf("  dim_d:        %u\n", run->dim_d);
	printf("  dma_status:   0x%02x\n", run->dma_status);
	printf("  stall_status: %u\n", run->stall_status);
	printf("  instr_count:  %u\n", run->instr_count);
	printf("  result_flags: 0x%08x\n", run->result_flags);
}

int main(int argc, char **argv)
{
	const char *dev = DEFAULT_DEV;
	const char *dax_override = NULL;
	uint32_t timeout_ms = DEFAULT_TIMEOUT_MS;
	struct cxl_type2_tmatmul_info info = {};
	struct cxl_type2_tmatmul_csr_run run = {};
	char dax_path[PATH_MAX];
	uint64_t dax_size = 0;
	int fd_dev = -1, fd_dax = -1, exit_rc = EXIT_FAILURE;
	void *base = MAP_FAILED;
	uint32_t dim_d;
	size_t matrix_len, vector_len, used_len;

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--dev") && i + 1 < argc) {
			dev = argv[++i];
		} else if (!strcmp(argv[i], "--dax") && i + 1 < argc) {
			dax_override = argv[++i];
		} else if (!strcmp(argv[i], "--timeout-ms") && i + 1 < argc) {
			timeout_ms = (uint32_t)parse_u64(argv[++i], "timeout-ms");
		} else if (!strcmp(argv[i], "--help")) {
			usage(argv[0]);
			return 0;
		} else {
			usage(argv[0]);
			return EXIT_FAILURE;
		}
	}

	/* 1. Locate the devdax window backing this tmatmul device. */
	if (dax_override) {
		if (strlen(dax_override) >= sizeof(dax_path)) {
			fprintf(stderr, "--dax path too long\n");
			return EXIT_FAILURE;
		}
		strcpy(dax_path, dax_override);
	} else if (discover_dax(dev, dax_path, sizeof(dax_path)) < 0) {
		return EXIT_FAILURE;
	}
	printf("dax_path: %s\n", dax_path);

	if (read_dax_size(dax_path, &dax_size) < 0)
		return EXIT_FAILURE;
	printf("dax_size: 0x%" PRIx64 "\n", dax_size);

	/* 2. Open the misc device and read static info. */
	fd_dev = open(dev, O_RDWR);
	if (fd_dev < 0) {
		perror(dev);
		return EXIT_FAILURE;
	}
	if (ioctl(fd_dev, CXL_TYPE2_TMATMUL_GET_INFO, &info)) {
		perror("CXL_TYPE2_TMATMUL_GET_INFO");
		goto out;
	}
	print_info(&info);
	dim_d = info.dim_d;
	if (!dim_d) {
		fprintf(stderr, "device reports dim_d=0\n");
		goto out;
	}

	matrix_len = (size_t)dim_d * dim_d / 4;
	vector_len = (size_t)dim_d * sizeof(uint16_t);
	used_len   = TMATMUL_DPA_PROGRAM + TMATMUL_PROGRAM_BYTES;

	if (used_len > dax_size) {
		fprintf(stderr,
			"dax window 0x%" PRIx64 " too small for layout 0x%zx\n",
			dax_size, used_len);
		goto out;
	}

	/* 3. mmap the full dax window. */
	fd_dax = open(dax_path, O_RDWR);
	if (fd_dax < 0) {
		perror(dax_path);
		goto out;
	}
	base = mmap(NULL, dax_size, PROT_READ | PROT_WRITE, MAP_SHARED,
		    fd_dax, 0);
	if (base == MAP_FAILED) {
		perror("mmap");
		goto out;
	}

	/* 4. Lay out matrix/input/output/program. */
	memset((char *)base + TMATMUL_DPA_MATRIX, 0, matrix_len);
	fill_input_vector((uint8_t *)base + TMATMUL_DPA_INPUT, dim_d);
	memset((char *)base + TMATMUL_DPA_OUTPUT, OUTPUT_SENTINEL, vector_len);
	encode_smoke_program((uint8_t *)base + TMATMUL_DPA_PROGRAM);

	/* 5. Flush the touched cachelines so the device sees the writes. */
	flush_range((char *)base + TMATMUL_DPA_MATRIX,  matrix_len);
	flush_range((char *)base + TMATMUL_DPA_INPUT,   vector_len);
	flush_range((char *)base + TMATMUL_DPA_OUTPUT,  vector_len);
	flush_range((char *)base + TMATMUL_DPA_PROGRAM, TMATMUL_PROGRAM_BYTES);

	/* 6. Kick the device. */
	run.timeout_ms = timeout_ms;
	int rc = ioctl(fd_dev, CXL_TYPE2_TMATMUL_RUN_CSR_ONLY, &run);
	int saved_errno = errno;
	print_run(&run);
	if (rc) {
		errno = saved_errno;
		perror("CXL_TYPE2_TMATMUL_RUN_CSR_ONLY");
		goto out;
	}
	if (!(run.result_flags & CXL_TYPE2_TMATMUL_RESULT_STALLED)) {
		fprintf(stderr, "device did not reach stall before timeout\n");
		goto out;
	}
	if (run.result_flags & CXL_TYPE2_TMATMUL_RESULT_DMA_ERROR) {
		fprintf(stderr, "device reported DMA error\n");
		goto out;
	}

	/* 7. Invalidate the output region in our cache, then verify. */
	invalidate_range((char *)base + TMATMUL_DPA_OUTPUT, vector_len);
	uint8_t *out_bytes = (uint8_t *)base + TMATMUL_DPA_OUTPUT;
	for (size_t i = 0; i < vector_len; i++) {
		if (out_bytes[i] != 0) {
			fprintf(stderr,
				"output non-zero at offset 0x%zx: 0x%02x\n",
				i, out_bytes[i]);
			goto out;
		}
	}

	printf("PASS: tmatmul smoke stalled cleanly; output buffer is zero\n");
	exit_rc = 0;

out:
	if (base != MAP_FAILED)
		munmap(base, dax_size);
	if (fd_dax >= 0)
		close(fd_dax);
	if (fd_dev >= 0)
		close(fd_dev);
	return exit_rc;
}
```

- [ ] **Step 2: Build the tool**

Run:
```bash
cd /home/victoryang00/cxl && \
  gcc -O2 -Wall -Wextra -msse2 -D__EXPORTED_HEADERS__ -Iinclude/uapi \
      tools/testing/cxl/tmatmul_type2_run.c -o /tmp/tmatmul_type2_run 2>&1
```

Expected: clean build, no output (gcc is silent on success). If unused variable warnings appear: fix the unused variable. If anything else: re-check the function definitions.

- [ ] **Step 3: Sanity check — tool runs without a device**

Run:
```bash
/tmp/tmatmul_type2_run --help
```

Expected: usage text printed; exit 0.

- [ ] **Step 4: Commit**

```bash
cd /home/victoryang00/cxl
git add tools/testing/cxl/tmatmul_type2_run.c
git commit -m "$(cat <<'EOF'
tools/cxl/tmatmul_type2_run: rewrite main flow on RUN_CSR_ONLY ioctl

Replace v1 main() with: auto-discover devdax, mmap full window, lay out
matrix/input/output/program at fixed DPAs, flush CPU cache with clflush
+ sfence, kick the kernel CSR-only ioctl, invalidate output region,
verify output is zero.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: End-to-end integration test

This task is not a code change but a controlled execution of the new path on the live FPGA. No commit unless something needs fixing.

**Files:**
- None modified

- [ ] **Step 1: Reload the kernel module**

```bash
sudo rmmod cxl_type2_accel 2>/dev/null
sudo insmod /home/victoryang00/cxl/drivers/cxl/cxl_type2_accel.ko
dmesg | tail -25
```

Expected dmesg lines:
- `cxl_type2_accel 0000:ad:00.0: CXL Type 2 Accelerator driver probing function 0`
- `cxl_type2_accel 0000:ad:00.0: PF1 0000:ad:00.1 enabled (COMMAND=0x0146)`
- `cxl_type2_accel 0000:ad:00.0: tmatmul ready: /dev/cxl_tmatmulad000 CSR=BAR0+0x1c0000` (no hpa_base/hpa_size now)
- `cxl_type2_accel 0000:ad:00.0: HDM Decoder 0 force-committed: global_ctrl=0x2 ctrl=0x600 base=0x4080000000 size=0x100000000`

If the log line still mentions `hpa_base=` you missed the Task 2 Step 8 edit.

- [ ] **Step 2: Configure devdax**

Check the current dax mode:
```bash
cat /sys/bus/cxl/devices/region0/mode 2>/dev/null
cat /sys/bus/cxl/devices/region13/mode 2>/dev/null
ls /dev/dax* 2>/dev/null
```

If no `/dev/dax*` exists, install daxctl and switch the region:
```bash
which daxctl || sudo apt install -y daxctl-utils
sudo modprobe device_dax
# pick the region backing mem0 — region0 in current setup; region13 if that's
# what your `discover_dax` matches.  Confirm via:
ls /sys/bus/cxl/devices/region0/dax_region* 2>/dev/null
sudo daxctl reconfigure-device dax0.0 --mode=devdax --force
ls /dev/dax*
```

Expected: at least one `/dev/daxN.M` after the reconfigure.

- [ ] **Step 3: Run the tool**

```bash
sudo /tmp/tmatmul_type2_run --timeout-ms 10000
```

Possible outcomes:

| Outcome | Meaning | Next |
|---|---|---|
| `PASS: tmatmul smoke stalled cleanly; output buffer is zero` | Success — both the new path and the device work | Commit any unexpected dmesg as a follow-up note, done |
| `device did not reach stall before timeout` / `ETIMEDOUT` | Kernel survived; device CSRs didn't progress | Inspect `dmesg`, check device's `stall_status` / `dma_status` from the tool's `print_run` output |
| `device reported DMA error` | Kernel survived; device DMA engine errored | Same — pivot to device-side investigation |
| `output non-zero at offset 0x%x: 0x%02x` | Smoke ran but result wrong | Same — investigate the FPGA-side path |
| Tool hangs on `msync`/output read | Kernel survived; CXL.mem responder broken | `kill -9` the tool; pivot to enabling AER + rasdaemon per earlier debug guidance |
| Host wedges, screen freezes / SSH dies | The new path is also crashing — back to Phase 1 of systematic-debugging | Capture AER/rasdaemon if previously enabled; otherwise see the systematic-debugging notes |

- [ ] **Step 4: Capture results**

Regardless of outcome, save the tool's stdout and the post-run dmesg tail somewhere (`/tmp/tmatmul-run-1.txt`, `/tmp/dmesg-tail-1.txt`). These are the inputs to the next investigation step if anything is off.

- [ ] **Step 5: If PASS, commit a CHANGELOG note (optional)**

If you want to document the green run in git:
```bash
cd /home/victoryang00/cxl
# (no files changed; create a note only if you have a CHANGELOG convention)
```

Otherwise skip; the plan's goal is achieved when Task 6 Step 3 produces PASS or a clean userspace-only failure that does not bring down the host.

---

## Self-review (post-write)

**Spec coverage:**

- UAPI section of spec ↔ Task 1 ✓
- Kernel driver section ↔ Task 2 ✓ (every deletion enumerated, every addition shown)
- Userspace section ↔ Tasks 3+4+5 (flags ↔ Task 3, auto-discovery ↔ Task 3, mmap full window ↔ Task 5 Step 1, encoding ↔ Task 4, flush+invalidate ↔ Task 4+5, error matrix ↔ Task 5 main flow)
- Pre-test setup ↔ Task 6 Step 2 ✓
- Testing matrix ↔ Task 6 Step 3 ✓
- Cache-flush ambiguity in spec ↔ resolved in Task 4 (`_mm_clflush` + `_mm_sfence`; `_mm_clflush` + `_mm_lfence` on read side) ✓

**Placeholder scan:** no TBDs, no "implement appropriately", no "similar to Task N" without code. Every code-touching step has the full code block.

**Type consistency:**

- `cxl_type2_tmatmul_csr_run` struct: declared in Task 1, used identically in Task 2 (kernel handler), Task 5 (userspace), Task 5 (print_run). ✓
- `CXL_TYPE2_TMATMUL_RUN_CSR_ONLY` ioctl number: Task 1 = `_IOWR(...,0x02,...)`, used in Task 2 Step 7 and Task 5 Step 1. ✓
- `TMATMUL_DDR_INSTR_DPA` (kernel) = `0x00300000`; `TMATMUL_DPA_PROGRAM` (userspace) = `0x00300000`. ✓
- `TMATMUL_PROGRAM_BYTES` (`6 * 16` = 96) defined the same way in both kernel (Task 2 Step 4) and userspace (Task 3 Step 1). ✓
- `encode_smoke_program`'s 6 instructions and field values match the kernel's `tmatmul_build_smoke_program` from the pre-rewrite driver. ✓
