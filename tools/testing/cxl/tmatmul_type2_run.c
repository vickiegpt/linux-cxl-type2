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
#include <endian.h>
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
		if (snprintf(tmp, sizeof(tmp), "/sys/class/misc/%s/%s", base, out)
		    >= (int)sizeof(tmp))
			return -1;
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
			if (snprintf(region_path, sizeof(region_path),
				     "%s/%s", memdir, r->d_name)
			    >= (int)sizeof(region_path))
				continue;
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
				if (snprintf(drpath, sizeof(drpath),
					     "%s/%s", region_canon, dr->d_name)
				    >= (int)sizeof(drpath))
					continue;
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
