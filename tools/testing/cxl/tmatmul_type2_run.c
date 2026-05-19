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

static const char *short_bdf(const char *bdf)
{
	if (!strncmp(bdf, "0000:", 5))
		return bdf + 5;
	return bdf;
}

static int run_setpci_pf1(const char *pf1)
{
	pid_t pid;
	int status;

	pid = fork();
	if (pid < 0) {
		perror("fork");
		return -1;
	}

	if (!pid) {
		execlp("setpci", "setpci", "-s", short_bdf(pf1),
		       "COMMAND=0x0146", (char *)NULL);
		perror("execlp(setpci)");
		_exit(127);
	}

	if (waitpid(pid, &status, 0) < 0) {
		perror("waitpid");
		return -1;
	}

	if (!WIFEXITED(status) || WEXITSTATUS(status)) {
		fprintf(stderr, "setpci failed for %s (status=0x%x)\n",
			pf1, status);
		return -1;
	}

	return 0;
}

static void print_info(const struct cxl_type2_tmatmul_info *info)
{
	printf("info:\n");
	printf("  version:          %u\n", info->version);
	printf("  dev_id:           0x%08x\n", info->dev_id);
	printf("  num_instances:    %u\n", info->num_instances);
	printf("  dim_d:            %u\n", info->dim_d);
	printf("  ddr_data_width:   %u\n", info->ddr_data_width);
	printf("  mc_status:        0x%08x\n", info->mc_status);
	printf("  default_hpa_base: 0x%016" PRIx64 "\n",
	       (uint64_t)info->default_hpa_base);
	printf("  default_hpa_size: 0x%016" PRIx64 "\n",
	       (uint64_t)info->default_hpa_size);
}

static void print_run(const struct cxl_type2_tmatmul_run *run)
{
	printf("run:\n");
	printf("  hpa_base:     0x%016" PRIx64 "\n", (uint64_t)run->hpa_base);
	printf("  hpa_size:     0x%016" PRIx64 "\n", (uint64_t)run->hpa_size);
	printf("  dim_d:        %u\n", run->dim_d);
	printf("  dma_status:   0x%02x\n", run->dma_status);
	printf("  stall_status: %u\n", run->stall_status);
	printf("  instr_count:  %u\n", run->instr_count);
	printf("  result_flags: 0x%08x\n", run->result_flags);
}

int main(int argc, char **argv)
{
	const char *dev = DEFAULT_DEV;
	const char *pf1 = DEFAULT_PF1;
	uint64_t hpa_base = DEFAULT_HPA_BASE;
	uint64_t hpa_size = DEFAULT_HPA_SIZE;
	uint32_t timeout_ms = DEFAULT_TIMEOUT_MS;
	bool apply_setpci = false;
	struct cxl_type2_tmatmul_info info = {};
	struct cxl_type2_tmatmul_run run = {};
	int fd, rc, saved_errno;

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--dev") && i + 1 < argc) {
			dev = argv[++i];
		} else if (!strcmp(argv[i], "--hpa-base") && i + 1 < argc) {
			hpa_base = parse_u64(argv[++i], "hpa-base");
		} else if (!strcmp(argv[i], "--hpa-size") && i + 1 < argc) {
			hpa_size = parse_u64(argv[++i], "hpa-size");
		} else if (!strcmp(argv[i], "--timeout-ms") && i + 1 < argc) {
			timeout_ms = parse_u64(argv[++i], "timeout-ms");
		} else if (!strcmp(argv[i], "--apply-setpci")) {
			apply_setpci = true;
		} else if (!strcmp(argv[i], "--pf1") && i + 1 < argc) {
			pf1 = argv[++i];
		} else if (!strcmp(argv[i], "--help")) {
			usage(argv[0]);
			return 0;
		} else {
			usage(argv[0]);
			return EXIT_FAILURE;
		}
	}

	if (apply_setpci) {
		printf("setpci: PF1 %s COMMAND=0x0146\n", pf1);
		if (run_setpci_pf1(pf1))
			return EXIT_FAILURE;
	}

	fd = open(dev, O_RDWR);
	if (fd < 0) {
		perror(dev);
		return EXIT_FAILURE;
	}

	if (ioctl(fd, CXL_TYPE2_TMATMUL_GET_INFO, &info)) {
		perror("CXL_TYPE2_TMATMUL_GET_INFO");
		close(fd);
		return EXIT_FAILURE;
	}
	print_info(&info);

	run.hpa_base = hpa_base;
	run.hpa_size = hpa_size;
	run.timeout_ms = timeout_ms;
	run.flags = CXL_TYPE2_TMATMUL_RUN_SMOKE;

	rc = ioctl(fd, CXL_TYPE2_TMATMUL_RUN, &run);
	saved_errno = errno;
	print_run(&run);
	close(fd);

	if (rc) {
		errno = saved_errno;
		perror("CXL_TYPE2_TMATMUL_RUN");
		return EXIT_FAILURE;
	}

	if (!(run.result_flags & CXL_TYPE2_TMATMUL_RESULT_STALLED) ||
	    !(run.result_flags & CXL_TYPE2_TMATMUL_RESULT_OUTPUT_ZERO)) {
		fprintf(stderr, "tmatmul smoke did not complete cleanly\n");
		return EXIT_FAILURE;
	}

	printf("PASS: tmatmul smoke completed and output buffer is zero\n");
	return 0;
}
