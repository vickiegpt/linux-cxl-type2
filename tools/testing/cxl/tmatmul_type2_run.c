// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal user-space runner for the cxl_type2_accel tmatmul ioctl path.
 *
 * Build from the kernel tree:
 *   gcc -O2 -Wall -Wextra -D__EXPORTED_HEADERS__ -Iinclude/uapi \
 *       tools/testing/cxl/tmatmul_type2_run.c -o /tmp/tmatmul_type2_run
 *
 * Example for the IA-780I currently enumerated at ad:00:
 *   sudo /tmp/tmatmul_type2_run --apply-setpci
 */

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <linux/cxl_type2_accel.h>

#define DEFAULT_DEV		"/dev/cxl_tmatmulad000"
#define DEFAULT_HPA_BASE	0x4080000000ULL
#define DEFAULT_HPA_SIZE	0x100000000ULL
#define DEFAULT_PF1		"0000:ad:00.1"
#define DEFAULT_TIMEOUT_MS	10000U

static void usage(const char *prog)
{
	fprintf(stderr,
		"Usage: %s [options]\n"
		"\n"
		"Options:\n"
		"  --dev PATH              tmatmul misc device (default %s)\n"
		"  --hpa-base ADDR         CXL.mem host physical base (default 0x%llx)\n"
		"  --hpa-size SIZE         CXL.mem host physical size (default 0x%llx)\n"
		"  --timeout-ms MSEC       launch timeout (default %u)\n"
		"  --apply-setpci          run setpci -s PF1 COMMAND=0x0146 first\n"
		"  --pf1 BDF               PF1 config-space BDF (default %s)\n"
		"  --help                  show this help\n",
		prog, DEFAULT_DEV,
		(unsigned long long)DEFAULT_HPA_BASE,
		(unsigned long long)DEFAULT_HPA_SIZE,
		DEFAULT_TIMEOUT_MS, DEFAULT_PF1);
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
