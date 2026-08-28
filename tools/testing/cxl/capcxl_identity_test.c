// SPDX-License-Identifier: GPL-2.0-only
#include <stdio.h>
#include <stdlib.h>

#include "../../../drivers/cxl/capcxl_identity.h"

static int failures;

static void expect_role(const char *name, enum capcxl_role actual,
			enum capcxl_role expected)
{
	if (actual == expected)
		return;

	fprintf(stderr, "FAIL: %s: got %d expected %d\n",
		name, actual, expected);
	failures++;
}

static enum capcxl_role role(unsigned long long magic,
			     unsigned long long caps,
			     unsigned int function,
			     struct capcxl_pci_identity pf0,
			     struct capcxl_pci_identity pf1)
{
	return capcxl_role_for_pair(magic, caps, function, &pf0, &pf1);
}

int main(void)
{
	const struct capcxl_pci_identity pf0 = {
		.vendor = 0x8086,
		.device = 0x0ddb,
		.function = 0,
		.class_code = 0x120000,
		.revision = 0x02,
	};
	const struct capcxl_pci_identity pf1 = {
		.vendor = 0x8086,
		.device = 0x0ddb,
		.function = 1,
		.class_code = 0x120000,
		.revision = 0x02,
	};
	struct capcxl_pci_identity bad;
	unsigned int bit;

	expect_role("PF0 exact match",
		role(CAPCXL_ID_VALUE, CAPCXL_CAPS_REQUIRED, 0, pf0, pf1),
		CAPCXL_ROLE_TYPE2);
	expect_role("PF1 exact match",
		role(CAPCXL_ID_VALUE, CAPCXL_CAPS_REQUIRED, 1, pf0, pf1),
		CAPCXL_ROLE_TYPE3);
	expect_role("future capability bits accepted",
		role(CAPCXL_ID_VALUE, CAPCXL_CAPS_REQUIRED | (1ULL << 12),
		     1, pf0, pf1), CAPCXL_ROLE_TYPE3);
	expect_role("bad magic",
		role(CAPCXL_ID_VALUE ^ 1ULL, CAPCXL_CAPS_REQUIRED,
		     0, pf0, pf1), CAPCXL_ROLE_NONE);

	for (bit = 0; bit < 4; bit++) {
		char name[64];

		snprintf(name, sizeof(name), "missing capability bit %u", bit);
		expect_role(name,
			role(CAPCXL_ID_VALUE,
			     CAPCXL_CAPS_REQUIRED & ~(1ULL << bit),
			     0, pf0, pf1), CAPCXL_ROLE_NONE);
	}

	bad = pf0;
	bad.vendor = 0x1234;
	expect_role("wrong PF0 vendor",
		role(CAPCXL_ID_VALUE, CAPCXL_CAPS_REQUIRED, 0, bad, pf1),
		CAPCXL_ROLE_NONE);
	bad = pf1;
	bad.device = 0xbeef;
	expect_role("wrong PF1 device",
		role(CAPCXL_ID_VALUE, CAPCXL_CAPS_REQUIRED, 1, pf0, bad),
		CAPCXL_ROLE_NONE);
	bad = pf0;
	bad.function = 1;
	expect_role("wrong PF0 function",
		role(CAPCXL_ID_VALUE, CAPCXL_CAPS_REQUIRED, 0, bad, pf1),
		CAPCXL_ROLE_NONE);
	bad = pf1;
	bad.class_code = 0x050210;
	expect_role("wrong PF1 class",
		role(CAPCXL_ID_VALUE, CAPCXL_CAPS_REQUIRED, 1, pf0, bad),
		CAPCXL_ROLE_NONE);
	bad = pf0;
	bad.revision = 0x01;
	expect_role("wrong PF0 revision",
		role(CAPCXL_ID_VALUE, CAPCXL_CAPS_REQUIRED, 0, bad, pf1),
		CAPCXL_ROLE_NONE);
	bad = pf1;
	bad.revision = 0x01;
	expect_role("wrong PF1 revision",
		role(CAPCXL_ID_VALUE, CAPCXL_CAPS_REQUIRED, 1, pf0, bad),
		CAPCXL_ROLE_NONE);
	expect_role("unsupported current function",
		role(CAPCXL_ID_VALUE, CAPCXL_CAPS_REQUIRED, 2, pf0, pf1),
		CAPCXL_ROLE_NONE);

	if (failures)
		return EXIT_FAILURE;

	puts("capcxl_identity_test: PASS");
	return EXIT_SUCCESS;
}
