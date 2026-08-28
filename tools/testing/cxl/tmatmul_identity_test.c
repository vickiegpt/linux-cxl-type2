// SPDX-License-Identifier: GPL-2.0-only
#include <stdio.h>
#include <stdlib.h>

#include "../../../drivers/cxl/tmatmul_identity.h"

static int failures;

static void expect_source(const char *name,
                          enum tmatmul_csr_source actual,
                          enum tmatmul_csr_source expected)
{
    if (actual == expected)
        return;

    fprintf(stderr, "FAIL: %s: got %d expected %d\n",
            name, actual, expected);
    failures++;
}

int main(void)
{
    expect_source("current PF owns TMM1",
                  tmatmul_csr_source_for_pair(TMATMUL_ID_VALUE, 0),
                  TMATMUL_CSR_CURRENT);
    expect_source("sibling PF owns TMM1",
                  tmatmul_csr_source_for_pair(0, TMATMUL_ID_VALUE),
                  TMATMUL_CSR_SIBLING);
    expect_source("current PF wins when both advertise TMM1",
                  tmatmul_csr_source_for_pair(TMATMUL_ID_VALUE,
                                              TMATMUL_ID_VALUE),
                  TMATMUL_CSR_CURRENT);
    expect_source("non-tmatmul pair is rejected",
                  tmatmul_csr_source_for_pair(0x43415043, 0),
                  TMATMUL_CSR_NONE);

    if (failures)
        return EXIT_FAILURE;

    puts("tmatmul_identity_test: PASS");
    return EXIT_SUCCESS;
}
