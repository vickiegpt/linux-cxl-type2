/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __TMATMUL_IDENTITY_H__
#define __TMATMUL_IDENTITY_H__

#define TMATMUL_ID_VALUE 0x544D4D31U /* "TMM1" */

enum tmatmul_csr_source {
	TMATMUL_CSR_NONE,
	TMATMUL_CSR_CURRENT,
	TMATMUL_CSR_SIBLING,
};

static inline enum tmatmul_csr_source
tmatmul_csr_source_for_pair(unsigned int current_id,
			    unsigned int sibling_id)
{
	if (current_id == TMATMUL_ID_VALUE)
		return TMATMUL_CSR_CURRENT;
	if (sibling_id == TMATMUL_ID_VALUE)
		return TMATMUL_CSR_SIBLING;
	return TMATMUL_CSR_NONE;
}

#endif /* __TMATMUL_IDENTITY_H__ */
