/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __CAPCXL_IDENTITY_H__
#define __CAPCXL_IDENTITY_H__

#define CAPCXL_ID_VALUE		0x43415043584c0001ULL

#define CAPCXL_CAP_PF0_TYPE2	(1ULL << 0)
#define CAPCXL_CAP_PF1_TYPE3	(1ULL << 1)
#define CAPCXL_CAP_SHARED_COMPONENT_REGS	(1ULL << 2)
#define CAPCXL_CAP_HDM1_REMOTE_MEMORY	(1ULL << 3)
#define CAPCXL_CAPS_REQUIRED	(CAPCXL_CAP_PF0_TYPE2 | \
				 CAPCXL_CAP_PF1_TYPE3 | \
				 CAPCXL_CAP_SHARED_COMPONENT_REGS | \
				 CAPCXL_CAP_HDM1_REMOTE_MEMORY)

#define CAPCXL_VENDOR_ID	0x8086
#define CAPCXL_DEVICE_ID	0x0ddb
#define CAPCXL_PF0_CLASS	0x120000
#define CAPCXL_PF1_CLASS	0x120000
#define CAPCXL_PF0_REVISION	0x02
#define CAPCXL_PF1_REVISION	0x02

enum capcxl_role {
	CAPCXL_ROLE_NONE,
	CAPCXL_ROLE_TYPE2,
	CAPCXL_ROLE_TYPE3,
};

struct capcxl_pci_identity {
	unsigned short vendor;
	unsigned short device;
	unsigned char function;
	unsigned int class_code;
	unsigned char revision;
};

static inline int capcxl_identity_matches(unsigned long long magic,
					  unsigned long long caps)
{
	return magic == CAPCXL_ID_VALUE &&
	       (caps & CAPCXL_CAPS_REQUIRED) == CAPCXL_CAPS_REQUIRED;
}

static inline int capcxl_pf_matches(const struct capcxl_pci_identity *id,
				    unsigned char function,
				    unsigned int class_code,
				    unsigned char revision)
{
	return id && id->vendor == CAPCXL_VENDOR_ID &&
	       id->device == CAPCXL_DEVICE_ID && id->function == function &&
	       id->class_code == class_code && id->revision == revision;
}

static inline enum capcxl_role capcxl_role_for_pair(
	unsigned long long magic, unsigned long long caps,
	unsigned int current_function,
	const struct capcxl_pci_identity *pf0,
	const struct capcxl_pci_identity *pf1)
{
	if (!capcxl_identity_matches(magic, caps) ||
	    !capcxl_pf_matches(pf0, 0, CAPCXL_PF0_CLASS,
			       CAPCXL_PF0_REVISION) ||
	    !capcxl_pf_matches(pf1, 1, CAPCXL_PF1_CLASS,
			       CAPCXL_PF1_REVISION))
		return CAPCXL_ROLE_NONE;

	if (current_function == 0)
		return CAPCXL_ROLE_TYPE2;
	if (current_function == 1)
		return CAPCXL_ROLE_TYPE3;

	return CAPCXL_ROLE_NONE;
}

#endif /* __CAPCXL_IDENTITY_H__ */
