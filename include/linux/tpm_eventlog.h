/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __LINUX_TPM_EVENTLOG_H__
#define __LINUX_TPM_EVENTLOG_H__

#include <crypto/hash_info.h>

#define TCG_EVENT_NAME_LEN_MAX	255
#define MAX_TEXT_EVENT		1000	/* Max event string length */
#define ACPI_TCPA_SIG		"TCPA"	/* 0x41504354 /'TCPA' */
#define TPM2_ACTIVE_PCR_BANKS	3

#define EFI_TCG2_EVENT_LOG_FORMAT_TCG_1_2 0x1
#define EFI_TCG2_EVENT_LOG_FORMAT_TCG_2   0x2

#ifdef CONFIG_PPC64
#define do_endian_conversion(x) be32_to_cpu(x)
#else
#define do_endian_conversion(x) x
#endif

enum bios_platform_class {
	BIOS_CLIENT = 0x00,
	BIOS_SERVER = 0x01,
};

struct tcpa_event {
	u32 pcr_index;
	u32 event_type;
	u8 pcr_value[20];	/* SHA1 */
	u32 event_size;
	u8 event_data[0];
};

enum tcpa_event_types {
	PREBOOT = 0,
	POST_CODE,
	UNUSED,
	NO_ACTION,
	SEPARATOR,
	ACTION,
	EVENT_TAG,
	SCRTM_CONTENTS,
	SCRTM_VERSION,
	CPU_MICROCODE,
	PLATFORM_CONFIG_FLAGS,
	TABLE_OF_DEVICES,
	COMPACT_HASH,
	IPL,
	IPL_PARTITION_DATA,
	NONHOST_CODE,
	NONHOST_CONFIG,
	NONHOST_INFO,
};

struct tcpa_pc_event {
	u32 event_id;
	u32 event_size;
	u8 event_data[0];
};

enum tcpa_pc_event_ids {
	SMBIOS = 1,
	BIS_CERT,
	POST_BIOS_ROM,
	ESCD,
	CMOS,
	NVRAM,
	OPTION_ROM_EXEC,
	OPTION_ROM_CONFIG,
	OPTION_ROM_MICROCODE = 10,
	S_CRTM_VERSION,
	S_CRTM_CONTENTS,
	POST_CONTENTS,
	HOST_TABLE_OF_DEVICES,
};

/* http://www.trustedcomputinggroup.org/tcg-efi-protocol-specification/ */

struct tcg_efi_specid_event_algs {
	u16 alg_id;
	u16 digest_size;
} __packed;

struct tcg_efi_specid_event {
	u8 signature[16];
	u32 platform_class;
	u8 spec_version_minor;
	u8 spec_version_major;
	u8 spec_errata;
	u8 uintnsize;
	u32 num_algs;
	struct tcg_efi_specid_event_algs digest_sizes[TPM2_ACTIVE_PCR_BANKS];
	u8 vendor_info_size;
	u8 vendor_info[0];
} __packed;

struct tcg_pcr_event {
	u32 pcr_idx;
	u32 event_type;
	u8 digest[20];
	u32 event_size;
	u8 event[0];
} __packed;

struct tcg_event_field {
	u32 event_size;
	u8 event[0];
} __packed;

struct tpm2_digest {
	u16 alg_id;
	u8 digest[SHA512_DIGEST_SIZE];
} __packed;

struct tcg_pcr_event2 {
	u32 pcr_idx;
	u32 event_type;
	u32 count;
	struct tpm2_digest digests[TPM2_ACTIVE_PCR_BANKS];
	struct tcg_event_field event;
} __packed;

#endif
