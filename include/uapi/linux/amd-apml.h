/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Copyright (C) 2021-2022 Advanced Micro Devices, Inc.
 */
#ifndef _AMD_APML_H_
#define _AMD_APML_H_

#include <linux/types.h>

/*
 * Currently signal 33 to 64 are unused,
 * using user signal number from that range
 */
#define USR_SIGNAL	44

enum apml_protocol {
	APML_CPUID	= 0x1000,
	APML_MCA_MSR,
	APML_REG,
};

/* These are byte indexes into data_in and data_out arrays */
#define RD_WR_DATA_INDEX	0
#define REG_OFF_INDEX		0
#define REG_VAL_INDEX		4
#define THREAD_LOW_INDEX	4
#define THREAD_HI_INDEX		5
#define EXT_FUNC_INDEX		6
#define RD_FLAG_INDEX		7

#define MB_DATA_SIZE		4

#define MAX_TBAI_DWORDS		32
#define MAX_TBAI_BYTES		128
#define TBAI_INPUT_BYTES	5
#define TBAI_CMD_INDEX		0
#define TBAI_LUT_INDEX		1
#define TBAI_OFFSET_LO		2
#define TBAI_OFFSET_HI		3
#define TBAI_DWORD_RD_INDEX	4

struct apml_message {
	/* message ids:
	 * Mailbox Messages:	0x0 ... 0x999
	 * APML_CPUID:		0x1000
	 * APML_MCA_MSR:	0x1001
	 * APML_REG:		0x1002 (RMI & TSI reg access)
	 */
	__u32 cmd;

	/*
	 * 8 bit data for reg read,
	 * 32 bit data in case of mailbox,
	 * upto 64 bit in case of cpuid and mca msr
	 */
	union {
		__u64 cpu_msr_out;
		__u32 mb_out[2];
		__u8 reg_out[8];
	} data_out;

	/*
	 * [0]...[3] mailbox 32bit input
	 *	     cpuid & mca msr,
	 *	     rmi rd/wr: reg_offset
	 * [4][5] cpuid & mca msr: thread
	 * [4] rmi reg wr: value
	 * [6] cpuid: ext function & read eax/ebx or ecx/edx
	 *	[7:0] -> bits [7:4] -> ext function &
	 *	bit [0] read eax/ebx or ecx/edx
	 * [7] read/write functionality
	 */
	union {
		__u64 cpu_msr_in;
		__u32 mb_in[2];
		__u8 reg_in[8];
	} data_in;
	/*
	 * Status code is returned in case of CPUID/MCA access
	 * Error code is returned in case of soft mailbox
	 */
	__u32 fw_ret_code;
} __attribute__((packed));

/*
 * 5 bytes input
 * TODO: res param may not be required as IOCTL only supporting TBAI
 * TODO: Two different IOCTL for TBAI Acquire and TBAI Flush
 */
struct apml_tbai_msg {
	/*
	 * Reserved, for Logging enable/disable
	 * Remove if not required
	 */
	__u32 rsvd;
	/*
	 * Input register data to write
	 * [0]: TBAI command ID
	 *	0x31: TBAI acquire
	 *	0x32: Clear TBAI buffer
	 * [1]: LUT index
	 * [2]: Entry offset low byte
	 * [3]: Entry offset high byte
	 * [4]: Dwords to read
	 *	Supported values:
	 *	1DW to 32DW
	 */
	__u8 reg_in[TBAI_INPUT_BYTES];
	/*
	 * Output register data to read
	 * maximum 32 Dwords can be read
	 */
	  union {
		__u32 dwords_out[MAX_TBAI_DWORDS];
		__u8 bytes_out[MAX_TBAI_BYTES];
	  } data_out;
	__u32 fw_ret_code;
} __attribute__((packed));

/* ioctl command for mailbox msgs using generic _IOWR */
#define SBRMI_BASE_IOCTL_NR      0xF9
#define SBRMI_IOCTL_CMD          _IOWR(SBRMI_BASE_IOCTL_NR, 0, struct apml_message)

/*
 * Single ioctl commands for acquire & flush
 * TODO: Different IOCTL for TBAI acquire and TBAI flush
 */
#define SBTBAI_IOCTL_CMD         _IOWR(SBRMI_BASE_IOCTL_NR, 1, struct apml_tbai_msg)

#endif /*_AMD_APML_H_*/
