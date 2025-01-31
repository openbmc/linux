// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * sbrmi-common.c - file defining SB-RMI protocols
 *		    compliant AMD SoC device.
 *
 * Copyright (C) 2021-2022 Advanced Micro Devices, Inc.
 */
#include <linux/err.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>

#include "sbrmi-common.h"

/* Mask for Status Register bit[1] */
#define SW_ALERT_MASK	0x2
/* Mask to check H/W Alert status bit */
#define HW_ALERT_MASK	0x80

/* Software Interrupt for triggering */
#define START_CMD	0x80
#define TRIGGER_MAILBOX	0x01

/* Mailbox error with additional data */
#define ERR_WITH_DATA	0x5

/* Default message lengths as per APML command protocol */
/* MSR */
#define MSR_RD_REG_LEN		0xa
#define MSR_WR_REG_LEN		0x8
#define MSR_WR_REG_LEN_v21	0x9
#define MSR_RD_DATA_LEN		0x8
#define MSR_WR_DATA_LEN		0x7
#define MSR_WR_DATA_LEN_v21	0x8
/* CPUID */
#define CPUID_RD_DATA_LEN	0x8
#define CPUID_WR_DATA_LEN	0x8
#define CPUID_WR_DATA_LEN_v21	0x9
#define CPUID_RD_REG_LEN	0xa
#define CPUID_WR_REG_LEN	0x9
#define CPUID_WR_REG_LEN_v21	0xa

/* CPUID MSR Command Ids */
#define CPUID_MCA_CMD	0x73
#define RD_CPUID_CMD	0x91
#define RD_MCA_CMD	0x86

/* SB-RMI registers */
enum sbrmi_reg {
	SBRMI_REV		= 0x0,
	SBRMI_CTRL		= 0x01,
	SBRMI_STATUS,
	SBRMI_OUTBNDMSG0	= 0x30,
	SBRMI_OUTBNDMSG1,
	SBRMI_OUTBNDMSG2,
	SBRMI_OUTBNDMSG3,
	SBRMI_OUTBNDMSG4,
	SBRMI_OUTBNDMSG5,
	SBRMI_OUTBNDMSG6,
	SBRMI_OUTBNDMSG7,
	SBRMI_INBNDMSG0,
	SBRMI_INBNDMSG1,
	SBRMI_INBNDMSG2,
	SBRMI_INBNDMSG3,
	SBRMI_INBNDMSG4,
	SBRMI_INBNDMSG5,
	SBRMI_INBNDMSG6,
	SBRMI_INBNDMSG7,
	SBRMI_SW_INTERRUPT,
	SBRMI_THREAD128CS	= 0x4b,
};

/* input for bulk write to v21 of CPUID and MSR protocol */
struct cpu_msr_indata_v21 {
	u8 wr_len;	/* const value */
	u8 rd_len;	/* const value */
	u8 proto_cmd;	/* const value */
	u16 thread;	/* thread number */
	union {
		u8 reg_offset[4];	/* input value */
		u32 value;
	};
	u8 ext; /* extended function */
} __packed;

/* input for bulk write to v20 of CPUID and MSR protocol */
struct cpu_msr_indata {
	u8 wr_len;	/* const value */
	u8 rd_len;	/* const value */
	u8 proto_cmd;	/* const value */
	u8 thread;	/* thread number */
	union {
		u8 reg_offset[4];	/* input value */
		u32 value;
	};
	u8 ext; /* extended function */
} __packed;

/* output for bulk read from CPUID and MSR protocol */
struct cpu_msr_outdata {
	u8 num_bytes;	/* number of bytes return */
	u8 status;	/* Protocol status code */
	union {
		u64 value;
		u8 reg_data[8];
	};
} __packed;

#define prepare_mca_msr_input_message(input, thread_id, data_in, wr_data_len)	\
	input.rd_len = MSR_RD_DATA_LEN,					\
	input.wr_len = wr_data_len,					\
	input.proto_cmd = RD_MCA_CMD,					\
	input.thread = thread_id << 1,					\
	input.value =  data_in

#define prepare_cpuid_input_message(input, thread_id, func, ext_func, wr_data_len)	\
	input.rd_len = CPUID_RD_DATA_LEN,				\
	input.wr_len = wr_data_len,				\
	input.proto_cmd = RD_CPUID_CMD,					\
	input.thread = thread_id << 1,					\
	input.value =  func,						\
	input.ext =  ext_func

static int sbrmi_get_rev(struct apml_sbrmi_device *rmi_dev)
{
	struct apml_message msg = { 0 };
	int ret;

	msg.data_in.reg_in[REG_OFF_INDEX] = SBRMI_REV;
	msg.data_in.reg_in[RD_FLAG_INDEX] = 1;
	ret = regmap_read(rmi_dev->regmap,
			  msg.data_in.reg_in[REG_OFF_INDEX],
			  &msg.data_out.mb_out[RD_WR_DATA_INDEX]);
	if (ret < 0)
		return ret;

	rmi_dev->rev = msg.data_out.reg_out[RD_WR_DATA_INDEX];
	return 0;
}

/*
 * For Mailbox command software alert status bit is set by firmware
 * to indicate command completion
 * For RMI Rev 0x20, new h/w status bit is introduced. which is used
 * by firmware to indicate completion of commands (0x71, 0x72, 0x73).
 * wait for the status bit to be set by the firmware before
 * reading the data out.
 */
static int sbrmi_wait_status(struct apml_sbrmi_device *rmi_dev,
			     int *status, int mask)
{
	int ret, retry = 100;

	do {
		ret = regmap_read(rmi_dev->regmap, SBRMI_STATUS, status);
		if (ret < 0)
			return ret;

		if (*status & mask)
			break;

		/* Wait 1~2 second for firmware to return data out */
		if (retry > 95)
			usleep_range(50, 100);
		else
			usleep_range(10000, 20000);
	} while (retry--);

	if (retry < 0)
		ret = -ETIMEDOUT;
	return ret;
}

static int msr_datain_v20(struct apml_sbrmi_device *rmi_dev,
			  struct apml_message *msg)
{
	struct cpu_msr_indata input = {0};
	int ret, val = 0;
	u16 thread;

	thread = msg->data_in.reg_in[THREAD_LOW_INDEX] |
		 msg->data_in.reg_in[THREAD_HI_INDEX] << 8;

	/* Thread > 127, Thread128 CS register, 1'b1 needs to be set to 1 */
	if (thread > 127) {
		thread -= 128;
		val = 1;
	}
	ret = regmap_write(rmi_dev->regmap, SBRMI_THREAD128CS, val);
	if (ret < 0)
		return ret;

	prepare_mca_msr_input_message(input, thread,
				      msg->data_in.mb_in[RD_WR_DATA_INDEX],
				      MSR_WR_DATA_LEN);

	ret = regmap_bulk_write(rmi_dev->regmap, CPUID_MCA_CMD,
				&input, MSR_WR_REG_LEN);
	return ret;
}

static int msr_datain_v21(struct apml_sbrmi_device *rmi_dev,
				struct apml_message *msg)
{
	struct cpu_msr_indata_v21 input = {0};
	int ret;
	u16 thread;

	thread = msg->data_in.reg_in[THREAD_LOW_INDEX] |
		 msg->data_in.reg_in[THREAD_HI_INDEX] << 8;

	prepare_mca_msr_input_message(input, thread,
				      msg->data_in.mb_in[RD_WR_DATA_INDEX],
				      MSR_WR_DATA_LEN_v21);

	ret = regmap_bulk_write(rmi_dev->regmap, CPUID_MCA_CMD,
				&input, MSR_WR_REG_LEN_v21);
	return ret;
}

/* MCA MSR protocol */
int rmi_mca_msr_read(struct apml_sbrmi_device *rmi_dev,
		     struct apml_message *msg)
{
	struct cpu_msr_outdata output = {0};
	int ret;
	int hw_status;

	if (!rmi_dev->regmap)
		return ENODEV;

	/* cache the rev value to identify if protocol is supported or not */
	if (!rmi_dev->rev) {
		ret = sbrmi_get_rev(rmi_dev);
		if (ret < 0)
			return ret;
	}

	switch(rmi_dev->rev) {
	/* MCA MSR protocol for REV 0x10 is not supported*/
	case 0x10:
		return -EOPNOTSUPP;
	case 0x20:
		ret = msr_datain_v20(rmi_dev, msg);
		if (ret < 0)
			goto exit_unlock;

		break;
	case 0x21:
		ret = msr_datain_v21(rmi_dev, msg);
		if (ret < 0)
			goto exit_unlock;
		break;
	default:
		return -EOPNOTSUPP;
	}

	ret = sbrmi_wait_status(rmi_dev, &hw_status, HW_ALERT_MASK);
	if (ret < 0)
		goto exit_unlock;

	ret = regmap_bulk_read(rmi_dev->regmap, CPUID_MCA_CMD,
			       &output, MSR_RD_REG_LEN);
	if (ret < 0)
		goto exit_unlock;

	ret = regmap_write(rmi_dev->regmap, SBRMI_STATUS,
			   HW_ALERT_MASK);
	if (ret < 0)
		goto exit_unlock;

	if (output.num_bytes != MSR_RD_REG_LEN - 1) {
		ret = -EMSGSIZE;
		goto exit_unlock;
	}
	if (output.status) {
		ret = -EPROTOTYPE;
		msg->fw_ret_code = output.status;
		goto exit_unlock;
	}
	msg->data_out.cpu_msr_out = output.value;

exit_unlock:
	return ret;
}

static int cpuid_datain_v20(struct apml_sbrmi_device *rmi_dev,
			    struct apml_message *msg)
{
	struct cpu_msr_indata input = {0};
	int ret, val = 0;
	u16 thread;

	thread = msg->data_in.reg_in[THREAD_LOW_INDEX] |
		 msg->data_in.reg_in[THREAD_HI_INDEX] << 8;

	/* Thread > 127, Thread128 CS register, 1'b1 needs to be set to 1 */
	if (thread > 127) {
		thread -= 128;
		val = 1;
	}
	ret = regmap_write(rmi_dev->regmap, SBRMI_THREAD128CS, val);
	if (ret < 0)
		return ret;
	prepare_cpuid_input_message(input, thread,
				    msg->data_in.mb_in[RD_WR_DATA_INDEX],
				    msg->data_in.reg_in[EXT_FUNC_INDEX],
				    CPUID_WR_DATA_LEN);

	ret = regmap_bulk_write(rmi_dev->regmap, CPUID_MCA_CMD,
				&input, CPUID_WR_REG_LEN);
	return ret;
}

static int cpuid_datain_v21(struct apml_sbrmi_device *rmi_dev,
			    struct apml_message *msg)
{
	struct cpu_msr_indata_v21 input = {0};
	int ret;
	u16 thread;

	thread = msg->data_in.reg_in[THREAD_LOW_INDEX] |
		 msg->data_in.reg_in[THREAD_HI_INDEX] << 8;

	prepare_cpuid_input_message(input, thread,
				    msg->data_in.mb_in[RD_WR_DATA_INDEX],
				    msg->data_in.reg_in[EXT_FUNC_INDEX],
				    CPUID_WR_DATA_LEN_v21);

	ret = regmap_bulk_write(rmi_dev->regmap, CPUID_MCA_CMD,
				&input, CPUID_WR_REG_LEN_v21);
	return ret;
}

/* CPUID protocol */
int rmi_cpuid_read(struct apml_sbrmi_device *rmi_dev,
		   struct apml_message *msg)
{
	struct cpu_msr_outdata output = {0};
	int ret, hw_status;

	if (!rmi_dev->regmap)
		return ENODEV;

	/* cache the rev value to identify if protocol is supported or not */
	if (!rmi_dev->rev) {
		ret = sbrmi_get_rev(rmi_dev);
		if (ret < 0)
			return ret;
	}

	switch(rmi_dev->rev) {
	/* CPUID protocol for REV 0x10 is not supported*/
	case 0x10:
		return -EOPNOTSUPP;
	case 0x20:
		ret = cpuid_datain_v20(rmi_dev, msg);
		if (ret < 0)
			goto exit_unlock;
		break;
	case 0x21:
		ret = cpuid_datain_v21(rmi_dev, msg);
		if (ret < 0)
			goto exit_unlock;
		break;
	default:
		return -EOPNOTSUPP;
	}

	ret = sbrmi_wait_status(rmi_dev, &hw_status, HW_ALERT_MASK);
	if (ret < 0)
		goto exit_unlock;

	ret = regmap_bulk_read(rmi_dev->regmap, CPUID_MCA_CMD,
			       &output, CPUID_RD_REG_LEN);
	if (ret < 0)
		goto exit_unlock;

	ret = regmap_write(rmi_dev->regmap, SBRMI_STATUS,
			   HW_ALERT_MASK);
	if (ret < 0)
		goto exit_unlock;

	if (output.num_bytes != CPUID_RD_REG_LEN - 1) {
		ret = -EMSGSIZE;
		goto exit_unlock;
	}
	if (output.status) {
		ret = -EPROTOTYPE;
		msg->fw_ret_code = output.status;
		goto exit_unlock;
	}
	msg->data_out.cpu_msr_out = output.value;
exit_unlock:
	return ret;
}

static int esmi_oob_clear_status_alert(struct apml_sbrmi_device *rmi_dev)
{
	int sw_status, ret;

	ret = regmap_read(rmi_dev->regmap, SBRMI_STATUS,
			  &sw_status);
	if (ret < 0)
		return ret;

	if (!(sw_status & SW_ALERT_MASK))
		return 0;

	return regmap_write(rmi_dev->regmap, SBRMI_STATUS,
			    SW_ALERT_MASK);
}

int rmi_mailbox_xfer(struct apml_sbrmi_device *rmi_dev,
		     struct apml_message *msg)
{
	unsigned int bytes = 0, ec = 0;
	int i, ret;
	int sw_status;
	u8 byte = 0;

	if (!rmi_dev->regmap)
		return ENODEV;

	msg->fw_ret_code = 0;

	ret = esmi_oob_clear_status_alert(rmi_dev);
	if (ret < 0)
		goto exit_unlock;

	/* Indicate firmware a command is to be serviced */
	ret = regmap_write(rmi_dev->regmap, SBRMI_INBNDMSG7, START_CMD);
	if (ret < 0)
		goto exit_unlock;

	/* Write the command to SBRMI::InBndMsg_inst0 */
	ret = regmap_write(rmi_dev->regmap, SBRMI_INBNDMSG0, msg->cmd);
	if (ret < 0)
		goto exit_unlock;

	/*
	 * For both read and write the initiator (BMC) writes
	 * Command Data In[31:0] to SBRMI::InBndMsg_inst[4:1]
	 * SBRMI_x3C(MSB):SBRMI_x39(LSB)
	 */
	for (i = 0; i < MB_DATA_SIZE; i++) {
		byte = msg->data_in.reg_in[i];
		ret = regmap_write(rmi_dev->regmap, SBRMI_INBNDMSG1 + i, byte);
		if (ret < 0)
			goto exit_unlock;
	}

	/*
	 * Write 0x01 to SBRMI::SoftwareInterrupt to notify firmware to
	 * perform the requested read or write command
	 */
	ret = regmap_write(rmi_dev->regmap, SBRMI_SW_INTERRUPT, TRIGGER_MAILBOX);
	if (ret)
		goto exit_unlock;

	/*
	 * Firmware will write SBRMI::Status[SwAlertSts]=1 to generate
	 * an ALERT (if enabled) to initiator (BMC) to indicate completion
	 * of the requested command
	 */
	ret = sbrmi_wait_status(rmi_dev, &sw_status, SW_ALERT_MASK);
	if (ret)
		goto exit_unlock;

	ret = regmap_read(rmi_dev->regmap, SBRMI_OUTBNDMSG7, &ec);
	if (ret || (ec && ec != ERR_WITH_DATA))
		goto exit_clear_alert;

	/*
	 * For a read operation, the initiator (BMC) reads the firmware
	 * response Command Data Out[31:0] from SBRMI::OutBndMsg_inst[4:1]
	 * {SBRMI_x34(MSB):SBRMI_x31(LSB)}.
	 */
	if (msg->data_in.reg_in[RD_FLAG_INDEX]) {
		for (i = 0; i < MB_DATA_SIZE; i++) {
			ret = regmap_read(rmi_dev->regmap,
					  SBRMI_OUTBNDMSG1 + i, &bytes);
			if (ret < 0)
				break;
			msg->data_out.reg_out[i] = bytes;
		}
	}
exit_clear_alert:
	/*
	 * BMC must write 1'b1 to SBRMI::Status[SwAlertSts] to clear the
	 * ALERT to initiator
	 */
	ret = regmap_write(rmi_dev->regmap, SBRMI_STATUS,
			   SW_ALERT_MASK);
	if (ec) {
		ret = -EPROTOTYPE;
		msg->fw_ret_code = ec;
	}
exit_unlock:
	return ret;
}
