/*
 *  I2C adapter for the ASPEED I2C bus access.
 *
 *  Copyright (C) 2012-2020  ASPEED Technology Inc.
 *  Copyright 2015 IBM Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  History:
 *    2012.07.26: Initial version [Ryan Chen]
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/slab.h>
#include <linux/of_address.h>

#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <linux/dma-mapping.h>

#include <asm/irq.h>
#include <asm/io.h>

#if defined(CONFIG_COLDFIRE)
#include <asm/arch/regs-iic.h>
#include <asm/arch/ast_i2c.h>
#else
//#include <plat/regs-iic.h>
//#include <plat/ast_i2c.h>
#endif

#define BYTE_MODE	0
#define BUFF_MODE	1
#define DEC_DMA_MODE	2
#define INC_DMA_MODE	3

/* I2C Register */
#define  I2C_FUN_CTRL_REG    				0x00
#define  I2C_AC_TIMING_REG1         		0x04
#define  I2C_AC_TIMING_REG2         		0x08
#define  I2C_INTR_CTRL_REG					0x0c
#define  I2C_INTR_STS_REG					0x10
#define  I2C_CMD_REG						0x14
#define  I2C_DEV_ADDR_REG					0x18
#define  I2C_BUF_CTRL_REG					0x1c
#define  I2C_BYTE_BUF_REG					0x20
#define  I2C_DMA_BASE_REG					0x24
#define  I2C_DMA_LEN_REG					0x28

#define AST_I2C_DMA_SIZE				0
#define AST_I2C_PAGE_SIZE 				256
#define MASTER_XFER_MODE				BUFF_MODE
#define SLAVE_XFER_MODE				BYTE_MODE
#define NUM_BUS 14

/*AST I2C Register Definition */
// if defined(AST_SOC_G4)
#define AST_I2C_POOL_BUFF_2048	
#define AST_I2C_GLOBAL_REG		0x00
#define AST_I2C_DEVICE1			0x40
#define AST_I2C_DEVICE2			0x80
#define AST_I2C_DEVICE3			0xc0
#define AST_I2C_DEVICE4			0x100
#define AST_I2C_DEVICE5			0x140
#define AST_I2C_DEVICE6			0x180
#define AST_I2C_DEVICE7			0x1c0
#define AST_I2C_BUFFER_POOL2	0x200
#define AST_I2C_DEVICE8			0x300
#define AST_I2C_DEVICE9			0x340
#define AST_I2C_DEVICE10		0x380
#define AST_I2C_DEVICE11		0x3c0
#define AST_I2C_DEVICE12		0x400
#define AST_I2C_DEVICE13		0x440
#define AST_I2C_DEVICE14		0x480
#define AST_I2C_BUFFER_POOL1	0x800

/* Gloable Register Definition */
/* 0x00 : I2C Interrupt Status Register  */
/* 0x08 : I2C Interrupt Target Assignment  */

/* Device Register Definition */
/* 0x00 : I2CD Function Control Register  */
#define AST_I2CD_BUFF_SEL_MASK				(0x7 << 20)
#define AST_I2CD_BUFF_SEL(x) 				(x << 20)		// page 0 ~ 7
#define AST_I2CD_M_SDA_LOCK_EN			(0x1 << 16)
#define AST_I2CD_MULTI_MASTER_DIS			(0x1 << 15)
#define AST_I2CD_M_SCL_DRIVE_EN		(0x1 << 14)
#define AST_I2CD_MSB_STS					(0x1 << 9)
#define AST_I2CD_SDA_DRIVE_1T_EN			(0x1 << 8)
#define AST_I2CD_M_SDA_DRIVE_1T_EN		(0x1 << 7)
#define AST_I2CD_M_HIGH_SPEED_EN		(0x1 << 6)
#define AST_I2CD_DEF_ADDR_EN				(0x1 << 5)
#define AST_I2CD_DEF_ALERT_EN				(0x1 << 4)
#define AST_I2CD_DEF_ARP_EN					(0x1 << 3)
#define AST_I2CD_DEF_GCALL_EN				(0x1 << 2)
#define AST_I2CD_SLAVE_EN					(0x1 << 1)
#define AST_I2CD_MASTER_EN					(0x1 )

/* 0x04 : I2CD Clock and AC Timing Control Register #1 */
#define AST_I2CD_tBUF						(0x1 << 28) 	// 0~7 
#define AST_I2CD_tHDSTA						(0x1 << 24)		// 0~7 
#define AST_I2CD_tACST						(0x1 << 20)		// 0~7 
#define AST_I2CD_tCKHIGH					(0x1 << 16)		// 0~7 
#define AST_I2CD_tCKLOW						(0x1 << 12)		// 0~7 
#define AST_I2CD_tHDDAT						(0x1 << 10)		// 0~7 
#define AST_I2CD_CLK_TO_BASE_DIV			(0x1 << 8)		// 0~3
#define AST_I2CD_CLK_BASE_DIV				(0x1 )			// 0~0xf

/* 0x08 : I2CD Clock and AC Timing Control Register #2 */
#define AST_I2CD_tTIMEOUT					(0x1 )			// 0~7
#define AST_NO_TIMEOUT_CTRL					0x0


/* 0x0c : I2CD Interrupt Control Register &
 * 0x10 : I2CD Interrupt Status Register
 *
 * These share bit definitions, so use the same values for the enable &
 * status bits.
 */
#define AST_I2CD_INTR_SDA_DL_TIMEOUT			(0x1 << 14)
#define AST_I2CD_INTR_BUS_RECOVER_DONE			(0x1 << 13)
#define AST_I2CD_INTR_SMBUS_ALERT			(0x1 << 12)
#define AST_I2CD_INTR_SMBUS_ARP_ADDR			(0x1 << 11)
#define AST_I2CD_INTR_SMBUS_DEV_ALERT_ADDR		(0x1 << 10)
#define AST_I2CD_INTR_SMBUS_DEF_ADDR			(0x1 << 9)
#define AST_I2CD_INTR_GCALL_ADDR			(0x1 << 8)
#define AST_I2CD_INTR_SLAVE_MATCH			(0x1 << 7)
#define AST_I2CD_INTR_SCL_TIMEOUT			(0x1 << 6)
#define AST_I2CD_INTR_ABNORMAL				(0x1 << 5)
#define AST_I2CD_INTR_NORMAL_STOP			(0x1 << 4)
#define AST_I2CD_INTR_ARBIT_LOSS			(0x1 << 3)
#define AST_I2CD_INTR_RX_DONE				(0x1 << 2)
#define AST_I2CD_INTR_TX_NAK				(0x1 << 1)
#define AST_I2CD_INTR_TX_ACK				(0x1 << 0)

/* 0x14 : I2CD Command/Status Register   */
#define AST_I2CD_SDA_OE					(0x1 << 28)
#define AST_I2CD_SDA_O					(0x1 << 27)		
#define AST_I2CD_SCL_OE					(0x1 << 26)		
#define AST_I2CD_SCL_O					(0x1 << 25)		
#define AST_I2CD_TX_TIMING				(0x1 << 24)		// 0 ~3
#define AST_I2CD_TX_STATUS				(0x1 << 23)		
// Tx State Machine 
#define AST_I2CD_IDLE	 				0x0
#define AST_I2CD_MACTIVE				0x8
#define AST_I2CD_MSTART					0x9
#define AST_I2CD_MSTARTR				0xa
#define AST_I2CD_MSTOP					0xb
#define AST_I2CD_MTXD					0xc
#define AST_I2CD_MRXACK					0xd
#define AST_I2CD_MRXD 					0xe
#define AST_I2CD_MTXACK 				0xf
#define AST_I2CD_SWAIT					0x1
#define AST_I2CD_SRXD 					0x4
#define AST_I2CD_STXACK 				0x5
#define AST_I2CD_STXD					0x6
#define AST_I2CD_SRXACK 				0x7
#define AST_I2CD_RECOVER 				0x3

#define AST_I2CD_SCL_LINE_STS				(0x1 << 18)		
#define AST_I2CD_SDA_LINE_STS				(0x1 << 17)		
#define AST_I2CD_BUS_BUSY_STS				(0x1 << 16)		
#define AST_I2CD_SDA_OE_OUT_DIR				(0x1 << 15)		
#define AST_I2CD_SDA_O_OUT_DIR				(0x1 << 14)		
#define AST_I2CD_SCL_OE_OUT_DIR				(0x1 << 13)		
#define AST_I2CD_SCL_O_OUT_DIR				(0x1 << 12)		
#define AST_I2CD_BUS_RECOVER_CMD_EN			(0x1 << 11)		
#define AST_I2CD_S_ALT_EN				(0x1 << 10)		
// 0 : DMA Buffer, 1: Pool Buffer
//AST1070 DMA register 
#define AST_I2CD_RX_DMA_ENABLE				(0x1 << 9)		
#define AST_I2CD_TX_DMA_ENABLE				(0x1 << 8)		

/* Command Bit */
#define AST_I2CD_RX_BUFF_ENABLE				(0x1 << 7)		
#define AST_I2CD_TX_BUFF_ENABLE				(0x1 << 6)		
#define AST_I2CD_M_STOP_CMD					(0x1 << 5)		
#define AST_I2CD_M_S_RX_CMD_LAST			(0x1 << 4)		
#define AST_I2CD_M_RX_CMD					(0x1 << 3)		
#define AST_I2CD_S_TX_CMD					(0x1 << 2)		
#define AST_I2CD_M_TX_CMD					(0x1 << 1)		
#define AST_I2CD_M_START_CMD				(0x1 )		

/* 0x18 : I2CD Slave Device Address Register   */

/* 0x1C : I2CD Pool Buffer Control Register   */
#define AST_I2CD_RX_BUF_ADDR_GET(x)				((x>> 24)& 0xff)
#define AST_I2CD_RX_BUF_END_ADDR_SET(x)			(x << 16)		
#define AST_I2CD_TX_DATA_BUF_END_SET(x)			((x&0xff) << 8)		
#define AST_I2CD_TX_DATA_BUF_GET(x)			((x >>8) & 0xff)		
#define AST_I2CD_BUF_BASE_ADDR_SET(x)			(x & 0x3f)		

/* 0x20 : I2CD Transmit/Receive Byte Buffer Register   */
#define AST_I2CD_GET_MODE(x)					((x >> 8) & 0x1)		

#define AST_I2CD_RX_BYTE_BUFFER					(0xff << 8)		
#define AST_I2CD_TX_BYTE_BUFFER					(0xff )		

//1. usage flag , 2 size,	3. request address
/* Use platform_data instead of module parameters */
/* Fast Mode = 400 kHz, Standard = 100 kHz */
//static int clock = 100; /* Default: 100 kHz */

#define AST_I2CD_CMDS	(AST_I2CD_BUS_RECOVER_CMD_EN | \
			 AST_I2CD_M_STOP_CMD | \
			 AST_I2CD_M_S_RX_CMD_LAST | \
			 AST_I2CD_M_RX_CMD | \
			 AST_I2CD_M_TX_CMD | \
			 AST_I2CD_M_START_CMD)


struct aspeed_i2c_bus {
	/* TODO: find a better way to do this */
	struct ast_i2c_dev *i2c_dev;
	struct device	*dev;

	void __iomem	*base;			/* virtual */	
	u32 state;				//I2C xfer mode state matchine 
	struct i2c_adapter adap;
	u32		bus_clk;

	/* i2c transfer state. this is accessed from both process and IRQ
	 * context, so is protected by cmd_lock */
	spinlock_t	cmd_lock;
	bool		send_start;
	bool		send_stop;	/* last message of an xfer? */
	bool		query_len;
	struct i2c_msg	*msg;		/* current tx/rx message */
	int		msg_pos;	/* current byte position in message */

	struct completion cmd_complete;
	u32		cmd_sent;
	u32		cmd_pending;
	u32		cmd_err;
};

struct ast_i2c_dev {
	struct device		*dev;
	void __iomem		*reg_gr;
	struct clk *pclk;
	struct aspeed_i2c_bus buses[14];
	int irq;				//I2C IRQ number 
};

static inline void ast_i2c_write(struct aspeed_i2c_bus *bus, u32 val, u32 reg)
{
	writel(val, bus->base + reg);
}

static inline u32 ast_i2c_read(struct aspeed_i2c_bus *bus, u32 reg)
{
	return readl(bus->base + reg);
}

static u32 select_i2c_clock(struct aspeed_i2c_bus *bus)
{
	unsigned int inc = 0, div, divider_ratio;
	u32 SCL_Low, SCL_High, data;

	divider_ratio = clk_get_rate(bus->i2c_dev->pclk) / bus->bus_clk;
	for (div = 0; divider_ratio >= 16; div++) {
		inc |= (divider_ratio & 1);
		divider_ratio >>= 1;
	}
	divider_ratio += inc;
	SCL_Low = (divider_ratio >> 1) - 1;
	SCL_High = divider_ratio - SCL_Low - 2;
	data = 0x77700300 | (SCL_High << 16) | (SCL_Low << 12) | div;
	return data;
}

static void ast_i2c_dev_init(struct aspeed_i2c_bus *bus)
{
	/* reset device: disable master & slave functions */
	ast_i2c_write(bus, 0, I2C_FUN_CTRL_REG);

	dev_dbg(bus->dev, "bus_clk %u, pclk %lu\n",
			bus->bus_clk, clk_get_rate(bus->i2c_dev->pclk));

	/* Set AC Timing */
	if(bus->bus_clk / 1000 > 400) {
		ast_i2c_write(bus, ast_i2c_read(bus, I2C_FUN_CTRL_REG) |
				AST_I2CD_M_HIGH_SPEED_EN |
				AST_I2CD_M_SDA_DRIVE_1T_EN |
				AST_I2CD_SDA_DRIVE_1T_EN,
				I2C_FUN_CTRL_REG);

		ast_i2c_write(bus, 0x3, I2C_AC_TIMING_REG2);
		ast_i2c_write(bus, select_i2c_clock(bus), I2C_AC_TIMING_REG1);
	} else {
		ast_i2c_write(bus, select_i2c_clock(bus), I2C_AC_TIMING_REG1);
		ast_i2c_write(bus, AST_NO_TIMEOUT_CTRL, I2C_AC_TIMING_REG2);
	}

	dev_dbg(bus->dev, "reg1: %x, reg2: %x, fun_ctrl: %x\n",
			ast_i2c_read(bus, I2C_AC_TIMING_REG1),
			ast_i2c_read(bus, I2C_AC_TIMING_REG2),
			ast_i2c_read(bus, I2C_FUN_CTRL_REG));

	/* Enable Master Mode */
	ast_i2c_write(bus, ast_i2c_read(bus, I2C_FUN_CTRL_REG)
			| AST_I2CD_MASTER_EN, I2C_FUN_CTRL_REG);


	/* Set interrupt generation of I2C controller */
	ast_i2c_write(bus, AST_I2CD_INTR_SDA_DL_TIMEOUT |
			AST_I2CD_INTR_BUS_RECOVER_DONE |
			AST_I2CD_INTR_SMBUS_ALERT |
			AST_I2CD_INTR_SCL_TIMEOUT |
			AST_I2CD_INTR_ABNORMAL |
			AST_I2CD_INTR_NORMAL_STOP |
			AST_I2CD_INTR_ARBIT_LOSS |
			AST_I2CD_INTR_RX_DONE |
			AST_I2CD_INTR_TX_NAK |
			AST_I2CD_INTR_TX_ACK,
			I2C_INTR_CTRL_REG);

}

static void ast_i2c_issue_cmd(struct aspeed_i2c_bus *bus, u32 cmd)
{
	dev_dbg(bus->dev, "issuing cmd: %x\n", cmd);
	bus->cmd_err = 0;
	bus->cmd_sent = bus->cmd_pending = cmd & AST_I2CD_CMDS;
	ast_i2c_write(bus, cmd, I2C_CMD_REG);
}

static int ast_i2c_issue_oob_command(struct aspeed_i2c_bus *bus, u32 cmd)
{
	spin_lock_irq(&bus->cmd_lock);
	init_completion(&bus->cmd_complete);
	ast_i2c_issue_cmd(bus, cmd);
	spin_unlock_irq(&bus->cmd_lock);
	return wait_for_completion_interruptible_timeout(&bus->cmd_complete,
					      bus->adap.timeout*HZ);
}

static u8 ast_i2c_bus_error_recover(struct aspeed_i2c_bus *bus)
{
	unsigned long flags;
	u32 sts;
	int r;
	u32 i = 0;

	//Check 0x14's SDA and SCL status
	sts = ast_i2c_read(bus,I2C_CMD_REG);

	if ((sts & AST_I2CD_SDA_LINE_STS) && (sts & AST_I2CD_SCL_LINE_STS)) {
		//Means bus is idle.
		dev_dbg(bus->dev,
				"I2C bus is idle. I2C slave doesn't exist?!\n");
		return -1;
	}

	dev_dbg(bus->dev, "I2C bus hung (status %x), attempting recovery\n",
			sts);

	if ((sts & AST_I2CD_SDA_LINE_STS) && !(sts & AST_I2CD_SCL_LINE_STS)) {
		//if SDA == 1 and SCL == 0, it means the master is locking the bus.
		//Send a stop command to unlock the bus.
		dev_dbg(bus->dev, "I2C's master is locking the bus, try to stop it.\n");

		init_completion(&bus->cmd_complete);

		ast_i2c_write(bus, AST_I2CD_M_STOP_CMD, I2C_CMD_REG);

		r = wait_for_completion_interruptible_timeout(&bus->cmd_complete,
							      bus->adap.timeout*HZ);

		if (bus->cmd_err) {
			dev_dbg(bus->dev, "recovery error \n");
			return -1;
		}

		if (r == 0) {
			 dev_dbg(bus->dev, "recovery timed out\n");
			 return -1;
		} else {
			dev_dbg(bus->dev, "Recovery successfully\n");
			return 0;
		}

	} else if (!(sts & AST_I2CD_SDA_LINE_STS)) {
		//else if SDA == 0, the device is dead. We need to reset the bus
		//And do the recovery command.
		dev_dbg(bus->dev, "I2C's slave is dead, try to recover it\n");
		for (i = 0; i < 2; i++) {
			ast_i2c_dev_init(bus);
			ast_i2c_issue_oob_command(bus,
					AST_I2CD_BUS_RECOVER_CMD_EN);
			if (bus->cmd_err != 0) {
				dev_dbg(bus->dev, "ERROR!! Failed to do recovery command(0x%08x)\n", bus->cmd_err);
				return -1;
			}
			//Check 0x14's SDA and SCL status
			sts = ast_i2c_read(bus,I2C_CMD_REG);
			if (sts & AST_I2CD_SDA_LINE_STS) //Recover OK
				break;
		}
		if (i == 2) {
			dev_dbg(bus->dev, "ERROR!! recover failed\n");
			return -1;
		}
	} else {
		dev_dbg(bus->dev, "Don't know how to handle this case?!\n");
		return -1;
	}
	dev_dbg(bus->dev, "Recovery successfully\n");
	return 0;
}

static int ast_i2c_wait_bus_not_busy(struct aspeed_i2c_bus *bus)
{
	int timeout = 2; //TODO number

	while (ast_i2c_read(bus, I2C_CMD_REG) & AST_I2CD_BUS_BUSY_STS) {
		ast_i2c_bus_error_recover(bus);
		if(timeout <= 0)
			break;
		timeout--;
		msleep(2);
	}

	return timeout <= 0 ? EAGAIN : 0;
}

static bool ast_i2c_do_byte_xfer(struct aspeed_i2c_bus *bus)
{
	u32 cmd, data;

	if (bus->send_start) {
		dev_dbg(bus->dev, "%s %c: addr %x start, len %d\n", __func__,
			bus->msg->flags & I2C_M_RD ? 'R' : 'W',
			bus->msg->addr,	bus->msg->len);

		data = bus->msg->addr << 1;
		if (bus->msg->flags & I2C_M_RD)
			data |= 0x1;

		cmd = AST_I2CD_M_TX_CMD | AST_I2CD_M_START_CMD;
		if (bus->send_stop && bus->msg->len == 0)
			cmd |= AST_I2CD_M_STOP_CMD;

		ast_i2c_write(bus, data, I2C_BYTE_BUF_REG);
		ast_i2c_issue_cmd(bus, cmd);

	} else if (bus->msg_pos < bus->msg->len){
		bool is_last = bus->msg_pos + 1 == bus->msg->len;

		dev_dbg(bus->dev, "%s %c%c: addr %x xfer %d, len %d\n",
			__func__,
			bus->msg->flags & I2C_M_RD ? 'R' : 'W',
			bus->send_stop && is_last ? 'T' : '-',
			bus->msg->addr,
			bus->msg_pos, bus->msg->len);

		if (bus->msg->flags & I2C_M_RD) {
			cmd = AST_I2CD_M_RX_CMD;
			if (bus->send_stop && is_last)
				cmd |= AST_I2CD_M_S_RX_CMD_LAST |
					AST_I2CD_M_STOP_CMD;

		} else {
			cmd = AST_I2CD_M_TX_CMD;
			ast_i2c_write(bus, bus->msg->buf[bus->msg_pos],
					I2C_BYTE_BUF_REG);

			if (bus->send_stop && is_last)
				cmd |= AST_I2CD_M_STOP_CMD;
		}
		ast_i2c_issue_cmd(bus, cmd);

	} else {
		return false;
	}

	return true;
}

//TX/Rx Done
static void ast_i2c_master_xfer_done(struct aspeed_i2c_bus *bus)
{
	bool next_msg_queued;

	dev_dbg(bus->dev, "%s xfer %d%c\n", __func__,
			bus->msg_pos,
			bus->send_start ? 'S' : ' ');

	if (bus->send_start) {
		bus->send_start = false;
	} else {

		if (bus->msg->flags & I2C_M_RD) {
			uint8_t data;

			data = (ast_i2c_read(bus, I2C_BYTE_BUF_REG) &
					AST_I2CD_RX_BYTE_BUFFER) >> 8;

			if (bus->query_len) {
				bus->msg->len += data;
				bus->query_len = false;
				dev_dbg(bus->dev, "got rx len: %d\n",
						bus->msg->len -1);
			}
			bus->msg->buf[bus->msg_pos] = data;
		}
		bus->msg_pos++;
	}

	/* queue the next message. If there's none left, we notify the
	 * waiter */
	next_msg_queued = ast_i2c_do_byte_xfer(bus);
	if (!next_msg_queued)
		complete(&bus->cmd_complete);
}

static irqreturn_t aspeed_i2c_bus_irq_handle(struct aspeed_i2c_bus *bus)
{
	const u32 errs = AST_I2CD_INTR_ARBIT_LOSS |
		AST_I2CD_INTR_ABNORMAL |
		AST_I2CD_INTR_SCL_TIMEOUT |
		AST_I2CD_INTR_SDA_DL_TIMEOUT |
		AST_I2CD_INTR_TX_NAK;
	u32 sts, cmd;

	spin_lock(&bus->cmd_lock);

	cmd = ast_i2c_read(bus, I2C_CMD_REG);
	sts = ast_i2c_read(bus, I2C_INTR_STS_REG);

	dev_dbg(bus->dev, "irq! status 0x%08x, cmd 0x%08x\n", sts, cmd);

	sts &= 0x7fff;
	bus->state = cmd >> 19 & 0xf;

	/* ack everything */
	ast_i2c_write(bus, sts, I2C_INTR_STS_REG);

	bus->cmd_err |= sts & errs;
	bus->cmd_pending = cmd & AST_I2CD_CMDS;

	/* if we've seen an error, notify our waiter */
	if (bus->cmd_err) {
		complete(&bus->cmd_complete);

	/* We have a transfer in progress */
	} else if (bus->msg && !bus->cmd_pending) {
		ast_i2c_master_xfer_done(bus);

	/* Other message queued: recovery, error stop. Notify waiters. */
	} else if (bus->cmd_sent && !bus->cmd_pending) {
		complete(&bus->cmd_complete);

	} else {
		dev_err(bus->dev, "Invalid state (msg %p, pending %x)?",
				bus->msg, bus->cmd_pending);
	}

	spin_unlock(&bus->cmd_lock);

	return IRQ_HANDLED;
}

#if 0

	if (AST_I2CD_INTR_SMBUS_ALERT & sts) {
		ast_master_alert_recv(bus);
		sts &= ~AST_I2CD_INTR_SMBUS_ALERT;
	}

	switch(sts) {
	case AST_I2CD_INTR_TX_ACK:
		ast_i2c_write(bus, AST_I2CD_INTR_TX_ACK, I2C_INTR_STS_REG);
		break;
	case AST_I2CD_INTR_TX_ACK | AST_I2CD_INTR_NORMAL_STOP:
		if (bus->send_stop) {
			ast_i2c_write(bus, AST_I2CD_INTR_TX_ACK | AST_I2CD_INTR_NORMAL_STOP, I2C_INTR_STS_REG);
			ast_i2c_master_xfer_done(bus);
		} else {
			dev_dbg(bus->dev,"TODO ...\n");
		}
		break;

	case AST_I2CD_INTR_TX_NAK:
		ast_i2c_write(bus, AST_I2CD_INTR_TX_NAK, I2C_INTR_STS_REG);
		if (bus->msg->flags == I2C_M_IGNORE_NAK) {
			dev_dbg(bus->dev, "I2C_M_IGNORE_NAK next send\n");
			bus->cmd_err = 0;
		} else {
			dev_dbg(bus->dev, "NAK error\n");
			bus->cmd_err = AST_I2CD_INTR_TX_NAK;
		}
		break;

	case AST_I2CD_INTR_TX_NAK | AST_I2CD_INTR_NORMAL_STOP:
		ast_i2c_write(bus, AST_I2CD_INTR_TX_NAK | AST_I2CD_INTR_NORMAL_STOP, I2C_INTR_STS_REG);
		dev_dbg(bus->dev, "M TX NAK | NORMAL STOP \n");
		bus->cmd_err = AST_I2CD_INTR_TX_NAK | AST_I2CD_INTR_NORMAL_STOP;
		complete(&bus->cmd_complete);
		break;

	//Issue : Workaround for I2C slave mode
	case AST_I2CD_INTR_TX_NAK | AST_I2CD_INTR_SLAVE_MATCH:
		dev_err(bus->dev,"error: TX_NAK | SLAVE_MATCH\n");
		break;
	case AST_I2CD_INTR_RX_DONE | AST_I2CD_INTR_SLAVE_MATCH:
		dev_err(bus->dev,"error: RX_DONE | SLAVE_MATCH\n");
		ast_i2c_write(bus, AST_I2CD_INTR_RX_DONE | AST_I2CD_INTR_SLAVE_MATCH, I2C_INTR_STS_REG);
		break;
	case AST_I2CD_INTR_RX_DONE:
		ast_i2c_write(bus, AST_I2CD_INTR_RX_DONE, I2C_INTR_STS_REG);
		ast_i2c_master_xfer_done(bus);
		break;
	case AST_I2CD_INTR_NORMAL_STOP:
		ast_i2c_write(bus, AST_I2CD_INTR_NORMAL_STOP, I2C_INTR_STS_REG);
		bus->cmd_err = 0;
		complete(&bus->cmd_complete);
		break;
	case (AST_I2CD_INTR_RX_DONE | AST_I2CD_INTR_NORMAL_STOP):
		if (bus->send_stop) {
			ast_i2c_write(bus, AST_I2CD_INTR_RX_DONE | AST_I2CD_INTR_NORMAL_STOP, I2C_INTR_STS_REG);
			ast_i2c_master_xfer_done(bus);
		} else {
			dev_dbg(bus->dev,"TODO: RX_DONE | NORMAL_STOP\n");
		}
		break;
	default:
		dev_err(bus->dev, "GR %p : No one care : %x, bus_id %d\n",
				bus->i2c_dev->reg_gr, sts, bus->bus_id);
		BUG();
		return IRQ_NONE;
	}

	return IRQ_HANDLED;

}
#endif


static irqreturn_t i2c_ast_handler(int this_irq, void *data)
{
	unsigned int p;
	unsigned long isr_sts;
	struct ast_i2c_dev *i2c = data;

	isr_sts = readl(i2c->reg_gr);

	for_each_set_bit(p, &isr_sts, 14)
		aspeed_i2c_bus_irq_handle(&i2c->buses[p]);

	/* TODO: use a proper irq chip so we can return the status from each handler? */
	return IRQ_HANDLED;

}
static int ast_i2c_do_msgs_xfer(struct aspeed_i2c_bus *bus,
				struct i2c_msg *msgs, int num)
{
	unsigned long flags;
	int i, ret = 0;
	u32 err, cmd;

	for (i = 0; i < num; i++) {

		spin_lock_irqsave(&bus->cmd_lock, flags);
		bus->msg = &msgs[i];
		bus->msg_pos = 0;
		bus->query_len = bus->msg->flags & I2C_M_RECV_LEN;
		bus->send_start = !(bus->msg->flags & I2C_M_NOSTART);
		bus->send_stop = !!(num == i+1);
		init_completion(&bus->cmd_complete);

		ast_i2c_do_byte_xfer(bus);
		spin_unlock_irqrestore(&bus->cmd_lock, flags);

		ret = wait_for_completion_interruptible_timeout(
					&bus->cmd_complete,
					bus->adap.timeout * HZ);

		spin_lock_irqsave(&bus->cmd_lock, flags);
		err = bus->cmd_err;
		cmd = bus->cmd_sent;
		bus->cmd_sent = 0;
		bus->msg = NULL;
		spin_unlock_irqrestore(&bus->cmd_lock, flags);

		if (!ret) {
			dev_dbg(bus->dev, "controller timed out\n");
			return -EIO;
		}

		if (err != 0) {
			if (cmd & AST_I2CD_M_STOP_CMD) {
				return -ETIMEDOUT;
			} else {
				dev_dbg(bus->dev, "send stop\n");
				ast_i2c_issue_oob_command(bus,
						AST_I2CD_M_STOP_CMD);
				return -EAGAIN;
			}
		}
	}

	return num;
}

static int ast_i2c_xfer(struct i2c_adapter *adap,
			struct i2c_msg *msgs, int num)
{
	struct aspeed_i2c_bus *bus = adap->algo_data;
	int ret, i;
	int sts;

	sts = ast_i2c_read(bus, I2C_CMD_REG);
	dev_dbg(bus->dev, "state[%x], SCL[%d], SDA[%d], BUS[%d]\n",
		(sts >> 19) & 0xf,
		(sts >> 18) & 0x1,
		(sts >> 17) & 0x1,
		(sts >> 16) & 1);
	/*
	 * Wait for the bus to become free.
	 */

	ret = ast_i2c_wait_bus_not_busy(bus);
	if (ret) {
		dev_err(&adap->dev, "i2c_ast: timeout waiting for bus free\n");
		goto out;
	}

	for (i = adap->retries; i >= 0; i--) {
		if (i != 0)
			dev_dbg(&adap->dev, "Do retrying transmission [%d]\n",i);

		ret = ast_i2c_do_msgs_xfer(bus, msgs, num);
		if (ret != -EAGAIN)
			goto out;

		udelay(100);
	}

	ret = -EREMOTEIO;
out:

	return ret;
}

static u32 ast_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_SMBUS_BLOCK_DATA;
}

static const struct i2c_algorithm i2c_ast_algorithm = {
	.master_xfer	= ast_i2c_xfer,
	.functionality	= ast_i2c_functionality,
};

static int aspeed_i2c_add_bus(struct device_node *np,
			      struct ast_i2c_dev *i2c,
			      struct platform_device *pdev)
{
	struct aspeed_i2c_bus *bus;
	struct resource res;
	int ret, bus_num;

	ret = of_property_read_u32(np, "bus", &bus_num);
	if (ret || bus_num >= ARRAY_SIZE(i2c->buses))
		return -ENXIO;

	bus = &i2c->buses[bus_num];

	ret = of_address_to_resource(np, 0, &res);
	if (ret < 0)
		return -ENXIO;
	bus->base = devm_ioremap_resource(&pdev->dev, &res);
	if (IS_ERR(bus->base))
		return PTR_ERR(bus->base);

	/* Initialize the I2C adapter */
	bus->adap.nr = bus_num;
	bus->adap.owner = THIS_MODULE;
	bus->adap.retries = 0;
	bus->adap.timeout = 5;
	bus->adap.algo = &i2c_ast_algorithm;
	bus->adap.algo_data = bus;
	bus->adap.dev.parent = &pdev->dev;
	bus->adap.dev.of_node = np;
	snprintf(bus->adap.name, sizeof(bus->adap.name), "Aspeed i2c at %p",
			bus->base);

	bus->dev = &bus->adap.dev;
	bus->i2c_dev = i2c;

	ret = of_property_read_u32(np, "clock-frequency", &bus->bus_clk);
	if (ret < 0) {
		dev_err(&pdev->dev,
				"Could not read clock-frequency property\n");
		bus->bus_clk = 100000;
	}

	ast_i2c_dev_init(bus);

	ret = i2c_add_numbered_adapter(&bus->adap);
	if (ret < 0)
		return -ENXIO;

	return 0;
}

static int ast_i2c_probe(struct platform_device *pdev)
{
	struct ast_i2c_dev *dev;
	struct resource *res;
	struct device_node *np;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev->reg_gr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dev->reg_gr))
		return PTR_ERR(dev->reg_gr);

	dev->irq = platform_get_irq(pdev, 0);
	if (dev->irq < 0)
		return -ENXIO;

	ret = request_irq(dev->irq, i2c_ast_handler, IRQF_SHARED,
			  "Apseed i2c", dev);
	if (ret)
		return -ENXIO;

	dev->dev = &pdev->dev;

	dev->pclk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(dev->pclk)) {
		dev_err(&pdev->dev, "cannot get pclk for i2c\n");
		return PTR_ERR(dev->pclk);
	}

	for_each_available_child_of_node(pdev->dev.of_node, np) {
		if (!of_device_is_compatible(np, "aspeed,ast2400-i2c-bus"))
			continue;

		ret = aspeed_i2c_add_bus(np, dev, pdev);
		if (ret < 0)
			dev_err(&pdev->dev, "faield to add i2c bus %s\n", np->name);
	}

	platform_set_drvdata(pdev, dev);

	return 0;
}

static const struct of_device_id aspeed_i2c_of_table[] = {
	{ .compatible = "aspeed,ast2400-i2c-common", },
	{ },
};
MODULE_DEVICE_TABLE(of, aspeed_i2c_of_table);

static struct platform_driver i2c_ast_driver = {
	.probe		= ast_i2c_probe,
	.driver         = {
		.name   = KBUILD_MODNAME,
		.of_match_table = aspeed_i2c_of_table,
	},
};

module_platform_driver(i2c_ast_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED AST I2C Bus Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ast_i2c");
