/*
 * ASPEED Static Memory Controller driver
 *
 * Copyright (c) 2015-2016, IBM Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/bug.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nor.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/sysfs.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>

#define DEVICE_NAME	"aspeed-smc"

/*
 * DMAs do not seem to be that fast, so disable by default
 */
static bool use_dma;
module_param(use_dma, bool, 0644);

static unsigned int min_dma_size = 256;
module_param(min_dma_size, uint, 0644);

/* with 100ms we had a couple of timeouts */
static unsigned int dma_timeout = 200;
module_param(dma_timeout, uint, 0644);

/*
 * In user mode all data bytes read or written to the chip decode address
 * range are transferred to or from the SPI bus. The range is treated as a
 * fifo of arbitratry 1, 2, or 4 byte width but each write has to be aligned
 * to its size.  The address within the multiple 8kB range is ignored when
 * sending bytes to the SPI bus.
 *
 * On the arm architecture, as of Linux version 4.3, memcpy_fromio and
 * memcpy_toio on little endian targets use the optimized memcpy routines
 * that were designed for well behavied memory storage.  These routines
 * have a stutter if the source and destination are not both word aligned,
 * once with a duplicate access to the source after aligning to the
 * destination to a word boundary, and again with a duplicate access to
 * the source when the final byte count is not word aligned.
 *
 * When writing or reading the fifo this stutter discards data or sends
 * too much data to the fifo and can not be used by this driver.
 *
 * While the low level io string routines that implement the insl family do
 * the desired accesses and memory increments, the cross architecture io
 * macros make them essentially impossible to use on a memory mapped address
 * instead of a a token from the call to iomap of an io port.
 *
 * These fifo routines use readl and friends to a constant io port and update
 * the memory buffer pointer and count via explicit code. The final updates
 * to len are optimistically suppressed.
 */
static int aspeed_smc_read_from_ahb(void *buf, const void __iomem *src,
				    size_t len)
{
	if ((((unsigned long) src | (unsigned long) buf | len) & 3) == 0) {
		while (len > 3) {
			*(u32 *) buf = readl(src);
			buf += 4;
			src += 4;
			len -= 4;
		}
	}

	while (len--) {
		*(u8 *) buf = readb(src);
		buf += 1;
		src += 1;
	}
	return 0;
}

static int aspeed_smc_write_to_ahb(void __iomem *dst, const void *buf,
				   size_t len)
{
	if ((((unsigned long) dst | (unsigned long) buf | len) & 3) == 0) {
		while (len > 3) {
			u32 val = *(u32 *) buf;

			writel(val, dst);
			buf += 4;
			dst += 4;
			len -= 4;
		}
	}

	while (len--) {
		u8 val = *(u8 *) buf;

		writeb(val, dst);
		buf += 1;
		dst += 1;
	}
	return 0;
}

enum smc_flash_type {
	smc_type_nor = 0,	/* controller connected to nor flash */
	smc_type_nand = 1,	/* controller connected to nand flash */
	smc_type_spi = 2,	/* controller connected to spi flash */
};

struct aspeed_smc_chip;

struct aspeed_smc_info {
	u32 maxsize;		/* maximum size of 1 chip window */
	u8 nce;			/* number of chip enables */
	u8 maxwidth;		/* max width of spi bus */
	bool hastype;		/* flash type field exists in cfg reg */
	u8 we0;			/* shift for write enable bit for ce 0 */
	u8 ctl0;		/* offset in regs of ctl for ce 0 */
	u8 time;		/* offset in regs of timing */
	u8 misc;		/* offset in regs of misc settings */

	void (*set_4b)(struct aspeed_smc_chip *chip);
};

static void aspeed_smc_chip_set_4b_smc_2400(struct aspeed_smc_chip *chip);
static void aspeed_smc_chip_set_4b(struct aspeed_smc_chip *chip);

static const struct aspeed_smc_info fmc_2400_info = {
	.maxsize = 64 * 1024 * 1024,
	.nce = 5,
	.maxwidth = 4,
	.hastype = true,
	.we0 = 16,
	.ctl0 = 0x10,
	.time = 0x94,
	.misc = 0x54,
	.set_4b = aspeed_smc_chip_set_4b,
};

static const struct aspeed_smc_info smc_2400_info = {
	.maxsize = 64 * 1024 * 1024,
	.nce = 1,
	.maxwidth = 2,
	.hastype = false,
	.we0 = 0,
	.ctl0 = 0x04,
	.time = 0x14,
	.misc = 0x10,
	.set_4b = aspeed_smc_chip_set_4b_smc_2400,
};

static const struct aspeed_smc_info fmc_2500_info = {
	.maxsize = 256 * 1024 * 1024,
	.nce = 3,
	.maxwidth = 2,
	.hastype = true,
	.we0 = 16,
	.ctl0 = 0x10,
	.time = 0x94,
	.misc = 0x54,
	.set_4b = aspeed_smc_chip_set_4b,
};

static const struct aspeed_smc_info smc_2500_info = {
	.maxsize = 128 * 1024 * 1024,
	.nce = 2,
	.maxwidth = 2,
	.hastype = false,
	.we0 = 16,
	.ctl0 = 0x10,
	.time = 0x94,
	.misc = 0x54,
	.set_4b = aspeed_smc_chip_set_4b,
};

enum smc_ctl_reg_value {
	smc_base,		/* base value without mode for other commands */
	smc_read,		/* command reg for (maybe fast) reads */
	smc_write,		/* command reg for writes with timings */
	smc_num_ctl_reg_values	/* last value to get count of commands */
};

struct aspeed_smc_controller;

struct aspeed_smc_chip {
	int cs;
	struct aspeed_smc_controller *controller;
	__le32 __iomem *ctl;			/* control register */
	void __iomem *base;			/* base of chip window */
	__le32 ctl_val[smc_num_ctl_reg_values];	/* controls with timing */
	unsigned long phys_base;
	enum smc_flash_type type;		/* what type of flash */
	struct spi_nor nor;
};

struct aspeed_smc_controller {
	struct device *dev;

	struct mutex mutex;			/* controller access mutex */
	const struct aspeed_smc_info *info;	/* type info of controller */
	void __iomem *regs;			/* controller registers */
	void __iomem *windows;			/* per-chip windows resource */

	/* interrupt handling */
	int irq;

	/* dma */
	bool has_dma;
	struct completion dma_done;

	/* dma logging */
	size_t dma_length;
	dma_addr_t dma_addr;			/* bus address of buffer */
	dma_addr_t flash_addr;			/* flash address */

	struct aspeed_smc_chip *chips[0];	/* pointers to attached chips */
};

/*
 * SPI Flash Configuration Register (AST2400 SPI)
 */
#define CONFIG_REG			0x0
#define    CONFIG_ENABLE_CE_INACTIVE	    BIT(1)
#define    CONFIG_WRITE			    BIT(0)

/*
 * SPI Flash Configuration Register (AST2500 SPI)
 * Type setting Register (AST2500 FMC and AST2400 FMC)
 */
#define TYPE_SETTING_REG		0x0
#define    CONFIG_DISABLE_LEGACY	    BIT(31) /* 1 on AST2500 FMC */

#define    CONFIG_CE2_WRITE		    BIT(18)
#define    CONFIG_CE1_WRITE		    BIT(17)
#define    CONFIG_CE0_WRITE		    BIT(16)

#define    CONFIG_CE2_TYPE		    BIT(4) /* FMC only */
#define    CONFIG_CE1_TYPE		    BIT(2) /* FMC only */
#define    CONFIG_CE0_TYPE		    BIT(0) /* FMC only */

/*
 * CE Control Register (AST2500 SPI,FMC and AST2400 FMC)
 */
#define CE_CONTROL_REG			0x4
#define    CE2_ENABLE_CE_INACTIVE           BIT(10)
#define    CE1_ENABLE_CE_INACTIVE           BIT(9)
#define    CE0_ENABLE_CE_INACTIVE           BIT(8)
#define    CE2_CONTROL_EXTENDED		    BIT(2)
#define    CE1_CONTROL_EXTENDED		    BIT(1)
#define    CE0_CONTROL_EXTENDED		    BIT(0)

/* CE0 Control Register (depends on the controller type) */
#define CONTROL_SPI_AAF_MODE BIT(31)
#define CONTROL_SPI_IO_MODE_MASK GENMASK(30, 28)
#define CONTROL_SPI_IO_DUAL_DATA BIT(29)
#define CONTROL_SPI_IO_DUAL_ADDR_DATA (BIT(29) | BIT(28))
#define CONTROL_SPI_IO_QUAD_DATA BIT(30)
#define CONTROL_SPI_IO_QUAD_ADDR_DATA (BIT(30) | BIT(28))
#define CONTROL_SPI_CE_INACTIVE_SHIFT 24
#define CONTROL_SPI_CE_INACTIVE_MASK GENMASK(27, CONTROL_SPI_CE_INACTIVE_SHIFT)
/* 0 = 16T ... 15 = 1T   T=HCLK */
#define CONTROL_SPI_COMMAND_SHIFT 16
#define CONTROL_SPI_DUMMY_CYCLE_COMMAND_OUTPUT BIT(15)
#define CONTROL_SPI_IO_DUMMY_CYCLES_HI BIT(14)
#define CONTROL_SPI_IO_DUMMY_CYCLES_HI_SHIFT 14
#define CONTROL_SPI_IO_ADDRESS_4B BIT(13) /* AST2400 SPI */
#define CONTROL_SPI_CLK_DIV4 BIT(13) /* others */
#define CONTROL_SPI_RW_MERGE BIT(12)
#define CONTROL_SPI_IO_DUMMY_CYCLES_LO_SHIFT 6
#define CONTROL_SPI_IO_DUMMY_CYCLES_LO GENMASK(7, \
				       CONTROL_SPI_IO_DUMMY_CYCLES_LO_SHIFT)
#define CONTROL_SPI_IO_DUMMY_CYCLES_MASK (CONTROL_SPI_IO_DUMMY_CYCLES_HI | \
					  CONTROL_SPI_IO_DUMMY_CYCLES_LO)
#define CONTROL_SPI_IO_DUMMY_CYCLES_SET(dummy)				\
	(((((dummy) >> 2) & 0x1) << CONTROL_SPI_IO_DUMMY_CYCLES_HI_SHIFT) | \
	(((dummy) & 0x3) << CONTROL_SPI_IO_DUMMY_CYCLES_LO_SHIFT))

#define CONTROL_SPI_CLOCK_FREQ_SEL_SHIFT 8
#define CONTROL_SPI_CLOCK_FREQ_SEL_MASK GENMASK(11, \
					CONTROL_SPI_CLOCK_FREQ_SEL_SHIFT)
#define CONTROL_SPI_LSB_FIRST BIT(5)
#define CONTROL_SPI_CLOCK_MODE_3 BIT(4)
#define CONTROL_SPI_IN_DUAL_DATA BIT(3)
#define CONTROL_SPI_CE_STOP_ACTIVE_CONTROL BIT(2)
#define CONTROL_SPI_COMMAND_MODE_MASK GENMASK(1, 0)
#define CONTROL_SPI_COMMAND_MODE_NORMAL (0)
#define CONTROL_SPI_COMMAND_MODE_FREAD (1)
#define CONTROL_SPI_COMMAND_MODE_WRITE (2)
#define CONTROL_SPI_COMMAND_MODE_USER (3)

#define CONTROL_SPI_KEEP_MASK (CONTROL_SPI_AAF_MODE | \
	CONTROL_SPI_CE_INACTIVE_MASK | CONTROL_SPI_CLK_DIV4 | \
	CONTROL_SPI_IO_DUMMY_CYCLES_MASK | CONTROL_SPI_CLOCK_FREQ_SEL_MASK | \
	CONTROL_SPI_LSB_FIRST | CONTROL_SPI_CLOCK_MODE_3)

/* Interrupt Control and Status Register */
#define INTERRUPT_STATUS_REG		0x08
#define     INTERRUPT_DMA_ENABLE	    BIT(3)
#define     INTERRUPT_DMA_STATUS	    BIT(11)

/* Segment Address Registers */
#define SEGMENT_ADDR_REG0		0x30
#define     SEGMENT_ADDR_START(_r)	    ((((_r) >> 16) & 0xFF) << 23)
#define     SEGMENT_ADDR_END(_r)	    ((((_r) >> 24) & 0xFF) << 23)


/* DMA Registers */
#define DMA_CONTROL_REG			0x80
#define     DMA_ENABLE			    BIT(0)
#define     DMA_WRITE			    BIT(1)

#define DMA_FLASH_BASE_REG		0x84
#define DMA_DRAM_BASE_REG		0x88
#define DMA_LENGTH_REG			0x8c

static void aspeed_smc_dma_done(struct aspeed_smc_controller *controller)
{
	writel(0, controller->regs + INTERRUPT_STATUS_REG);
	writel(0, controller->regs + DMA_CONTROL_REG);
}

static int aspeed_smc_dma_wait(struct aspeed_smc_chip *chip)
{
	struct aspeed_smc_controller *controller = chip->controller;

	if (!wait_for_completion_timeout(&controller->dma_done,
					 msecs_to_jiffies(dma_timeout))) {
		dev_err(chip->nor.dev, "timeout waiting for DMA interrupt "
			"addr@%.8x faddr@%.8x size=%x "
			"dram@%.8x flash@%.8x done=%x\n",
			controller->dma_addr,
			controller->flash_addr,
			controller->dma_length,
			readl(controller->regs + DMA_DRAM_BASE_REG),
			readl(controller->regs + DMA_FLASH_BASE_REG),
			readl(controller->regs + DMA_LENGTH_REG));
		return -ETIMEDOUT;
	}

	return 0;
}

#define DMA_LENGTH(x) (((x) - 4) & ~0xFE000003)
#define DMA_ADDR(x) ((x) & ~0x00000003)

static inline void aspeed_smc_chip_configure(struct aspeed_smc_chip *chip,
					     u32 ctl)
{
	ctl |= CONTROL_SPI_CE_STOP_ACTIVE_CONTROL;
	writel(ctl, chip->ctl);

	ctl &= ~CONTROL_SPI_CE_STOP_ACTIVE_CONTROL;
	writel(ctl, chip->ctl);
}

/*
 * TODO: configure FREAD mode
 */
static int aspeed_smc_dma_start(struct aspeed_smc_chip *chip,
				u32 offset, void *buf, size_t length,
				int is_write)
{
	struct aspeed_smc_controller *controller = chip->controller;
	dma_addr_t dma_addr, flash_addr;
	int ret;

	aspeed_smc_chip_configure(chip, is_write ? chip->ctl_val[smc_write] :
		chip->ctl_val[smc_read]);

	dev_dbg(chip->nor.dev, "DMA %s to=0x%08x len=0x%08x\n",
		is_write ? "write" : "read", offset, length);

	dma_addr = dma_map_single(chip->nor.dev, buf, length,
				  (is_write ? DMA_TO_DEVICE : DMA_FROM_DEVICE));

	if (unlikely(dma_mapping_error(chip->nor.dev, dma_addr))) {
		dev_err(chip->nor.dev, "Failed to dma_map_single()\n");
		ret = -ENOMEM;
		goto out;
	}
	flash_addr = chip->phys_base + offset;

	controller->dma_length = length;
	controller->dma_addr = dma_addr;
	controller->flash_addr = flash_addr;

	reinit_completion(&controller->dma_done);

	writel(0, controller->regs + DMA_CONTROL_REG);
	writel(DMA_ADDR(flash_addr), controller->regs +
	       DMA_FLASH_BASE_REG);
	writel(DMA_ADDR(dma_addr), controller->regs + DMA_DRAM_BASE_REG);
	writel(DMA_LENGTH(length), controller->regs + DMA_LENGTH_REG);

	writel(INTERRUPT_DMA_ENABLE,
	       controller->regs + INTERRUPT_STATUS_REG);

	writel(DMA_ENABLE | (is_write << 1),
	       controller->regs + DMA_CONTROL_REG);

	ret = aspeed_smc_dma_wait(chip);
	if (ret)
		aspeed_smc_dma_done(controller);

	dma_unmap_single(chip->nor.dev,
			 controller->dma_addr, controller->dma_length,
			 (is_write ? DMA_TO_DEVICE : DMA_FROM_DEVICE));
out:
	aspeed_smc_chip_configure(chip, chip->ctl_val[smc_read]);
	return ret;
}

static u32 spi_control_fill_opcode(u8 opcode)
{
	return ((u32)(opcode)) << CONTROL_SPI_COMMAND_SHIFT;
}

static inline u32 aspeed_smc_chip_write_bit(struct aspeed_smc_chip *chip)
{
	return ((u32)1 << (chip->controller->info->we0 + chip->cs));
}

static void aspeed_smc_chip_check_config(struct aspeed_smc_chip *chip)
{
	struct aspeed_smc_controller *controller = chip->controller;
	u32 reg;

	reg = readl(controller->regs + CONFIG_REG);

	if (!(reg & aspeed_smc_chip_write_bit(chip))) {
		dev_dbg(controller->dev,
			"config write is not set ! @%p: 0x%08x\n",
			controller->regs + CONFIG_REG, reg);
		reg |= aspeed_smc_chip_write_bit(chip);
		writel(reg, controller->regs + CONFIG_REG);
	}
}

static void aspeed_smc_start_user(struct spi_nor *nor)
{
	struct aspeed_smc_chip *chip = nor->priv;
	u32 ctl = chip->ctl_val[smc_base];

	/*
	 * When the chip is controlled in user mode, we need write
	 * access to send the opcodes to it. So check the config.
	 */
	aspeed_smc_chip_check_config(chip);

	ctl |= CONTROL_SPI_COMMAND_MODE_USER |
		CONTROL_SPI_CE_STOP_ACTIVE_CONTROL;
	writel(ctl, chip->ctl);

	ctl &= ~CONTROL_SPI_CE_STOP_ACTIVE_CONTROL;
	writel(ctl, chip->ctl);
}

static void aspeed_smc_stop_user(struct spi_nor *nor)
{
	struct aspeed_smc_chip *chip = nor->priv;

	u32 ctl = chip->ctl_val[smc_read];
	u32 ctl2 = ctl | CONTROL_SPI_COMMAND_MODE_USER |
		CONTROL_SPI_CE_STOP_ACTIVE_CONTROL;

	writel(ctl2, chip->ctl);	/* stop user CE control */
	writel(ctl, chip->ctl);		/* default to fread or read */
}

static int aspeed_smc_read_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	struct aspeed_smc_chip *chip = nor->priv;

	mutex_lock(&chip->controller->mutex);

	aspeed_smc_start_user(nor);
	aspeed_smc_write_to_ahb(chip->base, &opcode, 1);
	aspeed_smc_read_from_ahb(buf, chip->base, len);
	aspeed_smc_stop_user(nor);

	mutex_unlock(&chip->controller->mutex);

	return 0;
}

static int aspeed_smc_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf,
				int len)
{
	struct aspeed_smc_chip *chip = nor->priv;

	mutex_lock(&chip->controller->mutex);

	aspeed_smc_start_user(nor);
	aspeed_smc_write_to_ahb(chip->base, &opcode, 1);
	aspeed_smc_write_to_ahb(chip->base, buf, len);
	aspeed_smc_stop_user(nor);

	mutex_unlock(&chip->controller->mutex);

	return 0;
}

static void aspeed_smc_send_cmd_addr(struct spi_nor *nor, u8 cmd, u32 addr)
{
	struct aspeed_smc_chip *chip = nor->priv;
	__be32 temp;
	u32 cmdaddr;

	switch (nor->addr_width) {
	default:
		WARN_ONCE(1, "Unexpected address width %u, defaulting to 3\n",
			  nor->addr_width);
		/* FALLTHROUGH */
	case 3:
		cmdaddr = addr & 0xFFFFFF;

		cmdaddr |= (u32)cmd << 24;

		temp = cpu_to_be32(cmdaddr);
		aspeed_smc_write_to_ahb(chip->base, &temp, 4);
		break;
	case 4:
		temp = cpu_to_be32(addr);
		aspeed_smc_write_to_ahb(chip->base, &cmd, 1);
		aspeed_smc_write_to_ahb(chip->base, &temp, 4);
		break;
	}
}

static int aspeed_smc_read_user(struct spi_nor *nor, loff_t from, size_t len,
				size_t *retlen, u_char *read_buf)
{
	struct aspeed_smc_chip *chip = nor->priv;
	int ret;
	u32 ctl;
	int i;
	u8 dummy = 0xFF;

	mutex_lock(&chip->controller->mutex);

	/*
	 * Try DMA transfer when size and alignment are correct. In case
	 * of failure, just restart using the IO mode.
	 */
	if (!(from & 0x3) && !(len & 0x3) && (len >= min_dma_size) &&
	    chip->controller->has_dma && use_dma) {
		ret = aspeed_smc_dma_start(chip, from, read_buf, len, 0);
		if (!ret)
			goto out;
		dev_err(chip->nor.dev, "DMA read failed: %d", ret);
	}

	aspeed_smc_start_user(nor);
	aspeed_smc_send_cmd_addr(nor, nor->read_opcode, from);

	/* Send dummy bytes */
	for (i = 0; i < chip->nor.read_dummy / 8; ++i)
		aspeed_smc_write_to_ahb(chip->base, &dummy, 1);

	if (chip->nor.flash_read == SPI_NOR_DUAL) {
		/* Switch to dual I/O mode for data cycle */
		ctl = readl(chip->ctl) & ~CONTROL_SPI_IO_MODE_MASK;
		ctl |= CONTROL_SPI_IO_DUAL_DATA;
		writel(ctl, chip->ctl);
	}

	aspeed_smc_read_from_ahb(read_buf, chip->base, len);
	aspeed_smc_stop_user(nor);

out:
	mutex_unlock(&chip->controller->mutex);

	*retlen += len;
	return 0;
}

static void aspeed_smc_write_user(struct spi_nor *nor, loff_t to, size_t len,
				  size_t *retlen, const u_char *write_buf)
{
	struct aspeed_smc_chip *chip = nor->priv;
	int ret;

	mutex_lock(&chip->controller->mutex);

	/*
	 * Try DMA transfer when size and alignment are correct. In case
	 * of failure, just restart using the IO mode.
	 */
	if (!(to & 0x3) && !(len & 0x3) && (len >= min_dma_size) &&
	    chip->controller->has_dma && use_dma) {
		ret = aspeed_smc_dma_start(chip, to, (void *)write_buf,
					   len, 1);
		if (!ret)
			goto out;
		dev_err(chip->nor.dev, "DMA write failed: %d", ret);
	}

	aspeed_smc_start_user(nor);
	aspeed_smc_send_cmd_addr(nor, nor->program_opcode, to);
	aspeed_smc_write_to_ahb(chip->base, write_buf, len);
	aspeed_smc_stop_user(nor);

out:
	mutex_unlock(&chip->controller->mutex);

	*retlen += len;
}

static irqreturn_t aspeed_smc_irq(int irq, void *arg)
{
	struct aspeed_smc_controller *controller = arg;
	struct device *dev = controller->dev;
	irqreturn_t ret = IRQ_NONE;
	u32 dma_ctl = readl(controller->regs + DMA_CONTROL_REG);
	u32 status = readl(controller->regs + INTERRUPT_STATUS_REG);

	dev_dbg(dev, "received IRQ. status: %x", status);

	if (!(status & INTERRUPT_DMA_ENABLE) || !(dma_ctl & DMA_ENABLE)) {
		dev_err(dev, "No DMA. bad IRQ status: %x", status);
		goto out;
	}

	if (!(status & INTERRUPT_DMA_STATUS)) {
		dev_err(dev, "DMA still in progress. length %d\n",
			readl(controller->regs + DMA_LENGTH_REG));
		goto out;
	}

	ret = IRQ_HANDLED;
	aspeed_smc_dma_done(controller);
	complete(&controller->dma_done);

out:
	return ret;
}

static int aspeed_smc_config_irq(struct aspeed_smc_controller *controller,
				 struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int rc;

	controller->irq = platform_get_irq(pdev, 0);
	if (!controller->irq)
		return -ENODEV;

	rc = devm_request_irq(dev, controller->irq, aspeed_smc_irq, IRQF_SHARED,
			      DEVICE_NAME, controller);
	if (rc < 0) {
		dev_warn(dev, "Unable to request IRQ %d\n", controller->irq);
		controller->irq = 0;
		return rc;
	}

	dev_info(dev, "Using IRQ %d\n", controller->irq);
	return 0;
}

static void aspeed_smc_dma_setup(struct aspeed_smc_controller *controller,
				 struct platform_device *pdev)
{
	init_completion(&controller->dma_done);

	controller->has_dma = false;
	if (of_get_property(controller->dev->of_node, "aspeed,fmc-has-dma",
			    NULL))
		controller->has_dma = !aspeed_smc_config_irq(controller, pdev);

	if (controller->has_dma)
		dev_info(controller->dev, "DMA support %sactivated.\n",
			 use_dma ? "" : "de");
	else
		dev_info(controller->dev, "no DMA support.\n");
}

static int aspeed_smc_remove(struct platform_device *dev)
{
	struct aspeed_smc_chip *chip;
	struct aspeed_smc_controller *controller = platform_get_drvdata(dev);
	int n;

	for (n = 0; n < controller->info->nce; n++) {
		chip = controller->chips[n];
		if (chip)
			mtd_device_unregister(&chip->nor.mtd);
	}

	return 0;
}

const struct of_device_id aspeed_smc_matches[] = {
	{ .compatible = "aspeed,ast2400-fmc", .data = &fmc_2400_info },
	{ .compatible = "aspeed,ast2400-smc", .data = &smc_2400_info },
	{ .compatible = "aspeed,ast2500-fmc", .data = &fmc_2500_info },
	{ .compatible = "aspeed,ast2500-smc", .data = &smc_2500_info },
	{ }
};
MODULE_DEVICE_TABLE(of, aspeed_smc_matches);

static struct platform_device *
of_platform_device_create_or_find(struct device_node *child,
				  struct device *parent)
{
	struct platform_device *cdev;

	cdev = of_platform_device_create(child, NULL, parent);
	if (!cdev)
		cdev = of_find_device_by_node(child);
	return cdev;
}

static void __iomem *window_start(struct aspeed_smc_controller *controller,
				  struct resource *r, unsigned int n)
{
	u32 offset = 0;
	u32 reg;

	if (controller->info->nce > 1) {
		reg = readl(controller->regs + SEGMENT_ADDR_REG0 + n * 4);

		if (SEGMENT_ADDR_START(reg) >= SEGMENT_ADDR_END(reg))
			return NULL;

		offset = SEGMENT_ADDR_START(reg) - r->start;
	}

	return controller->windows + offset;
}

static void aspeed_smc_chip_enable_write(struct aspeed_smc_chip *chip)
{
	struct aspeed_smc_controller *controller = chip->controller;
	u32 reg;

	reg = readl(controller->regs + CONFIG_REG);

	reg |= aspeed_smc_chip_write_bit(chip);
	writel(reg, controller->regs + CONFIG_REG);
}

static void aspeed_smc_chip_set_type(struct aspeed_smc_chip *chip, int type)
{
	struct aspeed_smc_controller *controller = chip->controller;
	u32 reg;

	reg = readl(controller->regs + CONFIG_REG);

	chip->type = type;

	reg &= ~(3 << (chip->cs * 2));
	reg |= chip->type << (chip->cs * 2);
	writel(reg, controller->regs + CONFIG_REG);
}

/*
 * The AST2500 FMC and AST2400 FMC flash controllers should be
 * strapped by hardware, or autodetected, but the AST2500 SPI flash
 * needs to be set.
 */
static void aspeed_smc_chip_set_4b(struct aspeed_smc_chip *chip)
{
	struct aspeed_smc_controller *controller = chip->controller;
	u32 reg;

	if (chip->controller->info == &smc_2500_info) {
		reg = readl(controller->regs + CE_CONTROL_REG);
		reg |= 1 << chip->cs;
		writel(reg, controller->regs + CE_CONTROL_REG);
	}
}

/*
 * The AST2400 SPI flash controller does not have a CE Control
 * register. It uses the CE0 control register to set 4Byte mode at the
 * controller level.
 */
static void aspeed_smc_chip_set_4b_smc_2400(struct aspeed_smc_chip *chip)
{
	chip->ctl_val[smc_base] |= CONTROL_SPI_IO_ADDRESS_4B;
	chip->ctl_val[smc_read] |= CONTROL_SPI_IO_ADDRESS_4B;
}

static int aspeed_smc_chip_setup_init(struct aspeed_smc_chip *chip,
				      struct resource *r)
{
	struct aspeed_smc_controller *controller = chip->controller;
	const struct aspeed_smc_info *info = controller->info;
	u32 reg, base_reg;

	/*
	 * Always turn on the write enable bit to allow opcodes to be
	 * sent in user mode.
	 */
	aspeed_smc_chip_enable_write(chip);

	/* The driver only supports SPI type flash for the moment */
	if (info->hastype)
		aspeed_smc_chip_set_type(chip, smc_type_spi);

	/*
	 * Configure chip base address in memory
	 */
	chip->base = window_start(controller, r, chip->cs);
	if (!chip->base) {
		dev_warn(chip->nor.dev, "CE segment window closed.\n");
		return -1;
	}

	chip->phys_base = r->start;

	/*
	 * Read the existing control register to get basic values.
	 *
	 * XXX This register probably needs more sanitation.
	 *
	 * Do we need support for mode 3 vs mode 0 clock phasing?
	 */
	reg = readl(chip->ctl);
	dev_dbg(controller->dev, "control register: %08x\n", reg);

	base_reg = reg & CONTROL_SPI_KEEP_MASK;
	if (base_reg != reg) {
		dev_info(controller->dev,
			 "control register changed to: %08x\n",
			 base_reg);
	}
	chip->ctl_val[smc_base] = base_reg;

	/*
	 * Retain the prior value of the control register as the
	 * default if it was normal access mode. Otherwise start with
	 * the sanitized base value set to read mode.
	 */
	if ((reg & CONTROL_SPI_COMMAND_MODE_MASK) ==
	    CONTROL_SPI_COMMAND_MODE_NORMAL)
		chip->ctl_val[smc_read] = reg;
	else
		chip->ctl_val[smc_read] = chip->ctl_val[smc_base] |
			CONTROL_SPI_COMMAND_MODE_NORMAL;

	dev_dbg(controller->dev, "default control register: %08x\n",
		 chip->ctl_val[smc_read]);
	return 0;
}

static int aspeed_smc_chip_setup_finish(struct aspeed_smc_chip *chip)
{
	struct aspeed_smc_controller *controller = chip->controller;
	const struct aspeed_smc_info *info = controller->info;
	u32 cmd;

	if (chip->nor.addr_width == 4 && info->set_4b)
		info->set_4b(chip);

	/*
	 * base mode has not been optimized yet. use it for writes.
	 */
	chip->ctl_val[smc_write] = chip->ctl_val[smc_base] |
		spi_control_fill_opcode(chip->nor.program_opcode) |
		CONTROL_SPI_COMMAND_MODE_WRITE;

	dev_dbg(controller->dev, "write control register: %08x\n",
		 chip->ctl_val[smc_write]);

	/*
	 * XXX TODO
	 * Adjust clocks if fast read and write are supported.
	 * Interpret spi-nor flags to adjust controller settings.
	 * Check if resource size big enough for detected chip and
	 * add support assisted (normal or fast-) read and dma.
	 */
	switch (chip->nor.flash_read) {
	case SPI_NOR_NORMAL:
		cmd = CONTROL_SPI_COMMAND_MODE_NORMAL;
		break;
	case SPI_NOR_FAST:
		cmd = CONTROL_SPI_COMMAND_MODE_FREAD;
		break;
	case SPI_NOR_DUAL:
		cmd = CONTROL_SPI_COMMAND_MODE_FREAD |
			CONTROL_SPI_IO_DUAL_DATA;
		break;
	default:
		dev_err(chip->nor.dev, "unsupported SPI read mode\n");
		return -EINVAL;
	}

	chip->ctl_val[smc_read] |= cmd |
		spi_control_fill_opcode(chip->nor.read_opcode) |
		CONTROL_SPI_IO_DUMMY_CYCLES_SET(chip->nor.read_dummy / 8);

	dev_dbg(controller->dev, "read control register: %08x\n",
		chip->ctl_val[smc_read]);
	return 0;
}

static int aspeed_smc_probe(struct platform_device *pdev)
{
	struct aspeed_smc_controller *controller;
	const struct of_device_id *match;
	const struct aspeed_smc_info *info;
	struct resource *r;
	struct device_node *child;
	int err = 0;
	unsigned int n;

	match = of_match_device(aspeed_smc_matches, &pdev->dev);
	if (!match || !match->data)
		return -ENODEV;
	info = match->data;

	controller = devm_kzalloc(&pdev->dev, sizeof(*controller) +
		info->nce * sizeof(controller->chips[0]), GFP_KERNEL);
	if (!controller)
		return -ENOMEM;
	controller->info = info;

	mutex_init(&controller->mutex);
	platform_set_drvdata(pdev, controller);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	controller->regs = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(controller->regs))
		return PTR_ERR(controller->regs);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	controller->windows = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(controller->windows))
		return PTR_ERR(controller->windows);

	controller->dev = &pdev->dev;

	aspeed_smc_dma_setup(controller, pdev);

	/* The pinmux or bootloader will disable the legacy mode controller */

	/*
	 * XXX Need to add arbitration to the SMC (BIOS) controller if access
	 * is shared by the host.
	 */
	for_each_available_child_of_node(controller->dev->of_node, child) {
		struct platform_device *cdev;
		struct aspeed_smc_chip *chip;

		/* This version does not support nand or nor flash devices. */
		if (!of_device_is_compatible(child, "jedec,spi-nor"))
			continue;

		/*
		 * create a platform device from the of node.  If the device
		 * already was created (eg from a prior bind/unbind cycle)
		 * reuse it.
		 *
		 * The creating the device node for the child here allows its
		 * use for error reporting via dev_err below.
		 */
		cdev = of_platform_device_create_or_find(child,
							 controller->dev);
		if (!cdev)
			continue;

		err = of_property_read_u32(child, "reg", &n);
		if (err == -EINVAL && info->nce == 1)
			n = 0;
		else if (err || n >= info->nce)
			continue;
		if (controller->chips[n]) {
			dev_err(&cdev->dev,
				"chip-id %u already in use in use by %s\n",
				n, dev_name(controller->chips[n]->nor.dev));
			continue;
		}

		chip = devm_kzalloc(controller->dev, sizeof(*chip), GFP_KERNEL);
		if (!chip)
			continue;
		chip->controller = controller;
		chip->ctl = controller->regs + info->ctl0 + n * 4;
		chip->cs = n;

		chip->nor.dev = &cdev->dev;
		chip->nor.priv = chip;
		spi_nor_set_flash_node(&chip->nor, child);
		chip->nor.mtd.name = of_get_property(child, "label", NULL);
		chip->nor.read = aspeed_smc_read_user;
		chip->nor.write = aspeed_smc_write_user;
		chip->nor.read_reg = aspeed_smc_read_reg;
		chip->nor.write_reg = aspeed_smc_write_reg;

		err = aspeed_smc_chip_setup_init(chip, r);
		if (err)
			continue;

		err = spi_nor_scan(&chip->nor, NULL, SPI_NOR_DUAL);
		if (err)
			continue;

		err = aspeed_smc_chip_setup_finish(chip);
		if (err)
			continue;

		err = mtd_device_register(&chip->nor.mtd, NULL, 0);
		if (err)
			continue;
		controller->chips[n] = chip;
	}

	/* Were any children registered? */
	for (n = 0; n < info->nce; n++)
		if (controller->chips[n])
			break;

	if (n == info->nce)
		return -ENODEV;

	return 0;
}

static struct platform_driver aspeed_smc_driver = {
	.probe = aspeed_smc_probe,
	.remove = aspeed_smc_remove,
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = aspeed_smc_matches,
	}
};

module_platform_driver(aspeed_smc_driver);

MODULE_DESCRIPTION("ASPEED Static Memory Controller Driver");
MODULE_AUTHOR("Milton Miller");
MODULE_LICENSE("GPL v2");
