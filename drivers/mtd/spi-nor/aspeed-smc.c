/*
 * ASPEED Static Memory Controller driver
 * Copyright 2016 IBM Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

/* See comment by aspeed_smc_from_fifo */
#ifdef CONFIG_ARM
#define IO_SPACE_LIMIT (~0UL)
#endif

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

/*
 * On the arm architecture, as of Linux version 4.3, memcpy_fromio
 * stutters discarding some of the bytes read if the destination is
 * unaligned, so we can't use it for reading from a fifo to a buffer
 * of unknown alignment.  Instead use the ins (l, w, b) family
 * to read from the fifo.   However, ARM tries to hide io port
 * accesses from drivers unless there is a PCMCIA or PCI device, so
 * we define the limit before all include files.  There is a
 * check in probe to make sure this will work, as long as the
 * architecture uses an identity iomap.
 */

static void aspeed_smc_from_fifo(void *buf, const void __iomem *iop, size_t len)
{
	unsigned long io = (__force unsigned long)iop;

	if (!len)
		return;

	/* Expect a 4 byte input port.  Otherwise just read bytes */
	if (unlikely(io & 3)) {
		insb(io, buf, len);
		return;
	}

	/* Align target to word: first byte then half word */
	if ((unsigned long)buf & 1) {
		*(u8 *)buf = inb(io);
		buf++;
		len--;
	}
	if (((unsigned long)buf & 2) && (len >= 2)) {
		*(u16 *)buf = inw(io);
		buf += 2;
		len -= 2;
	}
	/* Transfer words, then remaining halfword and remaining byte */
	if (len >= 4) {
		insl(io, buf, len >> 2);
		buf += len & ~3;
	}
	if (len & 2) {
		*(u16 *)buf = inw(io);
		buf += 2;
	}
	if (len & 1) {
		*(u8 *)buf = inb(io);
	}
}

static void aspeed_smc_to_fifo(void __iomem *iop, const void *buf, size_t len)
{
	unsigned long io = (__force unsigned long)iop;

	if (!len)
		return;

	/* Expect a 4 byte output port.  Otherwise just write bytes */
	if (io & 3) {
		outsb(io, buf, len);
		return;
	}

	/* Align target to word: first byte then half word */
	if ((unsigned long)buf & 1) {
		outb(*(u8 *)buf, io);
		buf++;
		len--;
	}
	if (((unsigned long)buf & 2) && (len >= 2)) {
		outw(*(u16 *)buf, io);
		buf += 2;
		len -= 2;
	}
	/* Transfer words, then remaining halfword and remaining byte */
	if (len >= 4) {
		outsl(io, buf, len >> 2);
		buf += len & ~(size_t)3;
	}
	if (len & 2) {
		outw(*(u16 *)buf, io);
		buf += 2;
	}
	if (len & 1) {
		outb(*(u8 *)buf, io);
	}
}

enum smc_flash_type {
	smc_type_nor = 0,	/* controller connected to nor flash */
	smc_type_nand = 1,	/* controller connected to nand flash */
	smc_type_spi = 2,	/* controller connected to spi flash */
};

struct aspeed_smc_info {
	u8 nce;			/* number of chip enables */
	u8 maxwidth;		/* max width of spi bus */
	bool hasdma;		/* has dma engine */
	bool hastype;		/* type shift for ce 0 in cfg reg */
	u8 we0;			/* we shift for ce 0 in cfg reg */
	u8 ctl0;		/* offset in regs of ctl for ce 0 */
	u8 cfg;			/* offset in regs of cfg */
	u8 time;		/* offset in regs of timing */
	u8 misc;		/* offset in regs of misc settings */
};

static struct aspeed_smc_info fmc_info = {
	.nce = 5,
	.maxwidth = 4,
	.hasdma = true,
	.hastype = true,
	.we0 = 16,
	.ctl0 = 0x10,
	.cfg = 0x00,
	.time = 0x54,
	.misc = 0x50,
};

static struct aspeed_smc_info smc_info = {
	.nce = 1,
	.maxwidth = 2,
	.hasdma = false,
	.hastype = false,
	.we0 = 0,
	.ctl0 = 0x04,
	.cfg = 0x00,
	.time = 0x14,
	.misc = 0x10,
};

enum smc_ctl_reg_value {
	smc_base,		/* base value without mode for other commands */
	smc_read,		/* command reg for (maybe fast) reads */
	smc_write,		/* command reg for writes with timings */
	smc_num_ctl_reg_values	/* last value to get count of commands */
};

struct aspeed_smc_controller;

struct aspeed_smc_chip {
	struct aspeed_smc_controller *controller;
	__le32 __iomem *ctl;			/* control register */
	void __iomem *base;			/* base of chip window */
	__le32 ctl_val[smc_num_ctl_reg_values];	/* controls with timing */
	enum smc_flash_type type;		/* what type of flash */
	struct spi_nor nor;
};

struct aspeed_smc_controller {
	struct mutex mutex;			/* controller access mutex */
	const struct aspeed_smc_info *info;	/* type info of controller */
	void __iomem *regs;			/* controller registers */
	struct aspeed_smc_chip *chips[0];	/* attached chips */
};

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
#define CONTROL_SPI_IO_DUMMY_CYCLES_HI_SHIFT (14 - 2)
#define CONTROL_SPI_IO_ADDRESS_4B BIT(13) /* FMC, LEGACY */
#define CONTROL_SPI_CLK_DIV4 BIT(13) /* BIOS */
#define CONTROL_SPI_RW_MERGE BIT(12)
#define CONTROL_SPI_IO_DUMMY_CYCLES_LO_SHIFT 6
#define CONTROL_SPI_IO_DUMMY_CYCLES_LO GENMASK(7, CONTROL_SPI_IO_DUMMY_CYCLES_LO_SHIFT)
#define CONTROL_SPI_IO_DUMMY_CYCLES_MASK (CONTROL_SPI_IO_DUMMY_CYCLES_HI | \
					  CONTROL_SPI_IO_DUMMY_CYCLES_LO)
#define CONTROL_SPI_CLOCK_FREQ_SEL_SHIFT 8
#define CONTROL_SPI_CLOCK_FREQ_SEL_MASK GENMASK(11, CONTROL_SPI_CLOCK_FREQ_SEL_SHIFT)
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
	CONTROL_SPI_CE_INACTIVE_MASK | CONTROL_SPI_IO_ADDRESS_4B | \
	CONTROL_SPI_IO_DUMMY_CYCLES_MASK | CONTROL_SPI_CLOCK_FREQ_SEL_MASK | \
	CONTROL_SPI_LSB_FIRST | CONTROL_SPI_CLOCK_MODE_3)

#define CONTROL_SPI_CLK_DIV4 BIT(13) /* BIOS */

static u32 spi_control_fill_opcode(u8 opcode)
{
	return ((u32)(opcode)) << CONTROL_SPI_COMMAND_SHIFT;
}

static void aspeed_smc_start_user(struct spi_nor *nor)
{
	struct aspeed_smc_chip *chip = nor->priv;
	u32 ctl = chip->ctl_val[smc_base];

	mutex_lock(&chip->controller->mutex);

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

	mutex_unlock(&chip->controller->mutex);
}

static int aspeed_smc_read_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	struct aspeed_smc_chip *chip = nor->priv;

	aspeed_smc_start_user(nor);
	aspeed_smc_to_fifo(chip->base, &opcode, 1);
	aspeed_smc_from_fifo(buf, chip->base, len);
	aspeed_smc_stop_user(nor);

	return 0;
}

static int aspeed_smc_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf,
				int len)
{
	struct aspeed_smc_chip *chip = nor->priv;

	aspeed_smc_start_user(nor);
	aspeed_smc_to_fifo(chip->base, &opcode, 1);
	aspeed_smc_to_fifo(chip->base, buf, len);
	aspeed_smc_stop_user(nor);

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
		aspeed_smc_to_fifo(chip->base, &temp, 4);
		break;
	case 4:
		temp = cpu_to_be32(addr);
		aspeed_smc_to_fifo(chip->base, &cmd, 1);
		aspeed_smc_to_fifo(chip->base, &temp, 4);
		break;
	}
}

static int aspeed_smc_read_user(struct spi_nor *nor, loff_t from, size_t len,
				size_t *retlen, u_char *read_buf)
{
	struct aspeed_smc_chip *chip = nor->priv;

	aspeed_smc_start_user(nor);
	aspeed_smc_send_cmd_addr(nor, nor->read_opcode, from);
	aspeed_smc_from_fifo(read_buf, chip->base, len);
	*retlen += len;
	aspeed_smc_stop_user(nor);

	return 0;
}

static void aspeed_smc_write_user(struct spi_nor *nor, loff_t to, size_t len,
				  size_t *retlen, const u_char *write_buf)
{
	struct aspeed_smc_chip *chip = nor->priv;

	aspeed_smc_start_user(nor);
	aspeed_smc_send_cmd_addr(nor, nor->program_opcode, to);
	aspeed_smc_to_fifo(chip->base, write_buf, len);
	*retlen += len;
	aspeed_smc_stop_user(nor);
}

static int aspeed_smc_erase(struct spi_nor *nor, loff_t offs)
{
	aspeed_smc_start_user(nor);
	aspeed_smc_send_cmd_addr(nor, nor->erase_opcode, offs);
	aspeed_smc_stop_user(nor);

	return 0;
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
	{ .compatible = "aspeed,fmc", .data = &fmc_info },
	{ .compatible = "aspeed,smc", .data = &smc_info },
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

static int aspeed_smc_probe(struct platform_device *dev)
{
	struct aspeed_smc_controller *controller;
	const struct of_device_id *match;
	const struct aspeed_smc_info *info;
	struct resource *r;
	void __iomem *regs;
	struct device_node *child;
	int err = 0;
	unsigned int n;

	/*
	 * This driver passes ioremap addresses to io port accessors.
	 * This works on arm if the IO_SPACE_LIMIT does not truncate
	 * the address.
	 */
	if (~(unsigned long)IO_SPACE_LIMIT)
		return -ENODEV;

	match = of_match_device(aspeed_smc_matches, &dev->dev);
	if (!match || !match->data)
		return -ENODEV;
	info = match->data;
	r = platform_get_resource(dev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(&dev->dev, r);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	controller = devm_kzalloc(&dev->dev, sizeof(*controller) +
		info->nce * sizeof(controller->chips[0]), GFP_KERNEL);
	if (!controller)
		return -ENOMEM;
	platform_set_drvdata(dev, controller);
	controller->regs = regs;
	controller->info = info;
	mutex_init(&controller->mutex);

	/* XXX turn off legacy mode if fmc ? */
	/* XXX handshake to enable access to SMC (bios) controller w/ host? */

	for_each_available_child_of_node(dev->dev.of_node, child) {
		struct mtd_part_parser_data ppdata;
		struct platform_device *cdev;
		struct aspeed_smc_chip *chip;
		u32 reg;

		if (!of_device_is_compatible(child, "jedec,spi-nor"))
			continue;	/* XXX consider nand, nor children */


		/*
		 * create a platform device from the of node.
		 * if the device already was created (eg from
		 * a prior bind/unbind cycle) use it
		 *
		 * XXX The child name will become the default mtd
		 * name in ioctl and /proc/mtd.  Should we default
		 * to node->name (without unit)?  The name must be
		 * unique among all platform devices.  (Name would
		 * replace NULL in create call below).
		 * ... Or we can just encourage the label attribute.
		 *
		 * The only reason to do the child here is to use it in
		 * dev_err below for duplicate chip id.  We could use
		 * the controller dev.
		 */
		cdev = of_platform_device_create_or_find(child, &dev->dev);
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
		chip = devm_kzalloc(&dev->dev, sizeof(*chip), GFP_KERNEL);
		if (!chip)
			continue;

		r = platform_get_resource(dev, IORESOURCE_MEM, n + 1);
		chip->base = devm_ioremap_resource(&dev->dev, r);

		if (!chip->base)
			continue;
		chip->controller = controller;
		chip->ctl = controller->regs + info->ctl0 + n * 4;

		/* dt said its spi.  xxx Set it in controller if has_type */
		chip->type = smc_type_spi;

		/*
		 * Always turn on write enable bit in config register to
		 * allow opcodes to be sent in user mode.
		 */
		mutex_lock(&controller->mutex);
		reg = readl(controller->regs + info->cfg);
		dev_dbg(&dev->dev, "flash config was %08x\n", reg);
		reg |= 1 << (info->we0 + n); /* WEn */
		writel(reg, controller->regs + info->cfg);
		mutex_unlock(&controller->mutex);

		/* XXX check resource within fmc CEx Segment Address Register */
		/* XXX -- see dt vs jedec id vs bootloader */
		/* XXX check / program clock phase/polarity,  only 0 or 3 */

		/*
		 * Read the existing control register to get basic values.
		 *
		 * XXX probably need more sanitation.
		 * XXX do we trust the bootloader or the device tree?
		 * spi-nor.c trusts jtag id over passed ids.
		 */
		reg = readl(chip->ctl);
		chip->ctl_val[smc_base] = reg & CONTROL_SPI_KEEP_MASK;

		if ((reg & CONTROL_SPI_COMMAND_MODE_MASK) ==
		    CONTROL_SPI_COMMAND_MODE_NORMAL)
			chip->ctl_val[smc_read] = reg;
		else
			chip->ctl_val[smc_read] = chip->ctl_val[smc_base] |
				CONTROL_SPI_COMMAND_MODE_NORMAL;

		chip->nor.dev = &cdev->dev;
		chip->nor.priv = chip;
		chip->nor.flash_node = child;
		chip->nor.mtd.name = of_get_property(child, "label", NULL);
		chip->nor.erase = aspeed_smc_erase;
		chip->nor.read = aspeed_smc_read_user;
		chip->nor.write = aspeed_smc_write_user;
		chip->nor.read_reg = aspeed_smc_read_reg;
		chip->nor.write_reg = aspeed_smc_write_reg;

		/*
		 * XXX use of property and controller info width to choose
		 * SPI_NOR_QUAD , SPI_NOR_DUAL
		 */
		err = spi_nor_scan(&chip->nor, NULL, SPI_NOR_NORMAL);
		if (err)
			continue;

		chip->ctl_val[smc_write] = chip->ctl_val[smc_base] |
			spi_control_fill_opcode(chip->nor.program_opcode) |
			CONTROL_SPI_COMMAND_MODE_WRITE;

		/* XXX intrepret nor flags into controller settings */
		/* XXX enable fast read here */
		/* XXX check if resource size big enough for chip */

		memset(&ppdata, 0, sizeof(ppdata));
		ppdata.of_node = cdev->dev.of_node;
		err = mtd_device_parse_register(&chip->nor.mtd, NULL, &ppdata, NULL, 0);
		if (err)
			continue;
		controller->chips[n] = chip;
	}

	/* did we register any children? */
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
		.name = KBUILD_MODNAME,
		.of_match_table = aspeed_smc_matches,
	}
};

module_platform_driver(aspeed_smc_driver);

MODULE_DESCRIPTION("ASPEED Static Memory Controller Driver");
MODULE_AUTHOR("Milton Miller");
MODULE_LICENSE("GPL v2");
