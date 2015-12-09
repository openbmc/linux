/* ASPEED Static Memory Controller driver
 * Copyright 2015
 * Milton Miller
 *
 */
#define DEBUG

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

enum smc_controller_type {
	smc_type_leg,		/* legacy register mode and map */
	smc_type_fmc,		/* new FMC 5 CEs multiple flash types */
	smc_type_smc,		/* SPI memory controller for BIOS / host */
};

enum smc_flash_type {
	smc_type_nor = 0,	/* controller connected to nor flash */
	smc_type_nand = 1,	/* controller connected to nand flash */
	smc_type_spi = 2,	/* controller connected to spi flash */
};

struct aspeed_smc_info {
	unsigned long nce : 3;		/* number of chip enables */
	unsigned long hasdma : 1;	/* has dma engine */
	unsigned long maxwidth : 3;	/* max width of spi bus */
	unsigned long we0 : 5;		/* we shift for ce 0 in cfg reg */
	unsigned long hastype : 1;	/* type shift for ce 0 in cfg reg */
	u8 ctl0;			/* offset in regs of ctl for ce 0 */
	u8 cfg;				/* offset in regs of cfg */
	u8 time;			/* offset in regs of timing */
};

#define E(_ce, _dma, _w, _ctl, _cfg, _we0, _hastype, _time, _misc) \
	.nce = _ce, .ctl0 = _ctl, .cfg = 0, .we0 = _we0, .hastype = _hastype, \
	.hasdma = _dma, .maxwidth = _w, .time = _time

struct aspeed_smc_info aspeed_table[] = {
	[smc_type_fmc] = { E(5, 1, 4, 0x10, 0x00, 16, 1, 0x54, 0x50) },
	[smc_type_smc] = { E(1, 0, 2, 0x04, 0x00, 0,  0, 0x14, 0x10) },
};

#undef E

enum smc_ctl_reg {
	smc_base,		/* base value without mode for other commands */
	smc_read,		/* command reg for (maybe fast) reads */
	smc_write,		/* command reg for writes with timings */
	smc_num_ctl_reg		/* last value to get count of commands */
};

struct aspeed_smc_controller;

struct aspeed_smc_chip {
	struct aspeed_smc_controller *controller;
	__le32 __iomem *ctl;			/* control register */
	void __iomem *base;			/* base of chip window */
	__le32 ctl_val[smc_num_ctl_reg];	/* controls with timing */
	struct mtd_info mtd;
	struct spi_nor nor;
};

struct aspeed_smc_controller {
	struct mutex mutex;			/* controller access mutex */
	const struct aspeed_smc_info *info;	/* type info of controller */
	void __iomem *regs;			/* controller registers */
	struct aspeed_smc_chip *chips[0];	/* attached chips */
};

#define CONTROL_SPI_AAF_MODE BIT(31)
#define CONTROL_SPI_IO_MODE_MASK GENMAASK(30, 28)
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

#if 0 /* ATTR */
static u32 spi_control_to_freq_div(u32 control)
{
	u32 sel;

	sel = control & CONTROL_SPI_CLOCK_FREQ_SEL_MASK;
	sel >>= CONTROL_SPI_CLOCK_FREQ_SEL_SHIFT;

	/* 16, 14, 12, 10, 8, 6, 4, 2, 15, 13, 11, 9, 7, 5, 3, 1 */
	return 16 - (((sel & 7) << 1) + ((sel & 8) >> 1));
}

static u32 spi_control_to_dummy_bytes(u32 control)
{

	return ((control & CONTROL_SPI_IO_DUMMY_CYCLES_LO) >>
			CONTROL_SPI_IO_DUMMY_CYCLES_LO_SHIFT) |
		((control & CONTROL_SPI_IO_DUMMY_CYCLES_HI_SHIFT) >>
			CONTROL_SPI_IO_DUMMY_CYCLES_HI_SHIFT);
}

static size_t show_ctl_div(u32 control, char *buf)
{
	return sprintf(buf, "%u\n", spi_control_to_freq_div(control));
}

static size_t show_ctl_dummy_bytes(u32 control, char *buf)
{
	return sprintf(buf, "%u\n", spi_control_to_dummy_bytes(control));
}
#endif /* ATTR */

static void aspeed_smc_start_user(struct spi_nor *nor)
{
	struct aspeed_smc_chip *chip = nor->priv;
	u32 ctl = chip->ctl_val[smc_base];

	if (chip->controller)
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

	if (chip->controller)
		mutex_unlock(&chip->controller->mutex);
}

#if 0 /* memcpy */
static void aspeed_smc_start_write(struct spi_nor *nor)
{
	struct aspeed_smc_chip *chip = nor->priv;
	u32 ctl = chip->ctl_val[smc_write];

	if (chip->controller)
		mutex_lock(&chip->controller->mutex);

	writel(ctl | CONTROL_SPI_CE_STOP_ACTIVE_CONTROL,  chip->ctl);

	writel(ctl, chip->ctl);
}

static void aspeed_smc_stop_write(struct spi_nor *nor)
{
	aspeed_smc_stop_user(nor);
}
#endif

static int aspeed_smc_read_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	struct aspeed_smc_chip *chip = nor->priv;

	aspeed_smc_start_user(nor);
	writeb(opcode, chip->base);
	_memcpy_fromio(buf, chip->base, len);
	aspeed_smc_stop_user(nor);

	return 0;
}

static int aspeed_smc_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf,
				int len, int write_enable)
{
	struct aspeed_smc_chip *chip = nor->priv;

	aspeed_smc_start_user(nor);
	writeb(opcode, chip->base);
	_memcpy_toio(chip->base, buf, len);
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
		memcpy_toio(chip->base, &temp, 4);
		break;
	case 4:
		temp = cpu_to_be32(addr);
		writeb(cmd, chip->base);
		memcpy_toio(chip->base, &temp, 4);
		break;
	}
}

#if 0 /* memcpy */
static int aspeed_smc_read_memcpy(struct spi_nor *nor, loff_t from, size_t len,
				  size_t *retlen, u_char *read_buf)
{
	struct aspeed_smc_chip *chip = nor->priv;

	if (chip->controller)
		mutex_lock(&chip->controller->mutex);

	memcpy_fromio(read_buf, chip->base, len);
	*retlen += len;

	if (chip->controller)
		mutex_unlock(&chip->controller->mutex);

	return 0;
}

static void aspeed_smc_write_memcpy(struct spi_nor *nor, loff_t to, size_t len,
				    size_t *retlen, const u_char *write_buf)
{
	struct aspeed_smc_chip *chip = nor->priv;

	aspeed_smc_start_write(nor);
	memcpy_toio(chip->base, write_buf, len);
	*retlen += len;
	aspeed_smc_stop_write(nor);
}
#endif

static int aspeed_smc_read_user(struct spi_nor *nor, loff_t from, size_t len,
				size_t *retlen, u_char *read_buf)
{
	struct aspeed_smc_chip *chip = nor->priv;

	aspeed_smc_start_user(nor);
	aspeed_smc_send_cmd_addr(nor, nor->read_opcode, from);
	memcpy_fromio(read_buf, chip->base, len);
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
	memcpy_toio(chip->base, write_buf, len);
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
			mtd_device_unregister(&chip->mtd);
	}

	return 0;
}

const struct of_device_id aspeed_smc_matches[] = {
	{ .compatible = "aspeed,fmc", .data = &aspeed_table[smc_type_fmc] },
	{ .compatible = "aspeed,smc", .data = &aspeed_table[smc_type_smc] },
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

	match = of_match_device(aspeed_smc_matches, &dev->dev);
	if (!match)
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
		/* check type is ast_smc_spi or set it? */
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
#if 0 /* should we inherit fast read (fread) from boot ?  or id tables?*/
		else if ((reg & CONTROL_SPI_COMMAND_MODE_MASK) ==
			 CONTROL_SPI_COMMAND_MODE_FREAD)
			chip->ctl_val[smc_read] = reg;
#endif
		else
			chip->ctl_val[smc_read] = chip->ctl_val[smc_base] |
				CONTROL_SPI_COMMAND_MODE_NORMAL;

		chip->nor.dev = &cdev->dev;
		chip->nor.priv = chip;
		chip->nor.mtd = &chip->mtd;
		chip->mtd.priv = &chip->nor; /* should be in spi_nor_scan()!! */
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
		err = mtd_device_parse_register(&chip->mtd, NULL, &ppdata, NULL, 0);
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

/* XXX should this be a builtin driver ? */
module_platform_driver(aspeed_smc_driver);

MODULE_DESCRIPTION("ASPEED Static Memory Controller Driver");
MODULE_AUTHOR("Milton Miller");
MODULE_LICENSE("GPL v2");
