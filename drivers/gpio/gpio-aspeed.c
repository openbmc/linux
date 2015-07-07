/*
 * Copyright 2015 IBM Corp.
 *
 * Joel Stanley <joel@jms.id.au>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/gpio/driver.h>

struct aspeed_gpio {
	struct gpio_chip chip;
	spinlock_t lock;
	void __iomem *base;
	int irq;
	struct irq_chip irq_chip;
	struct irq_domain *irq_domain;
};

struct aspeed_gpio_bank {
	uint16_t	val_regs;
	uint16_t	irq_regs;
	const char	names[4];
};

static const struct aspeed_gpio_bank aspeed_gpio_banks[] = {
	{
		.val_regs = 0x0000,
		.irq_regs = 0x0008,
		.names = { 'A', 'B', 'C', 'D' },
	},
	{
		.val_regs = 0x0020,
		.irq_regs = 0x0028,
		.names = { 'E', 'F', 'G', 'H' },
	},
	{
		.val_regs = 0x0070,
		.irq_regs = 0x0098,
		.names = { 'I', 'J', 'K', 'L' },
	},
	{
		.val_regs = 0x0078,
		.irq_regs = 0x00e8,
		.names = { 'M', 'N', 'O', 'P' },
	},
	{
		.val_regs = 0x0080,
		.irq_regs = 0x0118,
		.names = { 'Q', 'R', 'S', 'T' },
	},
	{
		.val_regs = 0x0088,
		.irq_regs = 0x0148,
		.names = { 'U', 'V', 'W', 'X' },
	},
};

#define GPIO_BANK(x)	((x) >> 5)
#define GPIO_OFFSET(x)	((x) & 0x1f)
#define GPIO_BIT(x)	BIT(GPIO_OFFSET(x))

#define GPIO_DATA	0x00
#define GPIO_DIR	0x04

#define GPIO_IRQ_ENABLE	0x00
#define GPIO_IRQ_TYPE0	0x04
#define GPIO_IRQ_TYPE1	0x08
#define GPIO_IRQ_TYPE2	0x0c
#define GPIO_IRQ_STATUS	0x10

static inline struct aspeed_gpio *to_aspeed_gpio(struct gpio_chip *chip)
{
	return container_of(chip, struct aspeed_gpio, chip);
}

static const struct aspeed_gpio_bank *to_bank(unsigned int offset)
{
	unsigned int bank = GPIO_BANK(offset);
	WARN_ON(bank > ARRAY_SIZE(aspeed_gpio_banks));
	return &aspeed_gpio_banks[bank];
}

static void *bank_val_reg(struct aspeed_gpio *gpio,
		const struct aspeed_gpio_bank *bank,
		unsigned int reg)
{
	return gpio->base + bank->val_regs + reg;
}

static void *bank_irq_reg(struct aspeed_gpio *gpio,
		const struct aspeed_gpio_bank *bank,
		unsigned int reg)
{
	return gpio->base + bank->irq_regs + reg;
}

static int aspeed_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct aspeed_gpio *gpio = to_aspeed_gpio(gc);
	const struct aspeed_gpio_bank *bank = to_bank(offset);

	return !!(ioread32(bank_val_reg(gpio, bank, GPIO_DATA))
			& GPIO_BIT(offset));
}

static void aspeed_gpio_set(struct gpio_chip *gc, unsigned int offset,
			    int val)
{
	struct aspeed_gpio *gpio = to_aspeed_gpio(gc);
	const struct aspeed_gpio_bank *bank = to_bank(offset);
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&gpio->lock, flags);

	reg = ioread32(bank_val_reg(gpio, bank, GPIO_DATA));
	if (val)
		reg |= GPIO_BIT(offset);
	else
		reg &= ~GPIO_BIT(offset);

	iowrite32(reg, bank_val_reg(gpio, bank, GPIO_DATA));

	spin_unlock_irqrestore(&gpio->lock, flags);
}

static int aspeed_gpio_dir_in(struct gpio_chip *gc, unsigned int offset)
{
	struct aspeed_gpio *gpio = to_aspeed_gpio(gc);
	const struct aspeed_gpio_bank *bank = to_bank(offset);
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&gpio->lock, flags);

	reg = ioread32(bank_val_reg(gpio, bank, GPIO_DIR));
	iowrite32(reg & ~GPIO_BIT(offset), bank_val_reg(gpio, bank, GPIO_DIR));

	spin_unlock_irqrestore(&gpio->lock, flags);

	return 0;
}

static int aspeed_gpio_dir_out(struct gpio_chip *gc,
			       unsigned int offset, int val)
{
	struct aspeed_gpio *gpio = to_aspeed_gpio(gc);
	const struct aspeed_gpio_bank *bank = to_bank(offset);
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&gpio->lock, flags);

	reg = ioread32(bank_val_reg(gpio, bank, GPIO_DIR));
	iowrite32(reg | GPIO_BIT(offset), bank_val_reg(gpio, bank, GPIO_DIR));

	spin_unlock_irqrestore(&gpio->lock, flags);

	return 0;
}

static inline int irqd_to_aspeed_gpio_data(struct irq_data *d,
		struct aspeed_gpio **gpio,
		const struct aspeed_gpio_bank **bank,
		u32 *bit)
{
	int offset;

	offset = irqd_to_hwirq(d);

	*gpio = irq_data_get_irq_chip_data(d);
	*bank = to_bank(offset);
	*bit = GPIO_BIT(offset);

	return 0;
}

static void aspeed_gpio_irq_ack(struct irq_data *d)
{
	const struct aspeed_gpio_bank *bank;
	struct aspeed_gpio *gpio;
	unsigned long flags;
	void *status_addr;
	u32 bit;
	int rc;

	rc = irqd_to_aspeed_gpio_data(d, &gpio, &bank, &bit);
	if (rc)
		return;

	status_addr = bank_irq_reg(gpio, bank, GPIO_IRQ_STATUS);

	spin_lock_irqsave(&gpio->lock, flags);
	iowrite32(bit, status_addr);
	spin_unlock_irqrestore(&gpio->lock, flags);
}

static void __aspeed_gpio_irq_set_mask(struct irq_data *d, bool set)
{
	const struct aspeed_gpio_bank *bank;
	struct aspeed_gpio *gpio;
	unsigned long flags;
	u32 reg, bit;
	void *addr;
	int rc;

	rc = irqd_to_aspeed_gpio_data(d, &gpio, &bank, &bit);
	if (rc)
		return;

	addr = bank_irq_reg(gpio, bank, GPIO_IRQ_ENABLE);

	spin_lock_irqsave(&gpio->lock, flags);

	reg = ioread32(addr);
	if (set)
		reg |= bit;
	else
		reg &= bit;
	iowrite32(reg, addr);

	spin_unlock_irqrestore(&gpio->lock, flags);
}

static void aspeed_gpio_irq_mask(struct irq_data *d)
{
	__aspeed_gpio_irq_set_mask(d, false);
}

static void aspeed_gpio_irq_unmask(struct irq_data *d)
{
	__aspeed_gpio_irq_set_mask(d, true);
}

static int aspeed_gpio_set_type(struct irq_data *d, unsigned int type)
{
	u32 type0, type1, type2, bit, reg;
	const struct aspeed_gpio_bank *bank;
	irq_flow_handler_t handler;
	struct aspeed_gpio *gpio;
	unsigned long flags;
	void *addr;
	int rc;

	rc = irqd_to_aspeed_gpio_data(d, &gpio, &bank, &bit);
	if (rc)
		return -EINVAL;

	type0 = type1 = type2 = 0;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_BOTH:
		type2 |= bit;
	case IRQ_TYPE_EDGE_RISING:
		type0 |= bit;
	case IRQ_TYPE_EDGE_FALLING:
		handler = handle_edge_irq;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		type0 |= bit;
	case IRQ_TYPE_LEVEL_LOW:
		type1 |= bit;
		handler = handle_level_irq;
		break;
	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&gpio->lock, flags);

	addr = bank_irq_reg(gpio, bank, GPIO_IRQ_TYPE0);
	reg = ioread32(addr);
	reg = (reg & ~bit) | type0;
	iowrite32(reg, addr);

	addr = bank_irq_reg(gpio, bank, GPIO_IRQ_TYPE1);
	reg = ioread32(addr);
	reg = (reg & ~bit) | type1;
	iowrite32(reg, addr);

	addr = bank_irq_reg(gpio, bank, GPIO_IRQ_TYPE2);
	reg = ioread32(addr);
	reg = (reg & ~bit) | type2;
	iowrite32(reg, addr);

	spin_unlock_irqrestore(&gpio->lock, flags);

	irq_set_handler_locked(d, handler);

	return 0;
}

static void aspeed_gpio_irq_handler(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct aspeed_gpio *gpio = irq_desc_get_chip_data(desc);
	unsigned int i, p, girq;
	unsigned long reg;

	chained_irq_enter(chip, desc);

	for (i = 0; i < ARRAY_SIZE(aspeed_gpio_banks); i++) {
		const struct aspeed_gpio_bank *bank = &aspeed_gpio_banks[i];

		reg = ioread32(bank_irq_reg(gpio, bank, GPIO_IRQ_STATUS));

		for_each_set_bit(p, &reg, 32) {
			girq = irq_find_mapping(gpio->irq_domain, i * 32 + p);
			generic_handle_irq(girq);
		}

	}

	chained_irq_exit(chip, desc);
}

static struct irq_chip aspeed_gpio_irqchip = {
	.name		= "aspeed-gpio",
	.irq_ack	= aspeed_gpio_irq_ack,
	.irq_mask	= aspeed_gpio_irq_mask,
	.irq_unmask	= aspeed_gpio_irq_unmask,
	.irq_set_type	= aspeed_gpio_set_type,
};

static int aspeed_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct aspeed_gpio *gpio = to_aspeed_gpio(chip);
	return irq_find_mapping(gpio->irq_domain, offset);
}

static void aspeed_gpio_setup_irqs(struct aspeed_gpio *gpio,
		struct platform_device *pdev)
{
	int i, irq;

	/* request our upstream IRQ */
	gpio->irq = platform_get_irq(pdev, 0);
	if (gpio->irq < 0)
		return;

	/* establish our irq domain to provide IRQs for each extended bank */
	gpio->irq_domain = irq_domain_add_linear(pdev->dev.of_node,
			gpio->chip.ngpio, &irq_domain_simple_ops, NULL);
	if (!gpio->irq_domain)
		return;

	for (i = 0; i < gpio->chip.ngpio; i++) {
		irq = irq_create_mapping(gpio->irq_domain, i);
		irq_set_chip_data(irq, gpio);
		irq_set_chip_and_handler(irq, &aspeed_gpio_irqchip,
				handle_simple_irq);
		irq_set_probe(irq);
	}

	irq_set_chained_handler_and_data(gpio->irq,
			aspeed_gpio_irq_handler, gpio);
}


static int __init aspeed_gpio_probe(struct platform_device *pdev)
{
	struct aspeed_gpio *gpio;
	struct resource *res;
	int rc;

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	gpio->base = devm_ioremap_resource(&pdev->dev, res);
	if (!gpio->base)
		return -ENOMEM;

	spin_lock_init(&gpio->lock);

	gpio->chip.ngpio = ARRAY_SIZE(aspeed_gpio_banks) * 32;

	gpio->chip.dev = &pdev->dev;
	gpio->chip.direction_input = aspeed_gpio_dir_in;
	gpio->chip.direction_output = aspeed_gpio_dir_out;
	gpio->chip.get = aspeed_gpio_get;
	gpio->chip.set = aspeed_gpio_set;
	gpio->chip.to_irq = aspeed_gpio_to_irq;
	gpio->chip.label = dev_name(&pdev->dev);
	gpio->chip.base = -1;

	platform_set_drvdata(pdev, gpio);

	rc = gpiochip_add(&gpio->chip);
	if (rc < 0)
		return rc;

	aspeed_gpio_setup_irqs(gpio, pdev);

	return 0;
}

static int aspeed_gpio_remove(struct platform_device *pdev)
{
	struct aspeed_gpio *gpio = platform_get_drvdata(pdev);

	gpiochip_remove(&gpio->chip);
	return 0;
}

static const struct of_device_id aspeed_gpio_of_table[] = {
	{ .compatible = "aspeed,ast2400-gpio" },
	{}
};
MODULE_DEVICE_TABLE(of, aspeed_gpio_of_table);

static struct platform_driver aspeed_gpio_driver = {
	.remove = aspeed_gpio_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = aspeed_gpio_of_table,
	},
};

module_platform_driver_probe(aspeed_gpio_driver, aspeed_gpio_probe);

MODULE_DESCRIPTION("Aspeed AST2400 GPIO Driver");
