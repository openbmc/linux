#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/platform_device.h>
#include "core.h"

#define SCU 0x1e6e2000
#define SCU80 (SCU + 0x80)
#define SCU84 (SCU + 0x84)
#define SCU88 (SCU + 0x88)
#define SCU90 (SCU + 0x90)
#define SCU94 (SCU + 0x94)

struct ast_pinctrl_desc {
	uint32_t *c1_reg;
	uint32_t c1_mask;
	uint32_t *c2_reg;
	uint32_t c2_mask;
};

#define DEF_AST_PIN(name, a, b, c, d) \
  static struct ast_pinctrl_desc BALL_##name = {(u32 *) a, b, c, d}

DEF_AST_PIN(D6, SCU80, BIT_MASK(0), NULL, 0);
DEF_AST_PIN(B5, SCU80, BIT_MASK(1), NULL, 0);
DEF_AST_PIN(A4, SCU80, BIT_MASK(2), NULL, 0);
DEF_AST_PIN(E6, SCU80, BIT_MASK(3), NULL, 0);
DEF_AST_PIN(C5, SCU90, BIT_MASK(22), NULL, 0);
DEF_AST_PIN(B4, SCU90, BIT_MASK(22), NULL, 0);
DEF_AST_PIN(A3, SCU90, BIT_MASK(2), NULL, 0);
DEF_AST_PIN(D5, SCU90, BIT_MASK(2), NULL, 0);

#define AST_PIN_DESC(n, pinname)		\
	{ .number = n, .name = "pinname", .drv_data = &BALL_##pinname }

static const struct pinctrl_pin_desc ast2400_pins[] = {
	AST_PIN_DESC(0, D6),
	AST_PIN_DESC(1, B5),
	AST_PIN_DESC(2, A4),
	AST_PIN_DESC(3, E6),
	AST_PIN_DESC(4, C5),
	AST_PIN_DESC(5, B4),
	AST_PIN_DESC(6, A3),
	AST_PIN_DESC(7, D5),
};

static const unsigned int i2c9_pins[] = { 4, 5 };

struct ast_pin_group {
	const char *name;
	const unsigned int *pins;
	const unsigned num_pins;
};

static const struct ast_pin_group ast_pin_groups[] = {
	{
		.name = "i2c9",
		.pins = i2c9_pins,
		.num_pins = ARRAY_SIZE(i2c9_pins),
	},
};

static int ast_get_groups_count(struct pinctrl_dev *pctl)
{
	return ARRAY_SIZE(ast_pin_groups);
}

static const char *ast_get_group_name(struct pinctrl_dev *pctl,
					 unsigned selector)
{
	return ast_pin_groups[selector].name;
}

static int ast_get_group_pins(struct pinctrl_dev *pctl, unsigned selector,
				 const unsigned **pins, unsigned *num_pins)
{
	*pins = (unsigned *)ast_pin_groups[selector].pins;
	*num_pins = ast_pin_groups[selector].num_pins;
	return 0;
}

static struct pinctrl_ops ast_pinctrl_ops = {
	.get_groups_count = ast_get_groups_count,
	.get_group_name = ast_get_group_name,
	.get_group_pins = ast_get_group_pins,
};

static int ast_pinconf_get(struct pinctrl_dev *pctldev,
			   unsigned pin, unsigned long *config)
{
	dev_err(pctldev->dev, "Unimplemented function %s\n", __func__);

	return -ENOTSUPP;
}

static int ast_pinconf_set(struct pinctrl_dev *pctldev, unsigned pin,
			   unsigned long *configs, unsigned num_configs)
{
	struct ast_pinctrl_desc *desc;

	if (pin >= pctldev->desc->npins) {
		dev_err(pctldev->dev, "pin number %d is too damn high\n",
				pin);
		return -ERANGE;
	}

	desc = pctldev->desc->pins[pin].drv_data;

	dev_err(pctldev->dev, "Unimplemented function %s\n", __func__);

	return -ENOTSUPP;
}

static struct pinconf_ops ast_pconf_ops = {
	.pin_config_get = ast_pinconf_get,
	.pin_config_set = ast_pinconf_set,
};

struct ast_pinctrl {
	struct device		*dev;
	struct pinctrl_dev	*pctl;
};

struct pinctrl_desc ast_desc = {
	.name = "Aspeed",
	.pins = ast2400_pins,
	.npins = ARRAY_SIZE(ast2400_pins),
	.pctlops = &ast_pinctrl_ops,
	.confops = &ast_pconf_ops,
	.owner = THIS_MODULE,
};

struct aspeed_pinctrl {
	struct device *dev;
	void __iomem *base;
	struct pinctrl_dev *pctl_dev;
};

int ast_pinctrl_probe(struct platform_device *pdev)
{
	struct aspeed_pinctrl *pctl;
	struct resource *res;

	pctl = devm_kzalloc(&pdev->dev, sizeof(*pctl), GFP_KERNEL);
	if (!pctl)
		return -ENOMEM;

	platform_set_drvdata(pdev, pctl);
	pctl->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pctl->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pctl->base))
		return PTR_ERR(pctl->base);

	pctl->pctl_dev = pinctrl_register(&ast_desc, &pdev->dev, pctl);
	if (!pctl->dev) {
		pr_err("could not register apseed pin driver\n");
		return -EIO;
	}

	return 0;
}
