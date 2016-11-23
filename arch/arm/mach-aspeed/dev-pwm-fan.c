/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-pwm-fan.c
* Author        : Ryan chen
* Description   : ASPEED PWM-FAN Device
*
* Copyright (C) ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by the Free Software Foundation;
* either version 2 of the License, or (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

* History      :
*    1. 2012/08/06 ryan chen create this file
*
********************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>

//#include <mach/irqs.h>
#define IRQ_TACHO                                         28

//#include <mach/platform.h>
#define AST_PWM_BASE                       0x1E786000    /* PWM */
#define AST_SCU_BASE                       0x1E6E2000    /* SCU */
//#include <plat/devs.h>
extern void __init ast_add_device_pwm_fan(void);
//#include <plat/ast-scu.h>
extern void ast_scu_init_pwm_tacho(void);
extern u32 ast_get_h_pll_clk(void);
//#include <mach/ast_pwm_techo.h>
struct ast_pwm_driver_data { //from mach/ast_pwm_techo.h
	u32 (*get_pwm_clock)(void);
};

/* --------------------------------------------------------------------
 *  PWM-FAN
 * -------------------------------------------------------------------- */

//#if defined(CONFIG_PWM_AST_ASPEED)
#if defined(CONFIG_ARCH_ASPEED)

static struct resource ast_pwm_fan_resources[] = {
	[0] = {
		.start = AST_PWM_BASE,
		.end = AST_PWM_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_TACHO,
		.end = IRQ_TACHO,
		.flags = IORESOURCE_IRQ,
	},
};

static struct ast_pwm_driver_data ast_pwm_data = {
	.get_pwm_clock = ast_get_h_pll_clk,
};

struct platform_device ast_pwm_fan_device = {
	.name = "ast_pwm_tacho",
	.id = 0,
	.dev = {
		.platform_data = &ast_pwm_data,
	},	
	.resource = ast_pwm_fan_resources,
	.num_resources = ARRAY_SIZE(ast_pwm_fan_resources),
};


extern u32
ast_get_h_pll_clk(void)
{
	return 384000000;
}

EXPORT_SYMBOL(ast_get_h_pll_clk);

void __init ast_add_device_pwm_fan(void)
{
	platform_device_register(&ast_pwm_fan_device);
}



#if 0
static u32 ast_scu_base = IO_ADDRESS(AST_SCU_BASE);

static inline void
ast_scu_write(u32 val, u32 reg)
{
	writel(val, ast_scu_base + reg);
}
#endif


extern void
ast_scu_init_pwm_tacho(void)
{
	#if 0
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_PWM, AST_SCU_RESET);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_PWM, AST_SCU_RESET);
	#endif
}

EXPORT_SYMBOL(ast_scu_init_pwm_tacho);

#else
void __init ast_add_device_pwm_fan(void) {
}
#endif
