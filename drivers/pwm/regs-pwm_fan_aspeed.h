/* arch/arm/plat-aspeed/include/mach/regs-pwm-fan.h
 *
 * Copyright (c) 2012 ASPEED Technology Inc. <ryan_chen@aspeedtech.com>
 *		http://www.aspeedtech.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ASPEED PWM & Fan Tacho Controller
*/

#ifndef __ASM_ARCH_REGS_PWM_FAN_H
#define __ASM_ARCH_REGS_PWM_FAN_H __FILE__

/*AST PWM & FAN Register Definition */
#define AST_PTCR_CTRL			0x00
#define AST_PTCR_CLK_CTRL		0x04
#define AST_PTCR_DUTY0_CTRL		0x08
#define AST_PTCR_DUTY1_CTRL		0x0c
#define AST_PTCR_TYPEM_CTRL0	0x10
#define AST_PTCR_TYPEM_CTRL1	0x14
#define AST_PTCR_TYPEN_CTRL0	0x18
#define AST_PTCR_TYPEN_CTRL1	0x1c
#define AST_PTCR_TACH_SOURCE	0x20
// no 0x24
#define AST_PTCR_TRIGGER		0x28
#define AST_PTCR_RESULT			0x2c
#define AST_PTCR_INTR_CTRL		0x30
#define AST_PTCR_INTR_STS		0x34
#define AST_PTCR_TYPEM_LIMIT	0x38
#define AST_PTCR_TYPEN_LIMIT	0x3C
#define AST_PTCR_CTRL_EXT		0x40
#define AST_PTCR_CLK_EXT_CTRL	0x44
#define AST_PTCR_DUTY2_CTRL		0x48
#define AST_PTCR_DUTY3_CTRL		0x4c
#define AST_PTCR_TYPEO_CTRL0	0x50
#define AST_PTCR_TYPEO_CTRL1	0x54
#define AST_PTCR_TACH_SOURCE_EXT 0x60
#define AST_PTCR_TYPEO_LIMIT	0x78

//COMMON Definition 
#define FALL_EDGE					(0)
#define RISE_EDGE					(0x1)
#define BOTH_EDGE					(0x2)

#ifdef CONFIG_ARCH_AST1010
#define PWM_TYPE_NUM		2
#define PWM_TYPE_M			0x0
#define PWM_TYPE_N			0x1
#define PWM_TYPE_MASK		0x1
#else
#define PWM_TYPE_NUM		3
#define PWM_TYPE_M			0x0
#define PWM_TYPE_N			0x1
#define PWM_TYPE_O			0x2
#define PWM_TYPE_MASK		0x3

#endif

#define TACHO_NUM	16
#define PWM_CH_NUM	8
#define PWMA	0x0
#define PWMB	0x1
#define PWMC	0x2
#define PWMD	0x3
#define PWME	0x4
#define PWMF	0x5
#define PWMG	0x6
#define PWMH	0x7


// AST_PTCR_CTRL:0x00 - PWM-FAN General Control Register 
#define AST_PTCR_CTRL_SET_PWMD_TYPE(x)		((x&0x1)<<15 | (x&0x2) <<6)
#define AST_PTCR_CTRL_GET_PWMD_TYPE(x)		(((x&(0x1<<7))>>6) | ((x&(0x1<<15))>>15))
#define AST_PTCR_CTRL_SET_PWMD_TYPE_MASK	((0x1<<7) | (0x1<<15))

#define AST_PTCR_CTRL_SET_PWMC_TYPE(x)		((x&0x1)<<14 | (x&0x2) <<5)
#define AST_PTCR_CTRL_GET_PWMC_TYPE(x)		(((x&(0x1<<6))>>5) | ((x&(0x1<<14))>>14))
#define AST_PTCR_CTRL_SET_PWMC_TYPE_MASK	((0x1<<6) | (0x1<<14))

#define AST_PTCR_CTRL_SET_PWMB_TYPE(x)		((x&0x1)<<13 | (x&0x2) <<4)
#define AST_PTCR_CTRL_GET_PWMB_TYPE(x)		(((x&(0x1<<5))>>4) | ((x&(0x1<<13))>>13))
#define AST_PTCR_CTRL_SET_PWMB_TYPE_MASK	((0x1<<5) | (0x1<<13))


#define AST_PTCR_CTRL_SET_PWMA_TYPE(x)		((x&0x1)<<12 | (x&0x2) <<3)
#define AST_PTCR_CTRL_GET_PWMA_TYPE(x)		(((x&(0x1<<4))>>3) | ((x&(0x1<<12))>>12))
#define AST_PTCR_CTRL_SET_PWMA_TYPE_MASK	((0x1<<4) | (0x1<<12))

#define	AST_PTCR_CTRL_FAN_NUM_EN(x)		(0x1 << (16+x))

#define	AST_PTCR_CTRL_PWMD			(11)
#define	AST_PTCR_CTRL_PWMD_EN		(0x1 << 11)
#define	AST_PTCR_CTRL_PWMC			(10)
#define	AST_PTCR_CTRL_PWMC_EN		(0x1 << 10)
#define	AST_PTCR_CTRL_PWMB			(9)
#define	AST_PTCR_CTRL_PWMB_EN		(0x1 << 9)
#define	AST_PTCR_CTRL_PWMA			(8)
#define	AST_PTCR_CTRL_PWMA_EN		(0x1 << 8)

#define	AST_PTCR_CTRL_CLK_MCLK		0x2		//0:24Mhz, 1:MCLK
#define	AST_PTCR_CTRL_CLK_EN		0x1

// AST_PTCR_CLK_CTRL:0x04 - PWM-FAN Clock Control Register
//TYPE N
#define AST_PTCR_CLK_CTRL_TYPEN_UNIT				(24)
#define AST_PTCR_CLK_CTRL_TYPEN_UNIT_MASK			(0xff<<24)
#define AST_PTCR_CLK_CTRL_TYPEN_H					(20)
#define AST_PTCR_CLK_CTRL_TYPEN_H_MASK				(0xf<<20)
#define AST_PTCR_CLK_CTRL_TYPEN_L					(16)
#define AST_PTCR_CLK_CTRL_TYPEN_L_MASK				(0xf<<16)
//TYPE M
#define AST_PTCR_CLK_CTRL_TYPEM_UNIT				(8)
#define AST_PTCR_CLK_CTRL_TYPEM_UNIT_MASK			(0xff<<8)
#define AST_PTCR_CLK_CTRL_TYPEM_H					(4)
#define AST_PTCR_CLK_CTRL_TYPEM_H_MASK				(0xf<<4)
#define AST_PTCR_CLK_CTRL_TYPEM_L					(0)
#define AST_PTCR_CLK_CTRL_TYPEM_L_MASK				(0xf)


// AST_PTCR_DUTY_CTRL0:0x08 - PWM-FAN duty control 0 register
#define DUTY_CTRL0_PWMB_FALL_POINT					(24)
#define DUTY_CTRL0_PWMB_FALL_POINT_MASK				(0xff<<24)
#define DUTY_CTRL0_PWMB_RISE_POINT					(16)
#define DUTY_CTRL0_PWMB_RISE_POINT_MASK				(0xff<<16)
#define DUTY_CTRL0_PWMA_FALL_POINT					(8)
#define DUTY_CTRL0_PWMA_FALL_POINT_MASK				(0xff<<8)
#define DUTY_CTRL0_PWMA_RISE_POINT					(0)
#define DUTY_CTRL0_PWMA_RISE_POINT_MASK				(0xff)


// AST_PTCR_DUTY_CTRL1 : 0x0c - PWM-FAN duty control 1 register
#define DUTY_CTRL1_PWMD_FALL_POINT					(24)
#define DUTY_CTRL1_PWMD_FALL_POINT_MASK				(0xff<<24)
#define DUTY_CTRL1_PWMD_RISE_POINT					(16)
#define DUTY_CTRL1_PWMD_RISE_POINT_MASK				(0xff<<16)
#define DUTY_CTRL1_PWMC_FALL_POINT					(8)
#define DUTY_CTRL1_PWMC_FALL_POINT_MASK				(0xff<<8)
#define DUTY_CTRL1_PWMC_RISE_POINT					(0)
#define DUTY_CTRL1_PWMC_RISE_POINT_MASK				(0xff)


// AST_PTCR_TYPEM_CTRL0 : 0x10/0x18/0x50 - Type M/N/O Ctrl 0 Register
#define TYPE_CTRL0_FAN_PERIOD						(16)
#define TYPE_CTRL0_FAN_PERIOD_MASK					(0xffff<<16)
//Type O not have this
#define TYPE_CTRL0_FLAT_EN							(0x1<<7)


// 0 : FALL_EDGE,	0x1 : RISE_EDGE , 0x2 :BOTH_EDGE	
#define TYPE_CTRL0_FAN_MODE							(4)
#define TYPE_CTRL0_FAN_MODE_MASK					(0x3<<4)



#define TYPE_CTRL0_CLK_DIVISION						(1)
#define TYPE_CTRL0_CLK_DIVISION_MASK				(0x7<<1)

#define TYPE_CTRL0_FAN_TYPE_EN							(1)


// AST_PTCR_TYPEM_CTRL1 : 0x14/0x1c/0x54 - Type M/N/O Ctrl 1 Register
#define TYPE_CTRL1_FALL_POINT						(16)
#define TYPE_CTRL1_FALL_POINT_MASK					(0xff<<16)
#define TYPE_CTRL1_RISE_POINT						(0)
#define TYPE_CTRL1_RISE_POINT_MASK					(0xff)


// AST_PTCR_TACH_SOURCE : 0x20/0x60 - Tach Source Register
//bit [0,1] at 0x20, bit [2] at 0x60
#define TACH_PWM_SOURCE_BIT01(x)					(x*2)
#define TACH_PWM_SOURCE_BIT2(x)						(x*2)

#define TACH_PWM_SOURCE_MASK_BIT01(x)						(0x3<<(x*2))
#define TACH_PWM_SOURCE_MASK_BIT2(x)						(0x1<<(x*2))

// AST_PTCR_TRIGGER : 0x28 - Trigger Register
#define TRIGGER_READ_FAN_NUM(x)						(0x1<<x)

// AST_PTCR_RESULT : 0x2c - Result Register
#define RESULT_STATUS								(31)

#define RESULT_VALUE_MASK							(0xfffff)

// AST_PTCR_INTR_CTRL : 0x30 - Interrupt Ctrl Register
#define INTR_CTRL_EN_NUM(x)							(0x1<<x)

// AST_PTCR_INTR_STS : 0x34 - Interrupt Status Register		
#define INTR_CTRL_NUM(x)							(0x1<<x)

//AST_PTCR_TYPEM_LIMIT, AST_PTCR_TYPEN_LIMIT,AST_PTCR_TYPEO_LIMIT  : 0x38/0x3C/0x78 - Type M / N / O Limit Register		
#define FAN_LIMIT_MASK								(0xfffff)

// AST_PTCR_CTRL_EXT : 0x40 - General Ctrl Extension #1
#define AST_PTCR_CTRL_SET_PWMH_TYPE(x)		((x&0x1)<<15 | (x&0x2) <<6)
#define AST_PTCR_CTRL_GET_PWMH_TYPE(x)		(((x&(0x1<<7))>>6) | ((x&(0x1<<15))>>15))
#define AST_PTCR_CTRL_SET_PWMH_TYPE_MASK	((0x1<<7) | (0x1<<15))

#define AST_PTCR_CTRL_SET_PWMG_TYPE(x)		((x&0x1)<<14 | (x&0x2) <<5)
#define AST_PTCR_CTRL_GET_PWMG_TYPE(x)		(((x&(0x1<<6))>>5) | ((x&(0x1<<14))>>14))
#define AST_PTCR_CTRL_SET_PWMG_TYPE_MASK	((0x1<<6) | (0x1<<14))

#define AST_PTCR_CTRL_SET_PWMF_TYPE(x)		((x&0x1)<<13 | (x&0x2) <<4)
#define AST_PTCR_CTRL_GET_PWMF_TYPE(x)		(((x&(0x1<<5))>>4) | ((x&(0x1<<13))>>13))
#define AST_PTCR_CTRL_SET_PWMF_TYPE_MASK	((0x1<<5) | (0x1<<13))

#define AST_PTCR_CTRL_SET_PWME_TYPE(x)		((x&0x1)<<12 | (x&0x2) <<3)
#define AST_PTCR_CTRL_GET_PWME_TYPE(x)		(((x&(0x1<<4))>>3) | ((x&(0x1<<12))>>12))
#define AST_PTCR_CTRL_SET_PWME_TYPE_MASK	((0x1<<4) | (0x1<<12))

#define	AST_PTCR_CTRL_PWMH			(11)
#define	AST_PTCR_CTRL_PWMH_EN		(0x1 << 11)
#define	AST_PTCR_CTRL_PWMG			(10)
#define	AST_PTCR_CTRL_PWMG_EN		(0x1 << 10)
#define	AST_PTCR_CTRL_PWMF			(9)
#define	AST_PTCR_CTRL_PWMF_EN		(0x1 << 9)
#define	AST_PTCR_CTRL_PWME			(8)
#define	AST_PTCR_CTRL_PWME_EN		(0x1 << 8)

// AST_PTCR_CLK_EXT_CTRL : 0x44 - Clock Control Extension #1 
//TYPE O
#define AST_PTCR_CLK_CTRL_TYPEO_UNIT				(8)
#define AST_PTCR_CLK_CTRL_TYPEO_UNIT_MASK			(0xff<<8)
#define AST_PTCR_CLK_CTRL_TYPEO_H					(4)
#define AST_PTCR_CLK_CTRL_TYPEO_H_MASK				(0xf<<4)
#define AST_PTCR_CLK_CTRL_TYPEO_L					(0)
#define AST_PTCR_CLK_CTRL_TYPEO_L_MASK				(0xf)

// AST_PTCR_DUTY2_CTRL : 0x48 - Duty Control 2 Register
#define DUTY_CTRL2_PWMF_FALL_POINT					(24)
#define DUTY_CTRL2_PWMF_FALL_POINT_MASK				(0xff<<24)
#define DUTY_CTRL2_PWMF_RISE_POINT					(16)
#define DUTY_CTRL2_PWMF_RISE_POINT_MASK				(0xff<<16)
#define DUTY_CTRL2_PWME_FALL_POINT					(8)
#define DUTY_CTRL2_PWME_FALL_POINT_MASK				(0xff<<8)
#define DUTY_CTRL2_PWME_RISE_POINT					(0)
#define DUTY_CTRL2_PWME_RISE_POINT_MASK				(0xff)

// AST_PTCR_DUTY3_CTRL : 0x4c - Duty Control 3 Register
#define DUTY_CTRL3_PWMH_FALL_POINT					(24)
#define DUTY_CTRL3_PWMH_FALL_POINT_MASK				(0xff<<24)
#define DUTY_CTRL3_PWMH_RISE_POINT					(16)
#define DUTY_CTRL3_PWMH_RISE_POINT_MASK				(0xff<<16)
#define DUTY_CTRL3_PWMG_FALL_POINT					(8)
#define DUTY_CTRL3_PWMG_FALL_POINT_MASK				(0xff<<8)
#define DUTY_CTRL3_PWMG_RISE_POINT					(0)
#define DUTY_CTRL3_PWMG_RISE_POINT_MASK				(0xff)

#endif /* __ASM_ARCH_REGS_PWM_FAN_H */
