/*
 * Copyright 2015 IBM Corp.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 */

#ifndef AST2400_H
#define AST2400_H

/* Periperhal base addresses */
#define AST_BASE_LEGACY_SRAM	0x10000000 /* Legacy BMC Static Memory */
#define AST_BASE_LEGACY_SMC	0x16000000 /* Legacy BMC Static Memory Controller (SMC) */
#define AST_BASE_APBC		0x1E600000 /* AHB Bus Controller (AHBC) */
#define AST_BASE_FMC		0x1E620000 /* New BMC Static Memory Controller (FMC) */
#define AST_BASE_SPI		0x1E630000 /* SPI Memory Controller */
#define AST_BASE_MIC		0x1E640000 /* Memory Integrity Check Controller (MIC) */
#define AST_BASE_MAC1		0x1E660000 /* Fast Ethernet MAC Controller #1 (MAC1) */
#define AST_BASE_MAC2		0x1E680000 /* Fast Ethernet MAC Controller #2 (MAC2) */
#define AST_BASE_USB2HUB	0x1E6A0000 /* USB2.0 Hub Controller */
#define AST_BASE_USB2HC		0x1E6A1000 /* USB2.0 Host Controller */
#define AST_BASE_USB1HC		0x1E6B0000 /* USB1.1 Host Controller */
#define AST_BASE_VIC		0x1E6C0000 /* Interrupt Controller (VIC) */
#define AST_BASE_MMC		0x1E6E0000 /* SDRAM Controller (MMC) */
#define AST_BASE_USB1		0x1E6E1000 /* USB1.1 Controller */
#define AST_BASE_SCU		0x1E6E2000 /* System Control Unit (SCU) */
#define AST_BASE_HACE		0x1E6E3000 /* Hash & Crypto Engine (HACE) */
#define AST_BASE_JTAG		0x1E6E4000 /* JTAG Master */
#define AST_BASE_CRT		0x1E6E6000 /* Graphics Display Controller (CRT) */
#define AST_BASE_DMA		0x1E6E7000 /* X-DMA Controller */
#define AST_BASE_MCTP		0x1E6E8000 /* MCTP Controller */
#define AST_BASE_ADC		0x1E6E9000 /* ADC Voltage Monitor */
#define AST_BASE_LPCPLUS	0x1E6EC000 /* LPC+ Controller */
#define AST_BASE_VIDEO		0x1E700000 /* Video Engine */
#define AST_BASE_SRAM		0x1E720000 /* 32KB SRAM */
#define AST_BASE_SDIO		0x1E740000 /* SD/SDIO Controller */
#define AST_BASE_2D		0x1E760000 /* 2D Engine */
#define AST_BASE_GPIO		0x1E780000 /* GPIO Controller */
#define AST_BASE_RTC		0x1E781000 /* Real-Time Clock (RTC) */
#define AST_BASE_TIMER		0x1E782000 /* Timer #1 ∼ #8 Controller */
#define AST_BASE_UART1		0x1E783000 /* UART - #1 (LPC UART1) */
#define AST_BASE_UART5		0x1E784000 /* UART - #5 (BMC Debug) */
#define AST_BASE_WDT		0x1E785000 /* Watchdog Timer (WDT) */
#define AST_BASE_PWM		0x1E786000 /* PWM & Fan Tacho Controller */
#define AST_BASE_VUART		0x1E787000 /* Virtual UART (VUART) */
#define AST_BASE_PUART		0x1E788000 /* Pass Through UART (PUART) */
#define AST_BASE_LPC		0x1E789000 /* LPC Controller */
#define AST_BASE_I2C		0x1E78A000 /* I2C/SMBus Controller */
#define AST_BASE_PECI		0x1E78B000 /* PECI Controller */
#define AST_BASE_UART2		0x1E78D000 /* UART - #2 (LPC UART2) */
#define AST_BASE_UART3		0x1E78E000 /* UART - #3 */
#define AST_BASE_UART4		0x1E78F000 /* UART - #4 */

/* Memory */
#define AST_BASE_BMCSRAM	0x20000000 /* BMC Static Memory */
#define AST_BASE_SPIMEM		0x30000000 /* SPI Flash Memory */
#define AST_BASE_SDRAM		0x40000000 /* SDRAM */
#define AST_BASE_LPCBRIDGE	0x60000000 /* AHB Bus to LPC Bus Bridge */
#define AST_BASE_LPCPBRIDGE	0x70000000 /* AHB Bus to LPC+ Bus Bridge */

/* BMC interrupt sources */
#define AST_ID_SDRAM		0	/* SDRAM interrupt */
#define AST_ID_MIC		1	/* MIC interrupt */
#define AST_ID_MAC1		2	/* MAC1 interrupt */
#define AST_ID_MAC2		3	/* MAC2 interrupt */
#define AST_ID_CRYPTO		4	/* Crypto interrupt */
#define AST_ID_USB2		5	/* USB 2.0 Hub/Host interrupt */
#define AST_ID_XDMA		6	/* X-DMA interrupt */
#define AST_ID_VIDEO		7	/* Video Engine interrupt */
#define AST_ID_LPC		8	/* LPC interrupt */
#define AST_ID_UART1		9	/* UART1 interrupt */
#define AST_ID_UART5		10	/* UART5 interrupt */
#define AST_ID_11		11	/* Reserved */
#define AST_ID_I2C		12	/* I2C/SMBus interrupt */
#define AST_ID_USB1HID		13	/* USB 1.1 HID interrupt */
#define AST_ID_USB1HOST		14	/* USB 1.1 Host interrupt */
#define AST_ID_PECI		15	/* PECI interrupt */
#define AST_ID_TIMER1		16	/* Timer 1 interrupt */
#define AST_ID_TIMER2		17	/* Timer 2 interrupt */
#define AST_ID_TIMER3		18	/* Timer 3 interrupt */
#define AST_ID_SMC		19	/* SMC interrupt */
#define AST_ID_GPIO		20	/* GPIO interrupt */
#define AST_ID_SCU		21	/* SCU interrupt */
#define AST_ID_RTC		22	/* RTC alarm interrupt */
#define AST_ID_23		23	/* Reserved */
#define AST_ID_24		24	/* Reserved */
#define AST_ID_GRAPHICS		25	/* Graphics CRT interrupt */
#define AST_ID_SDIO		26	/* SD/SDIO interrupt */
#define AST_ID_WDT		27	/* WDT alarm interrupt */
#define AST_ID_PWM		28	/* PWM/Tachometer interrupt */
#define AST_ID_2D		29	/* Graphics 2D interrupt */
#define AST_ID_WAKEUP		30	/* System Wakeup Control */
#define AST_ID_ADC		31	/* ADC interrupt */
#define AST_ID_UART2		32	/* UART2 interrupt */
#define AST_ID_UART3		33	/* UART3 interrupt */
#define AST_ID_UART4		34	/* UART4 interrupt */
#define AST_ID_TIMER4		35	/* Timer 4 interrupt */
#define AST_ID_TIMER5		36	/* Timer 5 interrupt */
#define AST_ID_TIMER6		37	/* Timer 6 interrupt */
#define AST_ID_TIMER38		38	/* Timer 7 interrupt */
#define AST_ID_TIMER39		39	/* Timer 8 interrupt */
#define AST_ID_SGPIOMASTER	40	/* SGPIO Master interrupt */
#define AST_ID_SGPIOSLAVE	41	/* SGPIO Slave interrupt */
#define AST_ID_MCTP		42	/* MCTP interrupt */
#define AST_ID_JTAG		43	/* JTAG Master interrupt */
#define AST_ID_44		44	/* Reserved */
#define AST_ID_COPRO		45	/* Coprocessor interrupt */
#define AST_ID_MAILBOX		46	/* MailBox interrupt */
#define AST_ID_GPIOL1		47	/* GPIOL1 direct input */
#define AST_ID_GPIOL3		48	/* GPIOL3 direct input */
#define AST_ID_GPIOM1		49	/* GPIOM1 direct input */
#define AST_ID_GPIOM3		50	/* GPIOM3 direct input */

#endif /*AST2400_H*/
