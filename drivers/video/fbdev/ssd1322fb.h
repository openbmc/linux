/*
 * SSD1322 Framebuffer Driver Header
 * ---------------------------------
 *
 * Filename: ssd1322fb.h
 * Version: 1.0
 * Date: 2024-08-14
 * Author: Jacob Levinson
 * Company: AMD
 * License: GPL
 *
 * Description:
 * ------------
 * This header file contains all necessary definitions, structures, and function
 * prototypes for the SSD1322 framebuffer driver implementation. The driver
 * supports the SSD1322 OLED display controller, enabling it to work as a
 * framebuffer device using 3-wire SPI communication.
 *
 * Features:
 * ---------
 * - Provides framebuffer support for SSD1322 OLED displays.
 * - Configurable resolution set to 128x64 pixels with 4-bit grayscale.
 * - Supports basic framebuffer operations.
 * - NHD-2.7-12864WDXX
 *
 * Requirements:
 * -------------
 * - Linux kernel version 5.x or later.
 * - SPI interface.
 *
 * Pin Configuration:
 * ------------------
 * - SPI_MOSI: MOSI (Master Out Slave In)
 * - SPI_SCLK: SCLK (Serial Clock)
 * - SPI_CS: Chip Select
 *
 *
 * Revision History:
 * -----------------
 * - 1.0: Initial release.
 *
 */

#ifndef SSD1322FB_H
#define SSD1322FB_H

#include <linux/spi/spi.h>
#include <linux/fb.h>
#include <linux/module.h>
#include <linux/delay.h>

// return values
#define SSD1322_SUCCESS 0

// Defines for SSD1322 Display
#define SSD1322_WIDTH 128
#define SSD1322_HEIGHT 64
#define SSD1322_GRAYSCALE 16
#define SSD1322_START_COL 0x1C
#define SSD1322_END_COL 0x5B
#define SSD1322_START_ROW 0x00
#define SSD1322_END_ROW 0x3F
#define SSD1322_UPPER_NIBBLE_MASK 0xF0
#define SSD1322_LOWER_NIBBLE_MASK 0x0F
#define SSD1322_NIBBLE_SHIFT 4
#define SSD1322_BITS_PER_PIXEL 4

#define SSD1322_BITS_IN_CMD 9
#define SSD1322_BITS_IN_BYTE 8
#define SSD1322_BYTE_MASK 0xFF

// SSD1322 command definitions
#define SSD1322_CMD_DISPLAY_OFF 0xAE
#define SSD1322_CMD_COMMAND_LOCK 0xFD
#define SSD1322_CMD_SET_CLOCK_DIV 0xB3
#define SSD1322_CMD_SET_MULTIPLEX_RATIO 0xCA
#define SSD1322_CMD_SET_DISPLAY_OFFSET 0xA2
#define SSD1322_CMD_FUNCTION_SELECTION 0xAB
#define SSD1322_CMD_SET_START_LINE 0xA1
#define SSD1322_CMD_SET_REMAP 0xA0
#define SSD1322_CMD_MASTER_CONTRAST 0xC7
#define SSD1322_CMD_CONTRAST_CONTROL 0xC1
#define SSD1322_CMD_PHASE_LENGTH 0xB1
#define SSD1322_CMD_PRECHARGE_VOLTAGE 0xBB
#define SSD1322_CMD_EXTERNAL_VSL 0xB4
#define SSD1322_CMD_VCOMH_VOLTAGE 0xBE
#define SSD1322_CMD_DISPLAY_MODE 0xA6
#define SSD1322_CMD_EXIT_PARTIAL_DISPLAY 0xA9
#define SSD1322_CMD_DISPLAY_ENHANCEMENT 0xD1
#define SSD1322_CMD_SET_GPIO 0xB5
#define SSD1322_CMD_DEFAULT_GRAYSCALE 0xB9
#define SSD1322_CMD_SECOND_PRECHARGE 0xB6
#define SSD1322_CMD_DISPLAY_ON 0xAF
#define SSD1322_CMD_SET_COLUMN_ADDR 0x15
#define SSD1322_CMD_SET_ROW_ADDR 0x75
#define SSD1322_CMD_WRITE_RAM 0x5C

// Data values
#define COMMAND_LOCK 0x12
#define DISPLAY_CLOCK_FREQUENCY 0x91
#define MULTIPLEX_RATIO 0x3F
#define DISPLAY_OFFSET 0x00
#define FUNCTION_SELECTION 0x01
#define START_LINE 0x00
#define REMAP_SETTINGS (0x16), (0x11)
#define MASTER_CONTRAST_LEVEL 0x0F
#define CONTRAST_CONTROL_LEVEL 0x9F
#define PHASE_LENGTH 0x72
#define PRECHARGE_VOLTAGE_LEVEL 0x1F
#define EXTERNAL_VSL (0xA0), (0xFD)
#define VCOMH_VOLTAGE_LEVEL 0x04
#define DISPLAY_MODE_NORMAL 0xA6
#define DISPLAY_ENHANCEMENT_A 0xA2
#define DISPLAY_ENHANCEMENT_B 0x20
#define GPIO_SETTING 0x00
#define SECOND_PRECHARGE_PERIOD 0x08

// Structure for the SSD1322 Register Initialization
struct ssd1322_cmd {
    u8 cmd;
    u8 *data;
    size_t len;
};

// Structure representing the SSD1322 framebuffer parameters
struct ssd1322fb_par {
	struct spi_device *spi; // SPI device
	struct fb_info *info; // Framebuffer info
	u8 *buf; // Buffer for display data
};

// Device tree match table
static const struct of_device_id ssd1322fb_of_match[];

// Function Prototypes

/**
 * ssd1322fb_setcolreg - Set grayscale register
 * @regno: Register number
 * @red: Grayscale value (0-15)
 * @green: Grayscale value (0-15)
 * @blue: Grayscale value (0-15)
 * @transp: Transparency value
 * @info: Framebuffer info structure
 *
 * Return: 0 on success, negative error code on failure.
 */
static int ssd1322fb_setcolreg(unsigned regno, unsigned red, unsigned green,
				unsigned blue, unsigned transp,
				struct fb_info *info);

/**
 * ssd1322fb_probe - Initialize and register SSD1322 framebuffer
 * @spi: SPI device structure
 *
 * Return: 0 on success, negative error code on failure.
 */
static int ssd1322fb_probe(struct spi_device *spi);

/**
 * ssd1322fb_remove - Remove and unregister SSD1322 framebuffer
 * @spi: SPI device structure
 *
 * Return: 0 on success, negative error code on failure.
 */
static void ssd1322fb_remove(struct spi_device *spi);

/**
 * ssd1322_init - Initialize SSD1322 display
 * @par: Parameters for SSD1322 framebuffer
 *
 * This function initializes the SSD1322 display controller by sending the
 * necessary commands over the SPI bus. It configures the display settings and
 * prepares it for framebuffer updates.
 *
 * Return: 0 on success, negative error code on failure.
 */
static int ssd1322_init(struct ssd1322fb_par *par);

/**
 * ssd1322fb_update_display - Update the display with new framebuffer data
 * @par: Parameters for SSD1322 framebuffer
 *
 * This function sends the updated framebuffer data to the SSD1322 display over
 * the SPI bus.
 *
 * Return: 0 on success, negative error code on failure.
 */
static int ssd1322fb_update_display(struct ssd1322fb_par *par);

/**
 * ssd1322fb_read - Read data from the framebuffer
 * @info: Framebuffer info structure
 * @buf: User buffer to read data into
 * @count: Number of bytes to read
 * @ppos: Current position in the framebuffer
 *
 * This function reads data from the SSD1322 framebuffer into the user buffer
 * provided.
 * 
 * Return: Number of bytes read, or a negative error code on failure.
 */
static ssize_t ssd1322fb_read(struct fb_info *info, char __user *buf,
			       size_t count, loff_t *ppos);

/**
 * ssd1322fb_write - Write data to the framebuffer
 * @info: Framebuffer info structure
 * @buf: User buffer containing data to write
 * @count: Number of bytes to write
 * @ppos: Current position in the framebuffer
 *
 * This function writes data from the user buffer to the SSD1322 framebuffer.
 *
 * Return: Number of bytes written, or a negative error code on failure.
 */
static ssize_t ssd1322fb_write(struct fb_info *info, const char __user *buf,
				size_t count, loff_t *ppos);

/**
 * ssd1322_cmd - Send a command to the SSD1322 display
 * @par: Parameters for SSD1322 framebuffer
 * @cmd: Command byte to be sent to the display
 * @data: Optional data bytes to be sent with the command
 * @data_len: Length of the data to be sent
 *
 * This function sends a command along with optional data to the SSD1322
 * display controller over the SPI bus. The D/C bit is sent before the command
 * byte, followed by any additional data bytes if required by the command.
 *
 * Return: 0 on success, negative error code on failure.
 */
static int ssd1322_cmd(struct ssd1322fb_par *par, u8 cmd, const u8 *data,
		       size_t data_len);

#endif /* SSD1322FB_H */
