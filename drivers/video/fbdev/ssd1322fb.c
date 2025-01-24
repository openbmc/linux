/*
 * SSD1322 Framebuffer Driver
 * --------------------------
 *
 * Filename: ssd1322fb.c
 * Version: 1.0
 * Date: 2025-02-02
 * Author: Jacob Levinson
 *         Mohsen Dolaty (clean up)
 * Company: AMD
 * License: GPL
 *
 * Reference from New Haven
 * ------------------------
 * https://github.com/newhavendisplay
 * https://github.com/NewhavenDisplay/NHD-2.7-12864WD_Example
 *
 * Description:
 * ------------
 * This source file implements the SSD1322 framebuffer driver for a
 * monochrome OLED display. It supports 4-bit grayscale operations and
 * integrates with the system via 3-wire SPI communication.
 *
 * Features:
 * ---------
 * - 128x64 pixel resolution.
 * - 4-bit grayscale support.
 * - Basic framebuffer operations.
 * - NHD-2.7-12864WDXX
 *
 * Requirements:
 * -------------
 * - SPI interface.
 *
 * Pin Configuration:
 * ------------------
 * - SPI_MOSI: MOSI (Master Out Slave In)
 * - SPI_SCLK: SCLK (Serial Clock)
 * - SPI_CS: Chip Select
 *
 * Revision History:
 * -----------------
 * - 1.0: Initial release.
 *
 */

#include "ssd1322fb.h"

static int ssd1322_init(struct ssd1322fb_par *par)
{
	struct ssd1322_cmd cmds[] = {
		{SSD1322_CMD_DISPLAY_ON, NULL, 0},
		{SSD1322_CMD_COMMAND_LOCK, (u8[]){COMMAND_LOCK}, 1},
		{SSD1322_CMD_SET_CLOCK_DIV, (u8[]){DISPLAY_CLOCK_FREQUENCY}, 1},
		{SSD1322_CMD_SET_MULTIPLEX_RATIO, (u8[]){MULTIPLEX_RATIO}, 1},
		{SSD1322_CMD_SET_DISPLAY_OFFSET, (u8[]){DISPLAY_OFFSET}, 1},
		{SSD1322_CMD_FUNCTION_SELECTION, (u8[]){FUNCTION_SELECTION}, 1},
		{SSD1322_CMD_SET_START_LINE, (u8[]){START_LINE}, 1},
		{SSD1322_CMD_SET_REMAP, (u8[]){REMAP_SETTINGS}, 2},
		{SSD1322_CMD_MASTER_CONTRAST, (u8[]){MASTER_CONTRAST_LEVEL}, 1},
		{SSD1322_CMD_CONTRAST_CONTROL, (u8[]){CONTRAST_CONTROL_LEVEL}, 1},
		{SSD1322_CMD_PHASE_LENGTH, (u8[]){PHASE_LENGTH}, 1},
		{SSD1322_CMD_PRECHARGE_VOLTAGE, (u8[]){PRECHARGE_VOLTAGE_LEVEL}, 1},
		{SSD1322_CMD_EXTERNAL_VSL, (u8[]){EXTERNAL_VSL}, 2},
		{SSD1322_CMD_VCOMH_VOLTAGE, (u8[]){VCOMH_VOLTAGE_LEVEL}, 1},
		{SSD1322_CMD_DISPLAY_MODE, NULL, 0},
		{SSD1322_CMD_EXIT_PARTIAL_DISPLAY, NULL, 0},
		{SSD1322_CMD_DISPLAY_ENHANCEMENT, (u8[]){DISPLAY_ENHANCEMENT_A, DISPLAY_ENHANCEMENT_B}, 2},
		{SSD1322_CMD_SET_GPIO, (u8[]){GPIO_SETTING}, 1},
		{SSD1322_CMD_DEFAULT_GRAYSCALE, NULL, 0},
		{SSD1322_CMD_SECOND_PRECHARGE, (u8[]){SECOND_PRECHARGE_PERIOD}, 1},
		{SSD1322_CMD_DISPLAY_ON, NULL, 0}
	};
	int ret;

	for (int i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
		ret = ssd1322_cmd(par, cmds[i].cmd, cmds[i].data, cmds[i].len);
		if (ret) {
			dev_err(&par->spi->dev, "ssd1322fb oled init Error on index %d\n", i);
			return ret;
		}
	}

	dev_info(&par->spi->dev, "ssd1322fb oled init done.\n");
	return SSD1322_SUCCESS;
}

static ssize_t ssd1322fb_read(struct fb_info *info, char __user *buf,
			       size_t count, loff_t *ppos)
{
	struct ssd1322fb_par *par;
	char *src;

	// Initializing variables
	par = info->par;

	// Check if the position is valid
	if (*ppos >= info->fix.smem_len)
		return SSD1322_SUCCESS; // No more data to read

	// Adjust count if it goes beyond the end of the buffer
	if (*ppos + count > info->fix.smem_len)
		count = info->fix.smem_len - *ppos;

	// Point to the framebuffer memory
	src = (char *)info->screen_base + *ppos;

	// Copy framebuffer memory to user-space buffer
	if (copy_to_user(buf, src, count)) {
		dev_err(&par->spi->dev,"ssd1322fb_read: Error copy to user \n");
		return -EFAULT;
	}

	// Update the position pointer
	*ppos += count;

	return count; // Return the number of bytes read
}

// Function that is called when data is written to the fb
static ssize_t ssd1322fb_write(struct fb_info *info, const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct ssd1322fb_par *par;
	char *dst;

	// Initializing variables
	par = info->par;
	dst = (char *)info->screen_base + *ppos;

	dev_dbg(&par->spi->dev, "SPI ssd1322fb_write called!\n");
	dev_dbg(&par->spi->dev, "ppos is: %llu, count is: %zu\n", *ppos, count);

	// Check for overflow and adjust count if necessary
	if (*ppos >= info->fix.smem_len) {
		dev_err(&par->spi->dev,
			"issd1322fb_write:  Error ppos (%llu) is beyond the framebuffer size (%u)\n",
			*ppos, info->fix.smem_len);
		return -ENOSPC; // No space left in the framebuffer
	}

	if (*ppos + count > info->fix.smem_len) {
		dev_info(
			&par->spi->dev,
			"Framebuffer write adjustment: ppos (%llu) + count (%zu) exceeds framebuffer size (%u)\n",
			*ppos, count, info->fix.smem_len);
		count = info->fix.smem_len - *ppos;
	}

	// Copy data from user space to framebuffer memory
	if (copy_from_user(dst, buf, count)) {
		dev_err(&par->spi->dev,"ssd1322fb_write: Error copy from user \n");
		return -EFAULT;
	}

	// Update the position pointer
	*ppos += count;

	// Trigger display update here
	if (ssd1322fb_update_display(par)) {
		dev_info(&par->spi->dev,"ssd1322fb_write: Update Display failed \n");
	}

	return count; // Return the number of bytes written
}

static int ssd1322fb_update_display(struct ssd1322fb_par *par)
{
	u8 *image;
	int ret;
	int i, j;
	u8 col[2];
	u8 row[2];
	u8 *duplicated_image;
	int duplicated_size;

	// Initialize variables
	image = par->info->screen_base;
	col[0] = SSD1322_START_COL;
	col[1] = SSD1322_END_COL;
	row[0] = SSD1322_START_ROW;
	row[1] = SSD1322_END_ROW;

	// Set column address
	ret = ssd1322_cmd(par, SSD1322_CMD_SET_COLUMN_ADDR, col, 2);
	if (ret) {
		dev_err(&par->spi->dev, "ssd1322fb_update_display: Failed to Set Col \n");
		return ret;
	}

	// Set row address
	ret = ssd1322_cmd(par, SSD1322_CMD_SET_ROW_ADDR, row, 2);
	if (ret) {
		dev_err(&par->spi->dev, "ssd1322fb_update_display: Failed to Set Row \n");
		return ret;
	}

	// Calculate the size for the duplicated image data
	// Image must have each nibble duplicated horizonatally
	duplicated_size = SSD1322_WIDTH * SSD1322_HEIGHT;
	duplicated_image = kmalloc(duplicated_size, GFP_KERNEL);
	if (!duplicated_image) {
		dev_err(&par->spi->dev,
			"ssd1322fb_update_display: Failed to allocate memory \n");
		return -ENOMEM;
	}
	// Initialize the allocated memory to 0
	memset(duplicated_image, 0, duplicated_size);

	// Duplicate and remap image data

	// For each row
	for (i = 0; i < SSD1322_HEIGHT; i++) {
		// For each column in the original image (128 columns, 64 bytes)
		for (j = 0; j < SSD1322_WIDTH / 2; j++) {
			// Get the original byte (2 pixels)
			u8 byte = image[i * SSD1322_HEIGHT + j];
			// Isolate the upper and lower nibbles
			u8 upper_nibble = (byte & SSD1322_UPPER_NIBBLE_MASK) >> SSD1322_NIBBLE_SHIFT;
			u8 lower_nibble = byte & SSD1322_LOWER_NIBBLE_MASK;
			// Duplicate each nibble into its own byte
			duplicated_image[i * SSD1322_WIDTH + j * 2] =
				(upper_nibble << SSD1322_NIBBLE_SHIFT) | upper_nibble;
			duplicated_image[i * SSD1322_WIDTH + j * 2 + 1] =
				(lower_nibble << SSD1322_NIBBLE_SHIFT) | lower_nibble;
		}
	}

	// Write the duplicated image data to RAM
	ret = ssd1322_cmd(par, SSD1322_CMD_WRITE_RAM, duplicated_image,
			  duplicated_size);
	kfree(duplicated_image);

	if (ret) {
		dev_err(&par->spi->dev,
			"SPI transfer for duplicated_image failed: %d\n", ret);
		return ret;
	}

	dev_dbg(&par->spi->dev,
		"SPI transfer for duplicated_image complete!\n");

	return SSD1322_SUCCESS;
}

static int ssd1322_cmd(struct ssd1322fb_par *par, u8 cmd, const u8 *data,
		       size_t data_len)
{
	struct spi_device *spi = par->spi;
	size_t total_bits;
	size_t total_bytes;
	u8 *tx_buf;
	int bit_offset;
	int ret;
	size_t i;
	struct spi_transfer xfer;
	struct spi_message msg;

	total_bits = (data_len + 1) * SSD1322_BITS_IN_CMD;
	total_bytes = (total_bits + (SSD1322_BITS_IN_BYTE -1)) / SSD1322_BITS_IN_BYTE; // Round up to nearest byte
	tx_buf = kmalloc(total_bytes, GFP_KERNEL);
	if (!tx_buf)
		return -ENOMEM;
	memset(tx_buf, 0, total_bytes);

	// Fill tx_buf with cmd and data

	// Add cmd (command bit is 0)
	bit_offset = 1;
	// Insert the most significant bits of the command
	tx_buf[0] |= (cmd >> bit_offset);
	// Insert the remaining bits of the command
	tx_buf[1] |= (cmd << (SSD1322_BITS_IN_BYTE - bit_offset)) & SSD1322_BYTE_MASK;

	bit_offset += SSD1322_BITS_IN_BYTE; // 1 bit command flag + 8 bits command

	// Add data (data bit is 1)
	for (i = 0; i < data_len; i++) {
		int byte_index = bit_offset / SSD1322_BITS_IN_BYTE;
		int bit_index = bit_offset % SSD1322_BITS_IN_BYTE;
		// Set data/command bit to 1 for data
		tx_buf[byte_index] |= (1 << ((SSD1322_BITS_IN_BYTE -1) - bit_index));
		if (bit_index < (SSD1322_BITS_IN_BYTE - 1)) {
			tx_buf[byte_index] |= (data[i] >> (bit_index + 1));
			if (byte_index + 1 < total_bytes) {
				tx_buf[byte_index + 1] |=
					(data[i] << ((SSD1322_BITS_IN_BYTE - 1) - bit_index)) & SSD1322_BYTE_MASK;
			}
		} else {
			if (byte_index + 1 < total_bytes) {
				tx_buf[byte_index + 1] |= data[i];
			}
		}
		bit_offset += SSD1322_BITS_IN_CMD;
	}

	// SPI transfer setup
	spi_message_init(&msg);
	memset(&xfer, 0, sizeof(xfer)); // Initialize the spi_transfer structure
	xfer.tx_buf = tx_buf;
	xfer.len = total_bytes;
	xfer.cs_change = 0; // Ensure CS is deasserted after transfer
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(spi, &msg);
	if (ret)
		dev_err(&spi->dev, "Failed to write to SSD1322: %d\n", ret);

	kfree(tx_buf);
	return ret;
}

// Function to set grayscale values
static int ssd1322fb_setcolreg(unsigned regno, unsigned red, unsigned green,
				unsigned blue, unsigned transp,
				struct fb_info *info)
{
	// Ensure grayscale is within range
	if (red >= SSD1322_GRAYSCALE)
		return -EINVAL;

	// Implement grayscale setting here if needed
	// For monochrome, just ensure value fits within the expected range
	return SSD1322_SUCCESS;
}

// Framebuffer operations structure
static struct fb_ops ssd1322fb_ops = {
	.owner = THIS_MODULE,
	.fb_setcolreg = ssd1322fb_setcolreg,
	.fb_fillrect = sys_fillrect,
	.fb_copyarea = sys_copyarea,
	.fb_imageblit = sys_imageblit,
	.fb_write = ssd1322fb_write,
	.fb_read = ssd1322fb_read,
};

// Probe function for initializing the SSD1322 driver
static int ssd1322fb_probe(struct spi_device *spi)
{
	struct fb_info *info;
	struct ssd1322fb_par *par;
	int retval;

	dev_info(&spi->dev, "ssd1322fb_probe: start\n");

	retval = -ENOMEM;
	info = framebuffer_alloc(sizeof(struct ssd1322fb_par), &spi->dev);
	if (!info) {
		dev_err(&spi->dev, "ssd1322fb_probe: Error No Mem\n");
		return retval;
	}

	par = info->par;
	par->spi = spi;
	par->info = info;
	// Allocate buffer for grayscale
	par->buf = vzalloc(SSD1322_WIDTH * SSD1322_HEIGHT / 2);
	if (!par->buf) {
		dev_err(&spi->dev, "ssd1322fb_probe: Error allocate memory buffer\n");
		goto err_alloc;
	}

	info->screen_base = par->buf;
	info->fbops = &ssd1322fb_ops;
	info->var.xres = SSD1322_WIDTH;
	info->var.yres = SSD1322_HEIGHT;
	info->var.bits_per_pixel = SSD1322_BITS_PER_PIXEL; // 4 bits per pixel for grayscale
	info->fix.line_length = SSD1322_WIDTH / 2;
	info->fix.smem_len = SSD1322_WIDTH * SSD1322_HEIGHT / 2;

	spi_set_drvdata(spi, info);

	retval = register_framebuffer(info);
	if (retval < 0) {
		dev_err(&spi->dev, "ssd1322fb_probe: Error from register_framebuffer %d\n", retval);
		goto err_fb;
	}

	dev_info(&spi->dev,
		"fb%d: %s frame buffer device, using %d KiB of video memory\n",
		info->node, info->fix.id, info->fix.smem_len >> 10);

	retval = ssd1322_init(par);
	if (retval) {
		dev_err(&spi->dev, "ssd1322fb_probe: Error from ssd1322_init %d\n", retval);
		goto err_fb;
	}

	return SSD1322_SUCCESS;

err_fb:
	vfree(par->buf);
err_alloc:
	framebuffer_release(info);
	return retval;
}

// Remove function for cleaning up the SSD1322 driver
static void ssd1322fb_remove(struct spi_device *spi)
{
	struct fb_info *info = spi_get_drvdata(spi);
	struct ssd1322fb_par *par = info->par;

	unregister_framebuffer(info);
	vfree(par->buf);
	framebuffer_release(info);
}

// Device tree match table
static const struct of_device_id ssd1322fb_of_match[] = {
	{
		.compatible = "ssd,ssd1322",
	},
	{}
};
MODULE_DEVICE_TABLE(of, ssd1322fb_of_match);

// SPI driver structure for the SSD1322
static struct spi_driver ssd1322fb_driver = {
    .driver = {
        .name   = "ssd1322fb",
        .owner  = THIS_MODULE,
        .of_match_table = ssd1322fb_of_match,
    },
    .probe  = ssd1322fb_probe,
    .remove = ssd1322fb_remove,
};

module_spi_driver(ssd1322fb_driver);

MODULE_DESCRIPTION("SSD1322 Framebuffer Driver");
MODULE_AUTHOR("Jacob Levinson");
MODULE_LICENSE("GPL");
