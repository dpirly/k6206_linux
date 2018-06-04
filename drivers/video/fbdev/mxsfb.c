/*
 * Copyright (C) 2010 Juergen Beisert, Pengutronix
 *
 * This code is based on:
 * Author: Vitaly Wool <vital@embeddedalley.com>
 *
 * Copyright 2008-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define DRIVER_NAME "mxsfb"

/**
 * @file
 * @brief LCDIF driver for i.MX23 and i.MX28
 *
 * The LCDIF support four modes of operation
 * - MPU interface (to drive smart displays) -> not supported yet
 * - VSYNC interface (like MPU interface plus Vsync) -> not supported yet
 * - Dotclock interface (to drive LC displays with RGB data and sync signals)
 * - DVI (to drive ITU-R BT656)  -> not supported yet
 *
 * This driver depends on a correct setup of the pins used for this purpose
 * (platform specific).
 *
 * For the developer: Don't forget to set the data bus width to the display
 * in the imx_fb_videomode structure. You will else end up with ugly colours.
 * If you fight against jitter you can vary the clock delay. This is a feature
 * of the i.MX28 and you can vary it between 2 ns ... 8 ns in 2 ns steps. Give
 * the required value in the imx_fb_videomode structure.
 */

#include <linux/busfreq-imx.h>
#include <linux/console.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/pinctrl/consumer.h>
#include <linux/fb.h>
#include <linux/mxcfb.h>
#include <linux/regulator/consumer.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>
#include <linux/uaccess.h>

#include "mxc/mxc_dispdrv.h"
#include "mxsfb.h"

static struct mpu_match_lcd mpu_lcd_db[] = {
#ifdef CONFIG_FB_MXS_ST7789S_QVGA
	{
	"ST7789S-QVGA",
	{mpu_st7789s_get_lcd_videomode, mpu_st7789s_lcd_setup, mpu_st7789s_lcd_poweroff}
	},
#endif
#ifdef CONFIG_FB_MXS_ILI9225G_QCIF
	{
	"ILI9225G-QCIF",
	{mpu_ili9225g_get_lcd_videomode, mpu_ili9225g_lcd_setup, mpu_ili9225g_lcd_poweroff}
	},
#endif
	{
	"", {NULL, NULL}
	}
};

#define mxsfb_is_v3(host) (host->devdata->ipversion == 3)
#define mxsfb_is_v4(host) (host->devdata->ipversion == 4)

static const struct mxsfb_devdata mxsfb_devdata[] = {
	[MXSFB_V3] = {
		.transfer_count = LCDC_V3_TRANSFER_COUNT,
		.data = LCDC_V3_DATA,
		.cur_buf = LCDC_V3_CUR_BUF,
		.next_buf = LCDC_V3_NEXT_BUF,
		.debug0 = LCDC_V3_DEBUG0,
		.hs_wdth_mask = 0xff,
		.hs_wdth_shift = 24,
		.ipversion = 3,
	},
	[MXSFB_V4] = {
		.transfer_count = LCDC_V4_TRANSFER_COUNT,
		.data = LCDC_V4_DATA,
		.cur_buf = LCDC_V4_CUR_BUF,
		.next_buf = LCDC_V4_NEXT_BUF,
		.debug0 = LCDC_V4_DEBUG0,
		.hs_wdth_mask = 0x3fff,
		.hs_wdth_shift = 18,
		.ipversion = 4,
	},
};

#define to_imxfb_host(x) (container_of(x, struct mxsfb_info, fb_info))

static int mxsfb_map_videomem(struct fb_info *info);
static int mxsfb_unmap_videomem(struct fb_info *info);
static int mxsfb_set_par(struct fb_info *fb_info);

/* enable lcdif pix clock */
static inline void clk_enable_pix(struct mxsfb_info *host)
{
	if (!host->clk_pix_enabled && (host->clk_pix != NULL)) {
		clk_prepare_enable(host->clk_pix);
		host->clk_pix_enabled = true;
	}
}

/* disable lcdif pix clock */
static inline void clk_disable_pix(struct mxsfb_info *host)
{
	if (host->clk_pix_enabled && (host->clk_pix != NULL)) {
		clk_disable_unprepare(host->clk_pix);
		host->clk_pix_enabled = false;
	}
}

/* enable lcdif axi clock */
static inline void clk_enable_axi(struct mxsfb_info *host)
{
	if (!host->clk_axi_enabled && (host->clk_axi != NULL)) {
		clk_prepare_enable(host->clk_axi);
		host->clk_axi_enabled = true;
	}
}

/* disable lcdif axi clock */
static inline void clk_disable_axi(struct mxsfb_info *host)
{
	if (host->clk_axi_enabled && (host->clk_axi != NULL)) {
		clk_disable_unprepare(host->clk_axi);
		host->clk_axi_enabled = false;
	}
}

/* enable DISP axi clock */
static inline void clk_enable_disp_axi(struct mxsfb_info *host)
{
	if (!host->clk_disp_axi_enabled && (host->clk_disp_axi != NULL)) {
		clk_prepare_enable(host->clk_disp_axi);
		host->clk_disp_axi_enabled = true;
	}
}

/* disable DISP axi clock */
static inline void clk_disable_disp_axi(struct mxsfb_info *host)
{
	if (host->clk_disp_axi_enabled && (host->clk_disp_axi != NULL)) {
		clk_disable_unprepare(host->clk_disp_axi);
		host->clk_disp_axi_enabled = false;
	}
}

/* mask and shift depends on architecture */
static inline u32 set_hsync_pulse_width(struct mxsfb_info *host, unsigned val)
{
	return (val & host->devdata->hs_wdth_mask) <<
		host->devdata->hs_wdth_shift;
}

static inline u32 get_hsync_pulse_width(struct mxsfb_info *host, unsigned val)
{
	return (val >> host->devdata->hs_wdth_shift) &
		host->devdata->hs_wdth_mask;
}

static const struct fb_bitfield def_rgb565[] = {
	[RED] = {
		.offset = 11,
		.length = 5,
	},
	[GREEN] = {
		.offset = 5,
		.length = 6,
	},
	[BLUE] = {
		.offset = 0,
		.length = 5,
	},
	[TRANSP] = {	/* no support for transparency */
		.length = 0,
	}
};

static const struct fb_bitfield def_rgb666[] = {
	[RED] = {
		.offset = 16,
		.length = 6,
	},
	[GREEN] = {
		.offset = 8,
		.length = 6,
	},
	[BLUE] = {
		.offset = 0,
		.length = 6,
	},
	[TRANSP] = {	/* no support for transparency */
		.length = 0,
	}
};

static const struct fb_bitfield def_rgb888[] = {
	[RED] = {
		.offset = 16,
		.length = 8,
	},
	[GREEN] = {
		.offset = 8,
		.length = 8,
	},
	[BLUE] = {
		.offset = 0,
		.length = 8,
	},
	[TRANSP] = {	/* no support for transparency */
		.length = 0,
	}
};

#define bitfield_is_equal(f1, f2)  (!memcmp(&(f1), &(f2), sizeof(f1)))

static inline bool pixfmt_is_equal(struct fb_var_screeninfo *var,
				   const struct fb_bitfield *f)
{
	if (bitfield_is_equal(var->red, f[RED]) &&
	    bitfield_is_equal(var->green, f[GREEN]) &&
	    bitfield_is_equal(var->blue, f[BLUE]))
		return true;

	return false;
}

static inline unsigned chan_to_field(unsigned chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int mxsfb_mpu_wait_for_ready(struct mxsfb_info *host)
{
	unsigned int val;
	int timeout = 0;

	// Check for running
	val = readl(host->base + LCDC_CTRL);
	while(val & CTRL_RUN) {
		mdelay(1);
		timeout ++;
		if (timeout >= 1000) {
			dev_err(&host->pdev->dev, "mxsfb_mpu_wait_for_ready timeout!\n");
			return -ETIME;
		}
		val = readl(host->base + LCDC_CTRL);
	}

	// Check for running
	return 0;
}
unsigned int mxsfb_mpu_access(struct mxsfb_info *host, int mode, int rw, int data)
{
	unsigned int val, wordlen, ret = 0;

	if (mxsfb_mpu_wait_for_ready(host) != 0)
		return 0;

	writel(CTRL_MASTER,
			 host->base + LCDC_CTRL + REG_CLR);

	writel(CTRL1_BYTE_PACKING_FORMAT_MASK,
			 host->base + LCDC_CTRL1 + REG_CLR);

	val = readl(host->base + LCDC_CTRL);
	wordlen = CTRL_GET_WORD_LENGTH(val);
	writel(CTRL_WORD_LENGTH_MASK,
			 host->base + LCDC_CTRL + REG_CLR);

	writel(CTRL_YCBCR422_INPUT |
			 CTRL_INPUT_DATA_SWIZZLE_MASK,
			 host->base + LCDC_CTRL + REG_CLR);

	switch (host->mpu_lcd_sigs->interface_width)
	{
		case 8:
			writel((0x1 << CTRL1_BYTE_PACKING_FORMAT_OFFSET),
					 host->base + LCDC_CTRL1 + REG_SET);
			writel(CTRL_WORD_LENGTH_8BIT,
					 host->base + LCDC_CTRL + REG_SET);
			break;
		case 16:
			writel((0x3 << CTRL1_BYTE_PACKING_FORMAT_OFFSET),
					 host->base + LCDC_CTRL1 + REG_SET);
			writel(CTRL_WORD_LENGTH_16BIT,
					 host->base + LCDC_CTRL + REG_SET);
			break;
	}

	val = readl(host->base + host->devdata->transfer_count);
	val &= ~(TRANSFER_COUNT_H_COUNT_MASK |
		 TRANSFER_COUNT_V_COUNT_MASK);
	val |= (1 << TRANSFER_COUNT_V_COUNT_OFFSET) |
		   (1 << TRANSFER_COUNT_H_COUNT_OFFSET);
	writel(val, host->base + host->devdata->transfer_count);

	if(mode == MPU_CMD)
	{
		if (host->mpu_lcd_sigs->lcd_rs_is_gpio)
			gpio_set_value(host->mpu_lcd_sigs->lcd_rs_gpio, 0);
		writel(CTRL_DATA_SELECT,
				 host->base + LCDC_CTRL + REG_CLR);
	}
	else
	{
		if (host->mpu_lcd_sigs->lcd_rs_is_gpio)
			gpio_set_value(host->mpu_lcd_sigs->lcd_rs_gpio, 1);
		writel(CTRL_DATA_SELECT,
				 host->base + LCDC_CTRL + REG_SET);
	}

	if(rw == MPU_READ)
	{
		writel(CTRL2_READ_MODE_NUM_PACKED_SUBWORDS_MASK,
				host->base + LCDC_V4_CTRL2 + REG_CLR);
		writel((0x1 << CTRL2_READ_MODE_NUM_PACKED_SUBWORDS_OFFSET),
				 host->base + LCDC_V4_CTRL2 + REG_SET);

		writel(CTRL_READ_WRITEB,
				 host->base + LCDC_CTRL + REG_SET);
		writel(CTRL_RUN,
				 host->base + LCDC_CTRL + REG_SET);
	}
	else
	{
		writel(CTRL_READ_WRITEB,
				 host->base + LCDC_CTRL + REG_CLR);
		writel(CTRL_RUN,
				 host->base + LCDC_CTRL + REG_SET);

		writel(data,
			     host->base + host->devdata->data);
	}

	val = readl(host->base + LCDC_CTRL);
	while(val & CTRL_RUN)
	{
		if(rw == MPU_READ)
			ret = readl(host->base + host->devdata->data);

		val = readl(host->base + LCDC_CTRL);
	}

	writel(CTRL_MASTER,
			 host->base + LCDC_CTRL + REG_SET);

	writel(CTRL_WORD_LENGTH_MASK,
			 host->base + LCDC_CTRL + REG_CLR);
	writel((wordlen << CTRL_WORD_LENGTH_OFFSET),
			 host->base + LCDC_CTRL + REG_SET);  // 32 bits valid data
	writel(CTRL1_BYTE_PACKING_FORMAT_MASK,
			 host->base + LCDC_CTRL1 + REG_CLR);
	writel((0xF << CTRL1_BYTE_PACKING_FORMAT_OFFSET),
			 host->base + LCDC_CTRL1 + REG_SET);  // 32 bits valid data

	writel(CTRL_MASTER,
			 host->base + LCDC_CTRL + REG_SET);

	// For idle, set LCD_RS to high
	if (host->mpu_lcd_sigs->lcd_rs_is_gpio)
		gpio_set_value(host->mpu_lcd_sigs->lcd_rs_gpio, 1);


	return ret;
}
EXPORT_SYMBOL(mxsfb_mpu_access);

int mxsfb_mpu_refresh_panel(struct mxsfb_info *host)
{
	if (mxsfb_mpu_wait_for_ready(host) != 0)
	{
		return -ETIME;
	}

	writel(CTRL_MASTER,
			 host->base + LCDC_CTRL + REG_SET);
	writel(CTRL_RUN,
			 host->base + LCDC_CTRL + REG_SET);
	return 0;
}

void mxsfb_mpu_setup_refresh_data(struct mxsfb_info *host)
{
	unsigned int val;

	if (mxsfb_mpu_wait_for_ready(host) != 0)
		return;

	val = readl(host->base + host->devdata->transfer_count);
	val &= ~(TRANSFER_COUNT_H_COUNT_MASK |
		 TRANSFER_COUNT_V_COUNT_MASK);
	val |= (host->var.xres << TRANSFER_COUNT_H_COUNT_OFFSET) |
		   (host->var.yres << TRANSFER_COUNT_V_COUNT_OFFSET);
	writel(val, host->base + host->devdata->transfer_count);

	writel(CTRL_READ_WRITEB,
			 host->base + LCDC_CTRL + REG_CLR);
	writel(CTRL_BYPASS_COUNT,
			 host->base + LCDC_CTRL + REG_CLR);
	writel(CTRL_DATA_SELECT,
			 host->base + LCDC_CTRL + REG_SET);
	if (host->mpu_lcd_sigs->lcd_rs_is_gpio)
		gpio_set_value(host->mpu_lcd_sigs->lcd_rs_gpio, 1);

	if (host->mpu_lcd_sigs->panel_bpp == 16) {
		writel(CTRL_YCBCR422_INPUT |
				(1 << CTRL_INPUT_DATA_SWIZZLE_OFFSET),
				host->base + LCDC_CTRL + REG_SET);
	}
}

static void mxsfb_mpu_setup_interface(struct mxsfb_info *host)
{
	writel(CTRL_RUN, host->base + LCDC_CTRL + REG_CLR);
	writel(CTRL_MASTER, host->base + LCDC_CTRL + REG_CLR);

	/* Setup the bus protocol */
	if (host->mpu_lcd_sigs->bus_mode == MPU_BUS_8080)
		writel(CTRL1_MODE86,
			     host->base + LCDC_CTRL1 + REG_CLR);
	else
		writel(CTRL1_MODE86,
			     host->base + LCDC_CTRL1 + REG_SET);

	writel(CTRL1_BUSY_ENABLE,
		     host->base + LCDC_CTRL1 + REG_CLR);

	/* Take display out of reset */
	writel(CTRL1_RESET,
		     host->base + LCDC_CTRL1 + REG_SET);
	if (host->mpu_lcd_sigs->lcd_reset_is_gpio)
		gpio_set_value(host->mpu_lcd_sigs->lcd_reset_gpio, 1);

	/* VSYNC is an input by default */
	writel(VDCTRL0_VSYNC_OEB,
		    host->base + LCDC_VDCTRL0 + REG_SET);

	/*
	 * Make sure we do a high-to-low transition to reset the panel.
	 * First make it low for 100 msec, hi for 10 msec, low for 10 msec,
	 * then hi.
	 */
	writel(CTRL1_RESET,
		     host->base + LCDC_CTRL1 + REG_CLR);	/* low */
	if (host->mpu_lcd_sigs->lcd_reset_is_gpio)
		gpio_set_value(host->mpu_lcd_sigs->lcd_reset_gpio, 0);

	msleep(100);

	writel(CTRL1_RESET,
		     host->base + LCDC_CTRL1 + REG_SET);	/* high */
	if (host->mpu_lcd_sigs->lcd_reset_is_gpio)
		gpio_set_value(host->mpu_lcd_sigs->lcd_reset_gpio, 1);

	msleep(10);

	writel(CTRL1_RESET,
		     host->base + LCDC_CTRL1 + REG_CLR);	/* low */
	if (host->mpu_lcd_sigs->lcd_reset_is_gpio)
		gpio_set_value(host->mpu_lcd_sigs->lcd_reset_gpio, 0);

	/* For some panel, Reset must be held low at least 30 uSec
	 * Therefore, we'll hold it low for about 10 mSec just to be sure.
	 * Then we'll wait 1 mSec afterwards.
	 */
	msleep(10);
	writel(CTRL1_RESET,
		     host->base + LCDC_CTRL1 + REG_SET);	/* high */
	if (host->mpu_lcd_sigs->lcd_reset_is_gpio)
		gpio_set_value(host->mpu_lcd_sigs->lcd_reset_gpio, 1);
	msleep(1);

	writel(CTRL_DATA_SHIFT_DIR,
		     host->base + LCDC_CTRL + REG_CLR);

	writel(CTRL_SHIFT_NUM_BITS_MASK,
		     host->base + LCDC_CTRL + REG_CLR);

	writel(CTRL2_OUTSTANDING_REQS_MASK,
		    host->base + LCDC_V4_CTRL2 + REG_CLR);
	writel(CTRL2_OUTSTANDING_REQS_REQ_8 ,
		     host->base + LCDC_V4_CTRL2 + REG_SET);

	/* Recover on underflow */
	writel(CTRL1_RECOVERY_ON_UNDERFLOW,
	     host->base + LCDC_CTRL1 + REG_SET);

	/* Configure the input pixel format */
	writel(CTRL_YCBCR422_INPUT |
		     CTRL_WORD_LENGTH_MASK |
		     CTRL_INPUT_DATA_SWIZZLE_MASK |
			 CTRL_DATA_FORMAT_16_BIT |
		     CTRL_DATA_FORMAT_18_BIT |
		     CTRL_DATA_FORMAT_24_BIT,
		     host->base + LCDC_CTRL + REG_CLR);
	writel(CTRL1_BYTE_PACKING_FORMAT_MASK,
		     host->base + LCDC_CTRL1 + REG_CLR);

	switch (host->mpu_lcd_sigs->panel_bpp) {
	case 16:
		writel((0xF << CTRL1_BYTE_PACKING_FORMAT_OFFSET),
			     host->base + LCDC_CTRL1 + REG_SET);

		writel(CTRL_WORD_LENGTH_16BIT |
			     (0 << CTRL_INPUT_DATA_SWIZZLE_OFFSET) |(0x2 << CTRL_CSC_DATA_SWIZZLE_OFFSET),
			     host->base + LCDC_CTRL + REG_SET);

		break;
	case 18:
		writel((0xF << CTRL1_BYTE_PACKING_FORMAT_OFFSET),
				 host->base + LCDC_CTRL1 + REG_SET);
		writel(CTRL_WORD_LENGTH_18BIT |
				 (0 << CTRL_INPUT_DATA_SWIZZLE_OFFSET),
				 host->base + LCDC_CTRL + REG_SET);
		break;
	case 24:
	default:
		writel((0xF << CTRL1_BYTE_PACKING_FORMAT_OFFSET),
			     host->base + LCDC_CTRL1 + REG_SET);
		writel(CTRL_WORD_LENGTH_24BIT|
			     (0 << CTRL_INPUT_DATA_SWIZZLE_OFFSET),
			     host->base + LCDC_CTRL + REG_SET);
		break;
	}

	/* Configure the output bus width */
	writel(CTRL_LCD_DATABUS_WIDTH_MASK,
		     host->base + LCDC_CTRL + REG_CLR);
	switch (host->mpu_lcd_sigs->interface_width) {
	case 8:
		writel(CTRL_LCD_DATABUS_WIDTH_8BIT,
				 host->base + LCDC_CTRL + REG_SET);
		break;
	case 16:
		writel(CTRL_LCD_DATABUS_WIDTH_16BIT,
			     host->base + LCDC_CTRL + REG_SET);
		break;
	case 18:
		writel(CTRL_LCD_DATABUS_WIDTH_18BIT,
			     host->base + LCDC_CTRL + REG_SET);
		break;
	case 24:
	default:
		writel(CTRL_LCD_DATABUS_WIDTH_24BIT,
			     host->base + LCDC_CTRL + REG_SET);
		break;
	}

	/* Configure the MPU timing */
	writel((5 << TIMING_CMD_HOLD_OFFSET) | (5 << TIMING_CMD_SETUP_OFFSET) |
			 (5 << TIMING_DATA_HOLD_OFFSET) | (5 << TIMING_DATA_SETUP_OFFSET),
			 host->base + LCDC_TIMING);

	msleep(10);
}

static irqreturn_t mxsfb_irq_handler(int irq, void *dev_id)
{
	struct mxsfb_info *host = dev_id;
	u32 ctrl1, enable, status, acked_status;

	ctrl1 = readl(host->base + LCDC_CTRL1);
	enable = (ctrl1 & CTRL1_IRQ_ENABLE_MASK) >> CTRL1_IRQ_ENABLE_SHIFT;
	status = (ctrl1 & CTRL1_IRQ_STATUS_MASK) >> CTRL1_IRQ_STATUS_SHIFT;
	acked_status = (enable & status) << CTRL1_IRQ_STATUS_SHIFT;

	if ((acked_status & CTRL1_VSYNC_EDGE_IRQ) && host->wait4vsync) {
		writel(CTRL1_VSYNC_EDGE_IRQ,
				host->base + LCDC_CTRL1 + REG_CLR);
		writel(CTRL1_VSYNC_EDGE_IRQ_EN,
			     host->base + LCDC_CTRL1 + REG_CLR);
		host->wait4vsync = 0;
		complete(&host->vsync_complete);
	}

	if (acked_status & CTRL1_CUR_FRAME_DONE_IRQ) {
		writel(CTRL1_CUR_FRAME_DONE_IRQ,
				host->base + LCDC_CTRL1 + REG_CLR);
		writel(CTRL1_CUR_FRAME_DONE_IRQ_EN,
			     host->base + LCDC_CTRL1 + REG_CLR);
		complete(&host->flip_complete);
	}

	if (acked_status & CTRL1_UNDERFLOW_IRQ)
		writel(CTRL1_UNDERFLOW_IRQ, host->base + LCDC_CTRL1 + REG_CLR);

	if (acked_status & CTRL1_OVERFLOW_IRQ)
		writel(CTRL1_OVERFLOW_IRQ, host->base + LCDC_CTRL1 + REG_CLR);

	return IRQ_HANDLED;
}

static int mxsfb_check_var(struct fb_var_screeninfo *var,
		struct fb_info *fb_info)
{
	struct mxsfb_info *host = to_imxfb_host(fb_info);
	const struct fb_bitfield *rgb = NULL;

	if (var->xres < MIN_XRES)
		var->xres = MIN_XRES;
	if (var->yres < MIN_YRES)
		var->yres = MIN_YRES;

	if (var->xres_virtual > var->xres) {
		dev_dbg(fb_info->device, "stride not supported\n");
		return -EINVAL;
	}

	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;
	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;

	if ((var->bits_per_pixel != 32) && (var->bits_per_pixel != 16))
		var->bits_per_pixel = 32;

	switch (var->bits_per_pixel) {
	case 16:
		/* always expect RGB 565 */
		rgb = def_rgb565;
		break;
	case 32:
		if (host->is_mpu_lcd) {
			rgb = def_rgb888;
			break;
		}

		switch (host->ld_intf_width) {
		case STMLCDIF_8BIT:
			pr_debug("Unsupported LCD bus width mapping\n");
			break;
		case STMLCDIF_16BIT:
			/* 24 bit to 18 bit mapping */
			rgb = def_rgb666;
			break;
		case STMLCDIF_18BIT:
			if (pixfmt_is_equal(var, def_rgb666))
				/* 24 bit to 18 bit mapping */
				rgb = def_rgb666;
			else
				rgb = def_rgb888;
			break;
		case STMLCDIF_24BIT:
			/* real 24 bit */
			rgb = def_rgb888;
			break;
		}
		break;
	default:
		pr_debug("Unsupported colour depth: %u\n", var->bits_per_pixel);
		return -EINVAL;
	}

	/*
	 * Copy the RGB parameters for this display
	 * from the machine specific parameters.
	 */
	var->red    = rgb[RED];
	var->green  = rgb[GREEN];
	var->blue   = rgb[BLUE];
	var->transp = rgb[TRANSP];

	return 0;
}

static void mxsfb_enable_controller(struct fb_info *fb_info)
{
	struct mxsfb_info *host = to_imxfb_host(fb_info);
	u32 reg;
	int ret;

	dev_dbg(&host->pdev->dev, "%s\n", __func__);

	if (host->dispdrv && host->dispdrv->drv->setup) {
		ret = host->dispdrv->drv->setup(host->dispdrv, fb_info);
		if (ret < 0) {
			dev_err(&host->pdev->dev, "failed to setup"
				"dispdrv:%s\n", host->dispdrv->drv->name);
			return;
		}
		host->sync = fb_info->var.sync;
	}

	if (host->reg_lcd) {
		ret = regulator_enable(host->reg_lcd);
		if (ret) {
			dev_err(&host->pdev->dev,
				"lcd regulator enable failed:	%d\n", ret);
			return;
		}
	}

	/* the pixel clock should be disabled before
	 * trying to set its clock rate successfully.
	 */
	clk_disable_pix(host);
	ret = clk_set_rate(host->clk_pix,
			 PICOS2KHZ(fb_info->var.pixclock) * 1000U);
	if (ret) {
		dev_err(&host->pdev->dev,
			"lcd pixel rate set failed: %d\n", ret);

		if (host->reg_lcd) {
			ret = regulator_disable(host->reg_lcd);
			if (ret)
				dev_err(&host->pdev->dev,
					"lcd regulator disable failed: %d\n",
					ret);
		}
		return;
	}
	clk_enable_pix(host);

	if (!host->is_mpu_lcd) {
		writel(CTRL2_OUTSTANDING_REQS_REQ_16,
			host->base + LCDC_V4_CTRL2 + REG_SET);

		/* if it was disabled, re-enable the mode again */
		writel(CTRL_DOTCLK_MODE, host->base + LCDC_CTRL + REG_SET);

		/* enable the SYNC signals first, then the DMA engine */
		reg = readl(host->base + LCDC_VDCTRL4);
		reg |= VDCTRL4_SYNC_SIGNALS_ON;
		writel(reg, host->base + LCDC_VDCTRL4);

		writel(CTRL_MASTER, host->base + LCDC_CTRL + REG_SET);
		writel(CTRL_RUN, host->base + LCDC_CTRL + REG_SET);

		/* Recovery on underflow */
		writel(CTRL1_RECOVERY_ON_UNDERFLOW, host->base + LCDC_CTRL1 + REG_SET);
	}

	host->enabled = 1;

	if (host->dispdrv && host->dispdrv->drv->enable) {
		ret = host->dispdrv->drv->enable(host->dispdrv, fb_info);
		if (ret < 0)
			dev_err(&host->pdev->dev, "failed to enable "
				"dispdrv:%s\n", host->dispdrv->drv->name);
	}

}

static void mxsfb_disable_controller(struct fb_info *fb_info)
{
	struct mxsfb_info *host = to_imxfb_host(fb_info);
	unsigned loop;
	u32 reg;
	int ret;

	dev_dbg(&host->pdev->dev, "%s\n", __func__);

	if (host->dispdrv && host->dispdrv->drv->disable)
		host->dispdrv->drv->disable(host->dispdrv, fb_info);

	/*
	 * Even if we disable the controller here, it will still continue
	 * until its FIFOs are running out of data
	 */
	writel(CTRL_DOTCLK_MODE, host->base + LCDC_CTRL + REG_CLR);

	loop = 1000;
	while (loop) {
		reg = readl(host->base + LCDC_CTRL);
		if (!(reg & CTRL_RUN))
			break;
		loop--;
	}

	writel(CTRL_MASTER, host->base + LCDC_CTRL + REG_CLR);

	reg = readl(host->base + LCDC_VDCTRL4);
	writel(reg & ~VDCTRL4_SYNC_SIGNALS_ON, host->base + LCDC_VDCTRL4);

	host->enabled = 0;

	if (host->reg_lcd) {
		ret = regulator_disable(host->reg_lcd);
		if (ret)
			dev_err(&host->pdev->dev,
				"lcd regulator disable failed: %d\n", ret);
	}

}

/**
   This function compare the fb parameter see whether it was different
   parameter for hardware, if it was different parameter, the hardware
   will reinitialize. All will compared except x/y offset.
 */
static bool mxsfb_par_equal(struct fb_info *fbi, struct mxsfb_info *host)
{
	/* Here we set the xoffset, yoffset to zero, and compare two
	 * var see have different or not. */
	struct fb_var_screeninfo oldvar = host->var;
	struct fb_var_screeninfo newvar = fbi->var;

	if ((fbi->var.activate & FB_ACTIVATE_MASK) == FB_ACTIVATE_NOW &&
	    fbi->var.activate & FB_ACTIVATE_FORCE)
		return false;

	oldvar.xoffset = newvar.xoffset = 0;
	oldvar.yoffset = newvar.yoffset = 0;

	return memcmp(&oldvar, &newvar, sizeof(struct fb_var_screeninfo)) == 0;
}

static int mxsfb_set_par(struct fb_info *fb_info)
{
	struct mxsfb_info *host = to_imxfb_host(fb_info);
	u32 ctrl, vdctrl0, vdctrl4;
	int line_size, fb_size;
	int reenable = 0;
	static u32 equal_bypass = 0;

	if (likely(equal_bypass > 1)) {
		/* If parameter no change, don't reconfigure. */
		if (mxsfb_par_equal(fb_info, host))
			return 0;
	} else
		equal_bypass++;

	dev_dbg(&host->pdev->dev, "%s\n", __func__);

	/* If fb is in blank mode, it is
	 * unnecessary to really set par here.
	 * It can be delayed when unblank fb
	 */
	if (host->cur_blank != FB_BLANK_UNBLANK)
		return 0;

	line_size =  fb_info->var.xres * (fb_info->var.bits_per_pixel >> 3);
	fb_info->fix.line_length = line_size;
	fb_size = fb_info->var.yres_virtual * line_size;

	if (fb_size > fb_info->fix.smem_len) {
		dev_err(&host->pdev->dev, "exceeds the fb buffer size limit!\n");
		return -ENOMEM;
	}

	/*
	 * It seems, you can't re-program the controller if it is still running.
	 * This may lead into shifted pictures (FIFO issue?).
	 * So, first stop the controller and drain its FIFOs
	 */
	if (host->enabled && (!host->is_mpu_lcd)) {
		reenable = 1;
		mxsfb_disable_controller(fb_info);
	}

	if (host->is_mpu_lcd) {
		if (host->enabled) {
			mxsfb_mpu_setup_interface(host);
			writel(fb_info->fix.smem_start +
					fb_info->fix.line_length * fb_info->var.yoffset,
					host->base + host->devdata->cur_buf);
			writel(fb_info->fix.smem_start +
					fb_info->fix.line_length * fb_info->var.yoffset,
					host->base + host->devdata->next_buf);
			host->mpu_lcd_functions->mpu_lcd_setup(host);
			mxsfb_mpu_setup_refresh_data(host);
			mxsfb_mpu_refresh_panel(host);
		}
	} else {
		/* clear the FIFOs */
		writel(CTRL1_FIFO_CLEAR, host->base + LCDC_CTRL1 + REG_SET);

		ctrl = CTRL_BYPASS_COUNT | CTRL_MASTER |
			CTRL_SET_BUS_WIDTH(host->ld_intf_width);

		switch (fb_info->var.bits_per_pixel) {
		case 16:
			dev_dbg(&host->pdev->dev, "Setting up RGB565 mode\n");
			ctrl |= CTRL_SET_WORD_LENGTH(0);
			writel(CTRL1_SET_BYTE_PACKAGING(0xf), host->base + LCDC_CTRL1);
			break;
		case 32:
			dev_dbg(&host->pdev->dev, "Setting up RGB888/666 mode\n");
			ctrl |= CTRL_SET_WORD_LENGTH(3);
			switch (host->ld_intf_width) {
			case STMLCDIF_8BIT:
				dev_dbg(&host->pdev->dev,
						"Unsupported LCD bus width mapping\n");
				return -EINVAL;
			case STMLCDIF_16BIT:
				/* 24 bit to 18 bit mapping */
				ctrl |= CTRL_DATA_FORMAT_24_BIT; /* ignore the upper 2 bits in
						    *  each colour component
						    */
				break;
			case STMLCDIF_18BIT:
				if (pixfmt_is_equal(&fb_info->var, def_rgb666))
					/* 24 bit to 18 bit mapping */
					ctrl |= CTRL_DATA_FORMAT_24_BIT; /* ignore the upper 2 bits in
							    *  each colour component
							    */
				break;
			case STMLCDIF_24BIT:
				/* real 24 bit */
				break;
			}
			/* do not use packed pixels = one pixel per word instead */
			writel(CTRL1_SET_BYTE_PACKAGING(0x7), host->base + LCDC_CTRL1);
			break;
		default:
			dev_dbg(&host->pdev->dev, "Unhandled color depth of %u\n",
					fb_info->var.bits_per_pixel);
			return -EINVAL;
		}

		writel(ctrl, host->base + LCDC_CTRL);

		writel(TRANSFER_COUNT_SET_VCOUNT(fb_info->var.yres) |
				TRANSFER_COUNT_SET_HCOUNT(fb_info->var.xres),
				host->base + host->devdata->transfer_count);

		vdctrl0 = VDCTRL0_ENABLE_PRESENT |	/* always in DOTCLOCK mode */
			VDCTRL0_VSYNC_PERIOD_UNIT |
			VDCTRL0_VSYNC_PULSE_WIDTH_UNIT |
			VDCTRL0_SET_VSYNC_PULSE_WIDTH(fb_info->var.vsync_len);
		/* use the saved sync to avoid wrong sync information */
		if (host->sync & FB_SYNC_HOR_HIGH_ACT)
			vdctrl0 |= VDCTRL0_HSYNC_ACT_HIGH;
		if (host->sync & FB_SYNC_VERT_HIGH_ACT)
			vdctrl0 |= VDCTRL0_VSYNC_ACT_HIGH;
		if (!(host->sync & FB_SYNC_OE_LOW_ACT))
			vdctrl0 |= VDCTRL0_ENABLE_ACT_HIGH;
		if (host->sync & FB_SYNC_CLK_LAT_FALL)
			vdctrl0 |= VDCTRL0_DOTCLK_ACT_FALLING;

		writel(vdctrl0, host->base + LCDC_VDCTRL0);

		/* frame length in lines */
		writel(fb_info->var.upper_margin + fb_info->var.vsync_len +
			fb_info->var.lower_margin + fb_info->var.yres,
			host->base + LCDC_VDCTRL1);

		/* line length in units of clocks or pixels */
		writel(set_hsync_pulse_width(host, fb_info->var.hsync_len) |
			VDCTRL2_SET_HSYNC_PERIOD(fb_info->var.left_margin +
			fb_info->var.hsync_len + fb_info->var.right_margin +
			fb_info->var.xres),
			host->base + LCDC_VDCTRL2);

		writel(SET_HOR_WAIT_CNT(fb_info->var.left_margin +
			fb_info->var.hsync_len) |
			SET_VERT_WAIT_CNT(fb_info->var.upper_margin +
				fb_info->var.vsync_len),
			host->base + LCDC_VDCTRL3);

		vdctrl4 = SET_DOTCLK_H_VALID_DATA_CNT(fb_info->var.xres);
		if (mxsfb_is_v4(host))
			vdctrl4 |= VDCTRL4_SET_DOTCLK_DLY(host->dotclk_delay);
		writel(vdctrl4, host->base + LCDC_VDCTRL4);

		writel(fb_info->fix.smem_start +
				fb_info->fix.line_length * fb_info->var.yoffset,
				host->base + host->devdata->next_buf);
	}

	if (reenable && (!host->is_mpu_lcd))
		mxsfb_enable_controller(fb_info);

	/* Clear activate as not Reconfiguring framebuffer again */
	if ((fb_info->var.activate & FB_ACTIVATE_FORCE) &&
		(fb_info->var.activate & FB_ACTIVATE_MASK) == FB_ACTIVATE_NOW)
		fb_info->var.activate = FB_ACTIVATE_NOW;

	host->var = fb_info->var;
	return 0;
}

static int mxsfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
		u_int transp, struct fb_info *fb_info)
{
	unsigned int val;
	int ret = -EINVAL;

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no matter what visual we are using.
	 */
	if (fb_info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
					7471 * blue) >> 16;

	switch (fb_info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/*
		 * 12 or 16-bit True Colour.  We encode the RGB value
		 * according to the RGB bitfield information.
		 */
		if (regno < 16) {
			u32 *pal = fb_info->pseudo_palette;

			val  = chan_to_field(red, &fb_info->var.red);
			val |= chan_to_field(green, &fb_info->var.green);
			val |= chan_to_field(blue, &fb_info->var.blue);

			pal[regno] = val;
			ret = 0;
		}
		break;

	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		break;
	}

	return ret;
}

static int mxsfb_wait_for_vsync(struct fb_info *fb_info)
{
	struct mxsfb_info *host = to_imxfb_host(fb_info);
	int ret = 0;


	if (host->cur_blank != FB_BLANK_UNBLANK) {
		dev_err(fb_info->device, "can't wait for VSYNC when fb "
			"is blank\n");
		return -EINVAL;
	}

	if (host->is_mpu_lcd) {
		if (mxsfb_mpu_wait_for_ready(host) != 0)
			return -ETIME;
	} else {
		init_completion(&host->vsync_complete);

		host->wait4vsync = 1;
		writel(CTRL1_VSYNC_EDGE_IRQ_EN,
			host->base + LCDC_CTRL1 + REG_SET);
		ret = wait_for_completion_interruptible_timeout(
					&host->vsync_complete, 1 * HZ);
		if (ret == 0) {
			dev_err(fb_info->device,
				"mxs wait for vsync timeout\n");
			host->wait4vsync = 0;
			ret = -ETIME;
		} else if (ret > 0) {
			ret = 0;
		}
	}
	return ret;
}
 static int mxsfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *fb_info) ;

static int mxsfb_ioctl(struct fb_info *fb_info, unsigned int cmd,
			unsigned long arg)
{
	struct mxsfb_info *host = to_imxfb_host(fb_info);
	int ret = -EINVAL;

	switch (cmd) {
	case MXCFB_WAIT_FOR_VSYNC:
		ret = mxsfb_wait_for_vsync(fb_info);
		break;
	case MXCFB_MPU_REFRESH_PANEL:
		ret = mxsfb_mpu_refresh_panel(host);
	default:
		break;
	}
	return ret;
}

static int mxsfb_blank(int blank, struct fb_info *fb_info)
{
	struct mxsfb_info *host = to_imxfb_host(fb_info);

	host->cur_blank = blank;

	switch (blank) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		if (host->enabled) {
			if (host->is_mpu_lcd)
				host->mpu_lcd_functions->mpu_lcd_poweroff(host);
			mxsfb_disable_controller(fb_info);
			pm_runtime_put_sync_suspend(&host->pdev->dev);
		}

		clk_disable_disp_axi(host);
		clk_disable_axi(host);
		clk_disable_pix(host);
		break;

	case FB_BLANK_UNBLANK:
		fb_info->var.activate = (fb_info->var.activate & ~FB_ACTIVATE_MASK) |
				FB_ACTIVATE_NOW | FB_ACTIVATE_FORCE;

		clk_enable_pix(host);
		clk_enable_axi(host);
		clk_enable_disp_axi(host);

		if (!host->enabled) {
			pm_runtime_get_sync(&host->pdev->dev);

			writel(0, host->base + LCDC_CTRL);
			if (host->is_mpu_lcd) {
				mxsfb_enable_controller(fb_info);
				mxsfb_set_par(&host->fb_info);
			} else {
				mxsfb_set_par(&host->fb_info);
				mxsfb_enable_controller(fb_info);
			}
		}
		break;
	}
	return 0;
}

static int mxsfb_pan_display(struct fb_var_screeninfo *var,
		struct fb_info *fb_info)
{
	int ret = 0;
	struct mxsfb_info *host = to_imxfb_host(fb_info);
	unsigned offset;


	if (host->cur_blank != FB_BLANK_UNBLANK) {
		dev_dbg(fb_info->device, "can't do pan display when fb "
			"is blank\n");
		return -EINVAL;
	}

	if (var->xoffset > 0) {
		dev_dbg(fb_info->device, "x panning not supported\n");
		return -EINVAL;
	}

	if ((var->yoffset + var->yres > var->yres_virtual)) {
		dev_err(fb_info->device, "y panning exceeds\n");
		return -EINVAL;
	}

	if (host->is_mpu_lcd) {
		if (mxsfb_mpu_wait_for_ready(host) != 0)
			return -ETIMEDOUT;

		offset = fb_info->fix.line_length * var->yoffset;

		writel(fb_info->fix.smem_start + offset,
				host->base + host->devdata->next_buf);
		writel(fb_info->fix.smem_start + offset,
				host->base + host->devdata->cur_buf);
		mxsfb_mpu_refresh_panel(host);
	} else {
		init_completion(&host->flip_complete);

		offset = fb_info->fix.line_length * var->yoffset;

		/* update on next VSYNC */
		writel(fb_info->fix.smem_start + offset,
				host->base + host->devdata->next_buf);

		writel(CTRL1_CUR_FRAME_DONE_IRQ_EN,
			host->base + LCDC_CTRL1 + REG_SET);

		ret = wait_for_completion_timeout(&host->flip_complete, HZ / 2);
		if (!ret) {
			dev_err(fb_info->device,
				"mxs wait for pan flip timeout\n");
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int mxsfb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	u32 len;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;

	if (offset < info->fix.smem_len) {
		/* mapping framebuffer memory */
		len = info->fix.smem_len - offset;
		vma->vm_pgoff = (info->fix.smem_start + offset) >> PAGE_SHIFT;
	} else
		return -EINVAL;

	len = PAGE_ALIGN(len);
	if (vma->vm_end - vma->vm_start > len)
		return -EINVAL;

	/* make buffers bufferable */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		dev_dbg(info->device, "mmap remap_pfn_range failed\n");
		return -ENOBUFS;
	}

	return 0;
}

static struct fb_ops mxsfb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = mxsfb_check_var,
	.fb_set_par = mxsfb_set_par,
	.fb_setcolreg = mxsfb_setcolreg,
	.fb_ioctl = mxsfb_ioctl,
	.fb_blank = mxsfb_blank,
	.fb_pan_display = mxsfb_pan_display,
	.fb_mmap = mxsfb_mmap,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
};

static int mxsfb_restore_mode(struct mxsfb_info *host)
{
	struct fb_info *fb_info = &host->fb_info;
	unsigned line_count;
	unsigned period;
	unsigned long pa, fbsize;
	int bits_per_pixel, ofs;
	u32 transfer_count, vdctrl0, vdctrl2, vdctrl3, vdctrl4, ctrl;
	struct fb_videomode vmode;

	clk_enable_axi(host);
	clk_enable_disp_axi(host);

	/* Enable pixel clock earlier since in 7D
	 * the lcdif registers should be accessed
	 * when the pixel clock is enabled, otherwise
	 * the bus will be hang.
	 */
	clk_enable_pix(host);
	if (!host->is_mpu_lcd) {
		/* Only restore the mode when the controller is running */
		ctrl = readl(host->base + LCDC_CTRL);
		if (!(ctrl & CTRL_RUN))
			return -EINVAL;

		vdctrl0 = readl(host->base + LCDC_VDCTRL0);
		vdctrl2 = readl(host->base + LCDC_VDCTRL2);
		vdctrl3 = readl(host->base + LCDC_VDCTRL3);
		vdctrl4 = readl(host->base + LCDC_VDCTRL4);

		transfer_count = readl(host->base + host->devdata->transfer_count);

		vmode.xres = TRANSFER_COUNT_GET_HCOUNT(transfer_count);
		vmode.yres = TRANSFER_COUNT_GET_VCOUNT(transfer_count);

		switch (CTRL_GET_WORD_LENGTH(ctrl)) {
		case 0:
			bits_per_pixel = 16;
			break;
		case 3:
			bits_per_pixel = 32;
		case 1:
		default:
			return -EINVAL;
		}

		fb_info->var.bits_per_pixel = bits_per_pixel;

		vmode.pixclock = KHZ2PICOS(clk_get_rate(host->clk_pix) / 1000U);
		vmode.hsync_len = get_hsync_pulse_width(host, vdctrl2);
		vmode.left_margin = GET_HOR_WAIT_CNT(vdctrl3) - vmode.hsync_len;
		vmode.right_margin = VDCTRL2_GET_HSYNC_PERIOD(vdctrl2) - vmode.hsync_len -
			vmode.left_margin - vmode.xres;
		vmode.vsync_len = VDCTRL0_GET_VSYNC_PULSE_WIDTH(vdctrl0);
		period = readl(host->base + LCDC_VDCTRL1);
		vmode.upper_margin = GET_VERT_WAIT_CNT(vdctrl3) - vmode.vsync_len;
		vmode.lower_margin = period - vmode.vsync_len - vmode.upper_margin - vmode.yres;

		vmode.vmode = FB_VMODE_NONINTERLACED;

		vmode.sync = 0;
		if (vdctrl0 & VDCTRL0_HSYNC_ACT_HIGH)
			vmode.sync |= FB_SYNC_HOR_HIGH_ACT;
		if (vdctrl0 & VDCTRL0_VSYNC_ACT_HIGH)
			vmode.sync |= FB_SYNC_VERT_HIGH_ACT;

		pr_debug("Reconstructed video mode:\n");
		pr_debug("%dx%d, hsync: %u left: %u, right: %u, vsync: %u, upper: %u, lower: %u\n",
				vmode.xres, vmode.yres,
				vmode.hsync_len, vmode.left_margin, vmode.right_margin,
				vmode.vsync_len, vmode.upper_margin, vmode.lower_margin);
		pr_debug("pixclk: %ldkHz\n", PICOS2KHZ(vmode.pixclock));

		fb_add_videomode(&vmode, &fb_info->modelist);

		host->ld_intf_width = CTRL_GET_BUS_WIDTH(ctrl);
		host->dotclk_delay = VDCTRL4_GET_DOTCLK_DLY(vdctrl4);

		fb_info->fix.line_length = vmode.xres * (bits_per_pixel >> 3);

		pa = readl(host->base + host->devdata->cur_buf);
		fbsize = fb_info->fix.line_length * vmode.yres;
		if (pa < fb_info->fix.smem_start)
			return -EINVAL;
		if (pa + fbsize > fb_info->fix.smem_start + fb_info->fix.smem_len)
			return -EINVAL;
		ofs = pa - fb_info->fix.smem_start;
		if (ofs) {
			memmove(fb_info->screen_base, fb_info->screen_base + ofs, fbsize);
			writel(fb_info->fix.smem_start, host->base + host->devdata->next_buf);
		}

		line_count = fb_info->fix.smem_len / fb_info->fix.line_length;
		fb_info->fix.ypanstep = 1;
		fb_info->fix.ywrapstep = 1;

		host->enabled = 1;
	}

	return 0;
}

static int mxsfb_init_fbinfo_dt(struct mxsfb_info *host)
{
	struct fb_info *fb_info = &host->fb_info;
	struct fb_var_screeninfo *var = &fb_info->var;
	struct device *dev = &host->pdev->dev;
	struct device_node *np = host->pdev->dev.of_node;
	struct device_node *display_np;
	struct device_node *timings_np;
	struct display_timings *timings;
	const char *disp_dev;
	u32 width;
	int i;
	const char *lcd_panel;
	int ret = 0;

	host->id = of_alias_get_id(np, "lcdif");

	display_np = of_parse_phandle(np, "display", 0);
	if (!display_np) {
		dev_err(dev, "failed to find display phandle\n");
		return -ENOENT;
	}

	host->is_mpu_lcd = of_property_read_bool(display_np, "mpu-mode");
	if (host->is_mpu_lcd) {
		struct fb_videomode *mpu_lcd_modedb;
		struct fb_videomode fb_vm;
		int size;

		ret = of_property_read_string(display_np, "lcd_panel", &lcd_panel);
		if (ret) {
			dev_err(dev, "failed to read of property lcd_panel\n");
			goto put_display_node;
		}
		dev_info(dev, "read dts, lcd_panel is %s\n", lcd_panel);

		for (i = 0; i < ARRAY_SIZE(mpu_lcd_db); i++) {
			if (!strcmp(lcd_panel, mpu_lcd_db[i].lcd_panel)) {
				host->mpu_lcd_functions =
					&mpu_lcd_db[i].lcd_callback;
				break;
			}
		}
		if (i == ARRAY_SIZE(mpu_lcd_db)) {
			dev_err(dev, "failed to find supported lcd panel.\n");
			ret = -EINVAL;
			goto put_display_node;
		}
		host->mpu_lcd_functions->get_mpu_lcd_videomode(&mpu_lcd_modedb, &size,
						&host->mpu_lcd_sigs);

		memcpy(&fb_vm, mpu_lcd_modedb, sizeof(struct fb_videomode));
		var->bits_per_pixel = host->mpu_lcd_sigs->panel_bpp;

		switch (host->mpu_lcd_sigs->interface_width) {
		case 8:
			host->ld_intf_width = STMLCDIF_8BIT;
			break;
		case 16:

			host->ld_intf_width = STMLCDIF_16BIT;
			break;
		case 18:
			host->ld_intf_width = STMLCDIF_18BIT;
			break;
		case 24:
			host->ld_intf_width = STMLCDIF_24BIT;
			break;
		default:
			dev_err(dev, "invalid interface width value\n");
			ret = -EINVAL;
			goto put_display_node;
		}

		/* lcd reset gpio pin */
		host->mpu_lcd_sigs->lcd_reset_is_gpio = 0;
		host->mpu_lcd_sigs->lcd_reset_gpio = of_get_named_gpio(display_np, "lcd_reset_gpio", 0);
		if (gpio_is_valid(host->mpu_lcd_sigs->lcd_reset_gpio)) {
			dev_info(dev, "find lcd reset gpio pin.\n");
			 if (devm_gpio_request_one(dev, host->mpu_lcd_sigs->lcd_reset_gpio, GPIOF_OUT_INIT_HIGH, "lcd_reset") >= 0)
				 host->mpu_lcd_sigs->lcd_reset_is_gpio = 1;
		}

		/* lcd rs gpio pin */
		host->mpu_lcd_sigs->lcd_rs_is_gpio = 0;
		host->mpu_lcd_sigs->lcd_rs_gpio = of_get_named_gpio(display_np, "lcd_rs_gpio", 0);
		if (gpio_is_valid(host->mpu_lcd_sigs->lcd_rs_gpio)) {
			dev_info(dev, "find lcd rs gpio pin.\n");
			if (devm_gpio_request_one(dev, host->mpu_lcd_sigs->lcd_rs_is_gpio, GPIOF_OUT_INIT_HIGH, "lcd_rs") >= 0)
				host->mpu_lcd_sigs->lcd_rs_is_gpio = 1;
		}

		fb_add_videomode(&fb_vm, &fb_info->modelist);
		goto put_display_node;
	} else {
		ret = of_property_read_u32(display_np, "bus-width", &width);
		if (ret < 0) {
			dev_err(dev, "failed to get property bus-width\n");
			goto put_display_node;
		}

		switch (width) {
		case 8:
			host->ld_intf_width = STMLCDIF_8BIT;
			break;
		case 16:
			host->ld_intf_width = STMLCDIF_16BIT;
			break;
		case 18:
			host->ld_intf_width = STMLCDIF_18BIT;
			break;
		case 24:
			host->ld_intf_width = STMLCDIF_24BIT;
			break;
		default:
			dev_err(dev, "invalid bus-width value\n");
			ret = -EINVAL;
			goto put_display_node;
		}

		ret = of_property_read_u32(display_np, "bits-per-pixel",
					   &var->bits_per_pixel);
		if (ret < 0) {
			dev_err(dev, "failed to get property bits-per-pixel\n");
			goto put_display_node;
		}

		ret = of_property_read_string(np, "disp-dev", &disp_dev);
		if (!ret) {
			memcpy(host->disp_dev, disp_dev, strlen(disp_dev));
			/* Timing is from encoder driver */
			goto put_display_node;
		}

		timings = of_get_display_timings(display_np);
		if (!timings) {
			dev_err(dev, "failed to get display timings\n");
			ret = -ENOENT;
			goto put_display_node;
		}

		timings_np = of_find_node_by_name(display_np,
						  "display-timings");
		if (!timings_np) {
			dev_err(dev, "failed to find display-timings node\n");
			ret = -ENOENT;
			goto put_display_node;
		}

		for (i = 0; i < of_get_child_count(timings_np); i++) {
			struct videomode vm;
			struct fb_videomode fb_vm;

			ret = videomode_from_timings(timings, &vm, i);
			if (ret < 0)
				goto put_timings_node;
			ret = fb_videomode_from_videomode(&vm, &fb_vm);
			if (ret < 0)
				goto put_timings_node;

			if (!(vm.flags & DISPLAY_FLAGS_DE_HIGH))
				fb_vm.sync |= FB_SYNC_OE_LOW_ACT;
			if (vm.flags & DISPLAY_FLAGS_PIXDATA_NEGEDGE)
				fb_vm.sync |= FB_SYNC_CLK_LAT_FALL;
			fb_add_videomode(&fb_vm, &fb_info->modelist);
		}
	}

put_timings_node:
	of_node_put(timings_np);
put_display_node:
	of_node_put(display_np);
	return ret;
}

static int mxsfb_init_fbinfo(struct mxsfb_info *host)
{
	struct fb_info *fb_info = &host->fb_info;
	struct fb_var_screeninfo *var = &fb_info->var;
	struct fb_modelist *modelist;
	int ret;

	fb_info->fbops = &mxsfb_ops;
	fb_info->flags = FBINFO_FLAG_DEFAULT | FBINFO_READS_FAST;
	fb_info->fix.type = FB_TYPE_PACKED_PIXELS;
	fb_info->fix.ypanstep = 1;
	fb_info->fix.ywrapstep = 1;
	fb_info->fix.visual = FB_VISUAL_TRUECOLOR,
	fb_info->fix.accel = FB_ACCEL_NONE;

	ret = mxsfb_init_fbinfo_dt(host);
	if (ret) {
		return ret;
	}

	if (host->id < 0)
		sprintf(fb_info->fix.id, "mxs-lcdif");
	else
		sprintf(fb_info->fix.id, "mxs-lcdif%d", host->id);

	if (!list_empty(&fb_info->modelist)) {
		/* first video mode in the modelist as default video mode  */
		modelist = list_first_entry(&fb_info->modelist,
				struct fb_modelist, list);
		fb_videomode_to_var(var, &modelist->mode);
	}
	/* save the sync value getting from dtb */
	host->sync = fb_info->var.sync;

	var->nonstd = 0;
	var->activate = FB_ACTIVATE_NOW;
	var->accel_flags = 0;
	var->vmode = FB_VMODE_NONINTERLACED;

	/* init the color fields */
	mxsfb_check_var(var, fb_info);

	fb_info->fix.line_length =
		fb_info->var.xres * (fb_info->var.bits_per_pixel >> 3);
	fb_info->fix.smem_len = SZ_32M;

	/* Memory allocation for framebuffer */
	if (mxsfb_map_videomem(fb_info) < 0)
		return -ENOMEM;

	if (mxsfb_restore_mode(host))
		memset((char *)fb_info->screen_base, 0, fb_info->fix.smem_len);

	return 0;
}

static void mxsfb_dispdrv_init(struct platform_device *pdev,
			      struct fb_info *fbi)
{
	struct mxsfb_info *host = to_imxfb_host(fbi);
	struct mxc_dispdrv_setting setting;
	struct device *dev = &pdev->dev;
	char disp_dev[32];

	memset(&setting, 0x0, sizeof(setting));
	setting.fbi = fbi;

	setting.default_bpp = 16;

	memcpy(disp_dev, host->disp_dev, strlen(host->disp_dev));
	disp_dev[strlen(host->disp_dev)] = '\0';

	host->dispdrv = mxc_dispdrv_gethandle(disp_dev, &setting);
	if (IS_ERR(host->dispdrv)) {
		host->dispdrv = NULL;
		dev_info(dev, "failed to find mxc display driver %s\n",
			 disp_dev);
	} else {
		dev_info(dev, "registered mxc display driver %s\n",
			 disp_dev);
	}
}

static void mxsfb_free_videomem(struct mxsfb_info *host)
{
	struct fb_info *fb_info = &host->fb_info;

	mxsfb_unmap_videomem(fb_info);
}

/*!
 * Allocates the DRAM memory for the frame buffer.      This buffer is remapped
 * into a non-cached, non-buffered, memory region to allow palette and pixel
 * writes to occur without flushing the cache.  Once this area is remapped,
 * all virtual memory access to the video memory should occur at the new region.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @return      Error code indicating success or failure
 */
static int mxsfb_map_videomem(struct fb_info *fbi)
{
	if (fbi->fix.smem_len < fbi->var.yres_virtual * fbi->fix.line_length)
		fbi->fix.smem_len = fbi->var.yres_virtual *
				    fbi->fix.line_length;

	fbi->screen_base = dma_alloc_writecombine(fbi->device,
				fbi->fix.smem_len,
				(dma_addr_t *)&fbi->fix.smem_start,
				GFP_DMA | GFP_KERNEL);
	if (fbi->screen_base == 0) {
		dev_err(fbi->device, "Unable to allocate framebuffer memory\n");
		fbi->fix.smem_len = 0;
		fbi->fix.smem_start = 0;
		return -EBUSY;
	}

	dev_dbg(fbi->device, "allocated fb @ paddr=0x%08X, size=%d.\n",
		(uint32_t) fbi->fix.smem_start, fbi->fix.smem_len);

	fbi->screen_size = fbi->fix.smem_len;

	/* Clear the screen */
	memset((char *)fbi->screen_base, 0, fbi->fix.smem_len);
	memset((char *)fbi->screen_base, 0xF0, fbi->fix.smem_len);

	return 0;
}

/*!
 * De-allocates the DRAM memory for the frame buffer.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @return      Error code indicating success or failure
 */
static int mxsfb_unmap_videomem(struct fb_info *fbi)
{
	dma_free_writecombine(fbi->device, fbi->fix.smem_len,
			      fbi->screen_base, fbi->fix.smem_start);
	fbi->screen_base = 0;
	fbi->fix.smem_start = 0;
	fbi->fix.smem_len = 0;
	return 0;
}

static struct platform_device_id mxsfb_devtype[] = {
	{
		.name = "imx23-fb",
		.driver_data = MXSFB_V3,
	}, {
		.name = "imx28-fb",
		.driver_data = MXSFB_V4,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, mxsfb_devtype);

static const struct of_device_id mxsfb_dt_ids[] = {
	{ .compatible = "fsl,imx23-lcdif", .data = &mxsfb_devtype[0], },
	{ .compatible = "fsl,imx28-lcdif", .data = &mxsfb_devtype[1], },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxsfb_dt_ids);

static int mxsfb_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(mxsfb_dt_ids, &pdev->dev);
	struct resource *res;
	struct mxsfb_info *host;
	struct fb_info *fb_info;
	struct pinctrl *pinctrl;
	int irq = platform_get_irq(pdev, 0);
	int ret;

	if (of_id)
		pdev->id_entry = of_id->data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Cannot get memory IO resource\n");
		return -ENODEV;
	}

	fb_info = framebuffer_alloc(sizeof(struct mxsfb_info), &pdev->dev);
	if (!fb_info) {
		dev_err(&pdev->dev, "Failed to allocate fbdev\n");
		return -ENOMEM;
	}

	host = to_imxfb_host(fb_info);

	ret = devm_request_irq(&pdev->dev, irq, mxsfb_irq_handler, 0,
			  dev_name(&pdev->dev), host);
	if (ret) {
		dev_err(&pdev->dev, "request_irq (%d) failed with error %d\n",
				irq, ret);
		return -ENODEV;
	}

	host->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(host->base)) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = PTR_ERR(host->base);
		goto fb_release;
	}

	host->pdev = pdev;
	platform_set_drvdata(pdev, host);

	host->devdata = &mxsfb_devdata[pdev->id_entry->driver_data];

	host->clk_pix = devm_clk_get(&host->pdev->dev, "pix");
	if (IS_ERR(host->clk_pix)) {
		host->clk_pix = NULL;
		ret = PTR_ERR(host->clk_pix);
		goto fb_release;
	}

	host->clk_axi = devm_clk_get(&host->pdev->dev, "axi");
	if (IS_ERR(host->clk_axi)) {
		host->clk_axi = NULL;
		ret = PTR_ERR(host->clk_axi);
		goto fb_release;
	}

	host->clk_disp_axi = devm_clk_get(&host->pdev->dev, "disp_axi");
	if (IS_ERR(host->clk_disp_axi)) {
		host->clk_disp_axi = NULL;
		ret = PTR_ERR(host->clk_disp_axi);
		goto fb_release;
	}

	host->reg_lcd = devm_regulator_get(&pdev->dev, "lcd");
	if (IS_ERR(host->reg_lcd))
		host->reg_lcd = NULL;

	strncpy(host->disp_dev, "lcd", 3);

	fb_info->pseudo_palette = devm_kzalloc(&pdev->dev, sizeof(u32) * 16,
					       GFP_KERNEL);
	if (!fb_info->pseudo_palette) {
		ret = -ENOMEM;
		goto fb_release;
	}

	INIT_LIST_HEAD(&fb_info->modelist);

	pm_runtime_enable(&host->pdev->dev);

	ret = mxsfb_init_fbinfo(host);
	if (ret != 0)
		goto fb_pm_runtime_disable;

	mxsfb_dispdrv_init(pdev, fb_info);

	if (!host->dispdrv) {
		pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
		if (IS_ERR(pinctrl)) {
			ret = PTR_ERR(pinctrl);
			goto fb_pm_runtime_disable;
		}
	}

	platform_set_drvdata(pdev, fb_info);

	if (!host->enabled) {
		writel(0, host->base + LCDC_CTRL);
		if (host->is_mpu_lcd) {
			mxsfb_enable_controller(fb_info);
			mxsfb_set_par(&host->fb_info);
		} else {
			mxsfb_set_par(&host->fb_info);
			mxsfb_enable_controller(fb_info);
		}
		pm_runtime_get_sync(&host->pdev->dev);
	}

	ret = register_framebuffer(fb_info);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to register framebuffer\n");
		goto fb_destroy;
	}

	console_lock();
	ret = fb_blank(fb_info, FB_BLANK_UNBLANK);
	console_unlock();
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to unblank framebuffer\n");
		goto fb_unregister;
	}

	dev_info(&pdev->dev, "initialized\n");

	return 0;

fb_unregister:
	unregister_framebuffer(fb_info);
fb_destroy:
	if (host->enabled)
		clk_disable_unprepare(host->clk_pix);
	fb_destroy_modelist(&fb_info->modelist);
fb_pm_runtime_disable:
	pm_runtime_disable(&host->pdev->dev);
fb_release:
	framebuffer_release(fb_info);

	return ret;
}

static int mxsfb_remove(struct platform_device *pdev)
{
	struct fb_info *fb_info = platform_get_drvdata(pdev);
	struct mxsfb_info *host = to_imxfb_host(fb_info);

	if (host->enabled)
		mxsfb_disable_controller(fb_info);

	pm_runtime_disable(&host->pdev->dev);
	unregister_framebuffer(fb_info);
	mxsfb_free_videomem(host);

	framebuffer_release(fb_info);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static void mxsfb_shutdown(struct platform_device *pdev)
{
	struct fb_info *fb_info = platform_get_drvdata(pdev);
	struct mxsfb_info *host = to_imxfb_host(fb_info);

	/*
	 * Force stop the LCD controller as keeping it running during reboot
	 * might interfere with the BootROM's boot mode pads sampling.
	 */
	if (host->cur_blank == FB_BLANK_UNBLANK) {
		writel(CTRL_RUN, host->base + LCDC_CTRL + REG_CLR);
		writel(CTRL_MASTER, host->base + LCDC_CTRL + REG_CLR);
		if (host->is_mpu_lcd)
			host->mpu_lcd_functions->mpu_lcd_poweroff(host);
	}
}

#ifdef CONFIG_PM_RUNTIME
static int mxsfb_runtime_suspend(struct device *dev)
{
	release_bus_freq(BUS_FREQ_HIGH);
	dev_dbg(dev, "mxsfb busfreq high release.\n");

	return 0;
}

static int mxsfb_runtime_resume(struct device *dev)
{
	request_bus_freq(BUS_FREQ_HIGH);
	dev_dbg(dev, "mxsfb busfreq high request.\n");

	return 0;
}
#else
#define	mxsfb_runtime_suspend	NULL
#define	mxsfb_runtime_resume	NULL
#endif

#ifdef CONFIG_PM
static int mxsfb_suspend(struct device *pdev)
{
	struct fb_info *fb_info = dev_get_drvdata(pdev);
	struct mxsfb_info *host = to_imxfb_host(fb_info);
	int saved_blank;

	console_lock();
	fb_set_suspend(fb_info, 1);
	saved_blank = host->cur_blank;
	mxsfb_blank(FB_BLANK_POWERDOWN, fb_info);
	host->restore_blank = saved_blank;
	console_unlock();

	pinctrl_pm_select_sleep_state(pdev);

	return 0;
}

static int mxsfb_resume(struct device *pdev)
{
	struct fb_info *fb_info = dev_get_drvdata(pdev);
	struct mxsfb_info *host = to_imxfb_host(fb_info);

	mxsfb_resume(pdev);

	console_lock();
	mxsfb_blank(host->restore_blank, fb_info);
	fb_set_suspend(fb_info, 0);
	console_unlock();

	return 0;
}
#else
#define	mxsfb_suspend	NULL
#define	mxsfb_resume	NULL
#endif

static const struct dev_pm_ops mxsfb_pm_ops = {
	SET_RUNTIME_PM_OPS(mxsfb_runtime_suspend, mxsfb_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(mxsfb_suspend, mxsfb_resume)
};

static struct platform_driver mxsfb_driver = {
	.probe = mxsfb_probe,
	.remove = mxsfb_remove,
	.shutdown = mxsfb_shutdown,
	.id_table = mxsfb_devtype,
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = mxsfb_dt_ids,
		   .pm = &mxsfb_pm_ops,
	},
};

module_platform_driver(mxsfb_driver);

MODULE_DESCRIPTION("Freescale mxs framebuffer driver");
MODULE_AUTHOR("Sascha Hauer, Pengutronix");
MODULE_LICENSE("GPL");
