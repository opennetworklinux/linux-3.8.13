/*
 * Copyright 2010-2011, 2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Freescale SSD1289 TFT LCD framebuffer driver
 *
 * Author: Alison Wang <b18965@freescale.com>
 *         Jason Jin <Jason.jin@freescale.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/timer.h>
#include <linux/kthread.h>
#include <linux/platform_data/video-twrfb.h>

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#define SSD1289_REG_OSCILLATION		0x00
#define SSD1289_REG_DRIVER_OUT_CTRL	0x01
#define SSD1289_REG_LCD_DRIVE_AC	0x02
#define SSD1289_REG_POWER_CTRL_1	0x03
#define SSD1289_REG_COMPARE_1		0x05
#define SSD1289_REG_COMPARE_2		0x06
#define SSD1289_REG_DISPLAY_CTRL	0x07
#define SSD1289_REG_FRAME_CYCLE		0x0b
#define SSD1289_REG_POWER_CTRL_2	0x0c
#define SSD1289_REG_POWER_CTRL_3	0x0d
#define SSD1289_REG_POWER_CTRL_4	0x0e
#define SSD1289_REG_GATE_SCAN_START	0x0f
#define SSD1289_REG_SLEEP_MODE		0x10
#define SSD1289_REG_ENTRY_MODE		0x11
#define SSD1289_REG_OPT_SPEED_3		0x12
#define SSD1289_REG_H_PORCH		0x16
#define SSD1289_REG_V_PORCH		0x17
#define SSD1289_REG_POWER_CTRL_5	0x1e
#define SSD1289_REG_GDDRAM_DATA		0x22
#define SSD1289_REG_WR_DATA_MASK_1	0x23
#define SSD1289_REG_WR_DATA_MASK_2	0x24
#define SSD1289_REG_FRAME_FREQUENCY	0x25
#define SSD1289_REG_OPT_SPEED_1		0x28
#define SSD1289_REG_OPT_SPEED_2		0x2f
#define SSD1289_REG_GAMMA_CTRL_1	0x30
#define SSD1289_REG_GAMMA_CTRL_2	0x31
#define SSD1289_REG_GAMMA_CTRL_3	0x32
#define SSD1289_REG_GAMMA_CTRL_4	0x33
#define SSD1289_REG_GAMMA_CTRL_5	0x34
#define SSD1289_REG_GAMMA_CTRL_6	0x35
#define SSD1289_REG_GAMMA_CTRL_7	0x36
#define SSD1289_REG_GAMMA_CTRL_8	0x37
#define SSD1289_REG_GAMMA_CTRL_9	0x3a
#define SSD1289_REG_GAMMA_CTRL_10	0x3b
#define SSD1289_REG_V_SCROLL_CTRL_1	0x41
#define SSD1289_REG_V_SCROLL_CTRL_2	0x42
#define SSD1289_REG_H_RAM_ADR_POS	0x44
#define SSD1289_REG_V_RAM_ADR_START	0x45
#define SSD1289_REG_V_RAM_ADR_END	0x46
#define SSD1289_REG_FIRST_WIN_START	0x48
#define SSD1289_REG_FIRST_WIN_END	0x49
#define SSD1289_REG_SECND_WIN_START	0x4a
#define SSD1289_REG_SECND_WIN_END	0x4b
#define SSD1289_REG_GDDRAM_X_ADDR	0x4e
#define SSD1289_REG_GDDRAM_Y_ADDR	0x4f

/* The register value definition */
#define	SSD1289_DEVICE_ID		0x8989
#define SSD1289_OSC_ENABLE		0x0001
#define SSD1289_OSC_DISABLE		0x0000
#define SSD1289_DISPLAY_DISABLE		0x0200
#define SSD1289_DISPLAY_ENABLE		0x0233
#define SSD1289_POWER_CTRL1_VALUE	0xaeac
#define SSD1289_POWER_CTRL2_VALUE	0x0007
#define SSD1289_POWER_CTRL3_VALUE	0x000f
#define SSD1289_POWER_CTRL4_VALUE	0x2900
#define	SSD1289_POWER_CTRL5_VALUE	0x00b3
#define	SSD1289_DRIVER_OUT_CTRL_VALUE	0x2b3f
#define	SSD1289_LCD_WAVE_INVERSION	0x0600
#define SSD1289_SLEEP_MODE_ENABLE	0x0001
#define SSD1289_SLEEP_MODE_DISABLE	0x0000
#define SSD1289_ENTRY_MODE_64KCOLOR	0x60a8
#define	SSD1289_COMPARE_DISABLE		0x0000
#define	SSD1289_H_PORCH_VALUE		0xef1c
#define	SSD1289_V_PORCH_VALUE		0x0003
#define	SSD1289_FRAME_CYCLE_VALUE	0x5312
#define	SSD1289_V_SCROLL_LENGTH		0x0000
#define	SSD1289_GAMMA_CTRL1_VALUE	0x0707
#define	SSD1289_GAMMA_CTRL2_VALUE	0x0704
#define	SSD1289_GAMMA_CTRL3_VALUE	0x0204
#define	SSD1289_GAMMA_CTRL4_VALUE	0x0201
#define	SSD1289_GAMMA_CTRL5_VALUE	0x0203
#define	SSD1289_GAMMA_CTRL6_VALUE	0x0204
#define	SSD1289_GAMMA_CTRL7_VALUE	0x0204
#define	SSD1289_GAMMA_CTRL8_VALUE	0x0502
#define	SSD1289_GAMMA_CTRL9_VALUE	0x0302
#define	SSD1289_GAMMA_CTRL10_VALUE	0x0500
#define	SSD1289_FRAME_FREQUENCY_80HZ	0xe000
#define	SSD1289_OPT_SPEED1_VALUE	0x0006
#define	SSD1289_OPT_SPEED2_VALUE	0x12ae
#define	SSD1289_OPT_SPEED3_VALUE	0x6ceb
#define	SSD1289_H_RAM_ADR_POS_VALUE	0xef00
#define	SSD1289_GDDRAM_X_ADDR_VALUE	0x00ef
#define	SSD1289_GDDRAM_Y_ADDR_VALUE	0x0000
#define	SSD1289_WINDOW_Y_SIZE		0x013f


struct ssd1289 {
	unsigned short __iomem *cmd;
	unsigned short __iomem *data;
};

struct fsl_ssd1289_fb_info {
	struct device *dev;
	struct ssd1289 ssd1289_reg;
	int openflag;
	struct spi_device *spidev;

	struct task_struct *task;
	unsigned long pseudo_palette[16];
};

static int ssd1289_set_reg(struct fb_info *info, unsigned short reg,
				unsigned short data)
{
	struct fsl_ssd1289_fb_info *fbinfo = info->par;
	unsigned short __iomem *cmd_addr, *data_addr;

	cmd_addr = fbinfo->ssd1289_reg.cmd;
	data_addr = fbinfo->ssd1289_reg.data;

	out_be16(cmd_addr, reg);
	out_be16(data_addr, data);

	return 0;
}

static int ssd1289_write_data(struct fb_info *info, unsigned short *data,
				size_t size)
{
	struct fsl_ssd1289_fb_info *fbinfo = info->par;
	unsigned short __iomem *cmd_addr, *data_addr;
	int i;
	unsigned short tmp;

	cmd_addr = fbinfo->ssd1289_reg.cmd;
	data_addr = fbinfo->ssd1289_reg.data;

	out_be16(cmd_addr, SSD1289_REG_GDDRAM_DATA);

	if (data == NULL)
		return 0;
	else {
		for (i = 0; i < size; i += 2) {
			tmp = ((((*data) & 0x00ff) << 8) |
				(((*data) & 0xff00) >> 8));
			out_be16(data_addr, tmp);
			data++;
		}
	}

	return 0;
}

static void fsl_ssd1289_enable_lcd(struct fb_info *info)
{
	int i;

	ssd1289_set_reg(info, SSD1289_REG_DISPLAY_CTRL,
			SSD1289_DISPLAY_DISABLE);

	ssd1289_set_reg(info, SSD1289_REG_OSCILLATION, SSD1289_OSC_DISABLE);

	mdelay(100);

	/* turn on the oscillator */
	ssd1289_set_reg(info, SSD1289_REG_OSCILLATION, SSD1289_OSC_ENABLE);

	mdelay(100);
	/* power control 1 */
	ssd1289_set_reg(info, SSD1289_REG_POWER_CTRL_1,
			SSD1289_POWER_CTRL1_VALUE);

	/* power control 2 */
	ssd1289_set_reg(info, SSD1289_REG_POWER_CTRL_2,
			SSD1289_POWER_CTRL2_VALUE);

	/* power control 3 */
	ssd1289_set_reg(info, SSD1289_REG_POWER_CTRL_3,
			SSD1289_POWER_CTRL3_VALUE);

	/* power control 4 */
	ssd1289_set_reg(info, SSD1289_REG_POWER_CTRL_4,
			SSD1289_POWER_CTRL4_VALUE);

	/* power control 5 */
	ssd1289_set_reg(info, SSD1289_REG_POWER_CTRL_5,
			SSD1289_POWER_CTRL5_VALUE);

	mdelay(15);
	/* driver output control */
	ssd1289_set_reg(info, SSD1289_REG_DRIVER_OUT_CTRL,
			SSD1289_DRIVER_OUT_CTRL_VALUE);

	/* lcd-driving-waveform control */
	ssd1289_set_reg(info, SSD1289_REG_LCD_DRIVE_AC,
			SSD1289_LCD_WAVE_INVERSION);

	/* sleep mode */
	ssd1289_set_reg(info, SSD1289_REG_SLEEP_MODE,
			SSD1289_SLEEP_MODE_DISABLE);

	/* entry mode */
	ssd1289_set_reg(info, SSD1289_REG_ENTRY_MODE,
			SSD1289_ENTRY_MODE_64KCOLOR);

	mdelay(15);
	/* compare register */
	ssd1289_set_reg(info, SSD1289_REG_COMPARE_1,
			SSD1289_COMPARE_DISABLE);

	ssd1289_set_reg(info, SSD1289_REG_COMPARE_2,
			SSD1289_COMPARE_DISABLE);

	/* horizontal porch */
	ssd1289_set_reg(info, SSD1289_REG_H_PORCH, SSD1289_H_PORCH_VALUE);

	/* vertical porch */
	ssd1289_set_reg(info, SSD1289_REG_V_PORCH, SSD1289_V_PORCH_VALUE);

	/* display control */
	ssd1289_set_reg(info, SSD1289_REG_DISPLAY_CTRL,
			SSD1289_DISPLAY_ENABLE);

	/* frame cycle control */
	ssd1289_set_reg(info, SSD1289_REG_FRAME_CYCLE,
			SSD1289_FRAME_CYCLE_VALUE);

	/* gate scan position */
	ssd1289_set_reg(info, SSD1289_REG_GATE_SCAN_START, 0x0000);

	mdelay(20);
	/* vertical scroll control */
	ssd1289_set_reg(info, SSD1289_REG_V_SCROLL_CTRL_1,
			SSD1289_V_SCROLL_LENGTH);

	ssd1289_set_reg(info, SSD1289_REG_V_SCROLL_CTRL_2,
			SSD1289_V_SCROLL_LENGTH);

	/* 1st screen driving position */
	ssd1289_set_reg(info, SSD1289_REG_FIRST_WIN_START, 0x0000);

	ssd1289_set_reg(info, SSD1289_REG_FIRST_WIN_END,
			SSD1289_WINDOW_Y_SIZE);

	/* 2nd screen driving position */
	ssd1289_set_reg(info, SSD1289_REG_SECND_WIN_START, 0x0000);

	ssd1289_set_reg(info, SSD1289_REG_SECND_WIN_END, 0x0000);

	mdelay(20);
	/* gamma control */
	ssd1289_set_reg(info, SSD1289_REG_GAMMA_CTRL_1,
			SSD1289_GAMMA_CTRL1_VALUE);

	ssd1289_set_reg(info, SSD1289_REG_GAMMA_CTRL_2,
			SSD1289_GAMMA_CTRL2_VALUE);

	ssd1289_set_reg(info, SSD1289_REG_GAMMA_CTRL_3,
			SSD1289_GAMMA_CTRL3_VALUE);

	ssd1289_set_reg(info, SSD1289_REG_GAMMA_CTRL_4,
			SSD1289_GAMMA_CTRL4_VALUE);

	ssd1289_set_reg(info, SSD1289_REG_GAMMA_CTRL_5,
			SSD1289_GAMMA_CTRL5_VALUE);

	ssd1289_set_reg(info, SSD1289_REG_GAMMA_CTRL_6,
			SSD1289_GAMMA_CTRL6_VALUE);

	ssd1289_set_reg(info, SSD1289_REG_GAMMA_CTRL_7,
			SSD1289_GAMMA_CTRL7_VALUE);

	ssd1289_set_reg(info, SSD1289_REG_GAMMA_CTRL_8,
			SSD1289_GAMMA_CTRL8_VALUE);

	ssd1289_set_reg(info, SSD1289_REG_GAMMA_CTRL_9,
			SSD1289_GAMMA_CTRL9_VALUE);

	ssd1289_set_reg(info, SSD1289_REG_GAMMA_CTRL_10,
			SSD1289_GAMMA_CTRL10_VALUE);

	/* ram write data mask */
	ssd1289_set_reg(info, SSD1289_REG_WR_DATA_MASK_1, 0x0000);

	ssd1289_set_reg(info, SSD1289_REG_WR_DATA_MASK_2, 0x0000);

	/* frame frequency control */
	ssd1289_set_reg(info, SSD1289_REG_FRAME_FREQUENCY,
			SSD1289_FRAME_FREQUENCY_80HZ);

	/* optimize data access speed */
	ssd1289_set_reg(info, SSD1289_REG_OPT_SPEED_1,
			SSD1289_OPT_SPEED1_VALUE);

	ssd1289_set_reg(info, SSD1289_REG_OPT_SPEED_2,
			SSD1289_OPT_SPEED2_VALUE);

	ssd1289_set_reg(info, SSD1289_REG_OPT_SPEED_3,
			SSD1289_OPT_SPEED3_VALUE);

	/* horizontal ram address position */
	ssd1289_set_reg(info, SSD1289_REG_H_RAM_ADR_POS,
			SSD1289_H_RAM_ADR_POS_VALUE);

	/* vertical ram address position */
	ssd1289_set_reg(info, SSD1289_REG_V_RAM_ADR_START, 0x0000);

	ssd1289_set_reg(info, SSD1289_REG_V_RAM_ADR_END,
			SSD1289_WINDOW_Y_SIZE);

	mdelay(20);

	/* set start address counter */
	ssd1289_set_reg(info, SSD1289_REG_GDDRAM_X_ADDR,
			SSD1289_GDDRAM_X_ADDR_VALUE);

	ssd1289_set_reg(info, SSD1289_REG_GDDRAM_Y_ADDR,
			SSD1289_GDDRAM_Y_ADDR_VALUE);

	for (i = 0; i < info->screen_size; i += 2)
		ssd1289_set_reg(info, SSD1289_REG_GDDRAM_DATA, 0);
}

static void fsl_ssd1289_disable_lcd(struct fb_info *info)
{
	ssd1289_set_reg(info, SSD1289_REG_DISPLAY_CTRL,
			SSD1289_DISPLAY_DISABLE);

	ssd1289_set_reg(info, SSD1289_REG_OSCILLATION, SSD1289_OSC_DISABLE);
}

static int ssd1289fbd(void *arg)
{
	struct fb_info *info = arg;
	unsigned short *buf_p;
	struct fsl_ssd1289_fb_info *fbinfo = info->par;

	while (!kthread_should_stop()) {
		set_current_state(TASK_INTERRUPTIBLE);

		if (fbinfo->openflag == 1) {
			buf_p = (unsigned short *)(info->screen_base);
			ssd1289_write_data(info, buf_p, info->screen_size);
		}
		schedule_timeout(HZ/25);
	}

	return 0;
}

static int fsl_ssd1289_check_var(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;
	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;

	if (var->xoffset < 0)
		var->xoffset = 0;

	if (var->yoffset < 0)
		var->yoffset = 0;

	if (var->xoffset + info->var.xres > info->var.xres_virtual)
		var->xoffset = info->var.xres_virtual - info->var.xres;

	if (var->yoffset + info->var.yres > info->var.yres_virtual)
		var->yoffset = info->var.yres_virtual - info->var.yres;

	switch (var->bits_per_pixel) {
	case 8:
		/* 8 bpp, 332 format */
		var->red.length		= 3;
		var->red.offset		= 5;
		var->red.msb_right	= 0;

		var->green.length	= 3;
		var->green.offset	= 2;
		var->green.msb_right	= 0;

		var->blue.length	= 2;
		var->blue.offset	= 0;
		var->blue.msb_right	= 0;

		var->transp.length	= 0;
		var->transp.offset	= 0;
		var->transp.msb_right	= 0;
		break;
	case 16:
		/* 16 bpp, 565 format */
		var->red.length		= 5;
		var->red.offset		= 11;
		var->red.msb_right	= 0;

		var->green.length	= 6;
		var->green.offset	= 5;
		var->green.msb_right	= 0;

		var->blue.length	= 5;
		var->blue.offset	= 0;
		var->blue.msb_right	= 0;

		var->transp.length	= 0;
		var->transp.offset	= 0;
		var->transp.msb_right	= 0;
		break;
	default:
		printk(KERN_ERR "Depth not supported: %u BPP\n",
			var->bits_per_pixel);
		return -EINVAL;
	}
	return 0;
}

static int fsl_ssd1289_set_par(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;

	switch (var->bits_per_pixel) {
	case 16:
		info->fix.visual = FB_VISUAL_TRUECOLOR;
		break;
	case 8:
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
		break;
	default:
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
		break;
	}

	return 0;
}

static inline __u32 CNVT_TOHW(__u32 val, __u32 width)
{
	return ((val<<width) + 0x7FFF - val)>>16;
}

static int fsl_ssd1289_setcolreg(unsigned regno,
			       unsigned red, unsigned green, unsigned blue,
			       unsigned transp, struct fb_info *info)
{
	int ret = 1;

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no matter what visual we are using.
	 */
	if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
				      7471 * blue) >> 16;
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;
			u32 value;

			red = CNVT_TOHW(red, info->var.red.length);
			green = CNVT_TOHW(green, info->var.green.length);
			blue = CNVT_TOHW(blue, info->var.blue.length);
			transp = CNVT_TOHW(transp, info->var.transp.length);

			value = (red << info->var.red.offset) |
				(green << info->var.green.offset) |
				(blue << info->var.blue.offset) |
				(transp << info->var.transp.offset);

			pal[regno] = value;
			ret = 0;
		}
		break;
	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		break;
	}
	return ret;
}

static int fsl_ssd1289_blank(int blank_mode, struct fb_info *info)
{
	if (blank_mode == FB_BLANK_POWERDOWN)
		fsl_ssd1289_disable_lcd(info);
	else
		fsl_ssd1289_enable_lcd(info);

	return 0;
}

static int fsl_ssd1289_open(struct fb_info *info, int user)
{
	struct fsl_ssd1289_fb_info *fbinfo = info->par;
	struct task_struct *task;
	int ret;

	if (fbinfo->openflag == 0) {
		memset(info->screen_base, 0, info->screen_size);
		fsl_ssd1289_enable_lcd(info);

		task = kthread_run(ssd1289fbd, info, "SSD1289 LCD");
		if (IS_ERR(task)) {
			ret = PTR_ERR(task);
			return ret;
		}
		fbinfo->task = task;
	}

	fbinfo->openflag = 1;
	return 0;
}

static int fsl_ssd1289_release(struct fb_info *info, int user)
{
	struct fsl_ssd1289_fb_info *fbinfo = info->par;

	fbinfo->openflag = 0;
	if (fbinfo->task) {
		struct task_struct *task = fbinfo->task;
		fbinfo->task = NULL;
		kthread_stop(task);
	}

	memset(info->screen_base, 0, info->screen_size);
	fsl_ssd1289_disable_lcd(info);

	return 0;
}

static struct fb_ops fsl_ssd1289_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= fsl_ssd1289_check_var,
	.fb_set_par	= fsl_ssd1289_set_par,
	.fb_setcolreg	= fsl_ssd1289_setcolreg,
	.fb_blank	= fsl_ssd1289_blank,
	.fb_open	= fsl_ssd1289_open,
	.fb_release	= fsl_ssd1289_release,
	.fb_copyarea	= cfb_copyarea,
	.fb_fillrect	= cfb_fillrect,
	.fb_imageblit	= cfb_imageblit,
};

static int fsl_ssd1289_map_video_memory(struct fb_info *info,
					struct platform_device *pdev)
{
	unsigned int map_size = info->fix.smem_len;
	struct resource *ctrl_res;
	struct resource *data_res;
	unsigned short signature;
	int ret;

	struct fsl_ssd1289_fb_info *fbinfo = info->par;

	ctrl_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!ctrl_res) {
		printk(KERN_ERR
			"%s: unable to platform_get_resource for ctrl_res\n",
			__func__);
		ret = -ENOENT;
		return ret;
	}
	fbinfo->ssd1289_reg.cmd =
		ioremap_nocache(ctrl_res->start, 2);

	data_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!data_res) {
		printk(KERN_ERR
			"%s: unable to platform_get_resource for data_res\n",
			__func__);
		ret = -ENOENT;
		goto out_item1;
	}
	fbinfo->ssd1289_reg.data =
		ioremap_nocache(data_res->start, 2);

	signature = in_be16(fbinfo->ssd1289_reg.cmd);
	if (signature != SSD1289_DEVICE_ID) {
		ret = -ENODEV;
		printk(KERN_ERR
			"%s: unknown signature 0x%04x\n", __func__, signature);
		goto out_item2;
	}

	info->screen_base = kmalloc(map_size, GFP_KERNEL);
	info->fix.smem_start = virt_to_phys(info->screen_base);
	info->screen_size = info->fix.smem_len;

	if (info->screen_base)
		memset(info->screen_base, 0, map_size);

	return info->screen_base ? 0 : -ENOMEM;
out_item2:
	iounmap(fbinfo->ssd1289_reg.cmd);
out_item1:
	iounmap(fbinfo->ssd1289_reg.data);
	return ret;
}

static inline void fsl_ssd1289_unmap_video_memory(struct fb_info *info)
{
	struct fsl_ssd1289_fb_info *fbinfo = info->par;

	iounmap(fbinfo->ssd1289_reg.cmd);
	iounmap(fbinfo->ssd1289_reg.data);
	kfree(info->screen_base);
}

static int fsl_ssd1289_probe(struct platform_device *pdev)
{
	struct fsl_ssd1289_fb_info *fbinfo;
	struct fb_info *info;
	struct fsl_ssd1289_fb_display *display;
	int ret;
	unsigned long smem_len;

	info = framebuffer_alloc(sizeof(struct fsl_ssd1289_fb_info),
			&pdev->dev);
	if (!info)
		return -ENOMEM;

	platform_set_drvdata(pdev, info);

	fbinfo = info->par;
	fbinfo->dev = &pdev->dev;
	display = pdev->dev.platform_data;

	fbinfo->openflag = 0;
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.type_aux = 0;
	info->fix.xpanstep = 0;
	info->fix.ypanstep = 0;
	info->fix.ywrapstep = 0;
	info->fix.accel = FB_ACCEL_NONE;

	info->var.nonstd = 0;
	info->var.activate = FB_ACTIVATE_NOW;
	info->var.accel_flags = 0;
	info->var.vmode = FB_VMODE_NONINTERLACED;

	info->fbops = &fsl_ssd1289_ops;
	info->flags = FBINFO_HWACCEL_IMAGEBLIT | FBINFO_HWACCEL_FILLRECT
				| FBINFO_HWACCEL_COPYAREA;
	info->pseudo_palette = &fbinfo->pseudo_palette;

	/* find maximum required memory size for display */
	smem_len = display->xres;
	smem_len *= display->yres;
	smem_len *= display->bpp;
	smem_len >>= 3;
	if (info->fix.smem_len < smem_len)
		info->fix.smem_len = smem_len;

	/* Intialize video memory */
	ret = fsl_ssd1289_map_video_memory(info, pdev);
	if (ret) {
		printk(KERN_ERR "Failed to allocate video RAM: %d\n", ret);
		ret = -ENOMEM;
		goto dealloc_fb;
	}

	info->var.xres = display->xres;
	info->var.yres = display->yres;
	info->var.bits_per_pixel = display->bpp;
	info->fix.line_length = (info->var.xres * info->var.bits_per_pixel) / 8;

	fsl_ssd1289_check_var(&info->var, info);

	ret = register_framebuffer(info);
	if (ret < 0) {
		printk(KERN_ERR "Failed to register framebuffer device: %d\n",
			ret);
		goto free_video_memory;
	}

	printk(KERN_INFO "fb: SSD1289 TFT LCD Framebuffer Driver\n");
	return 0;

free_video_memory:
	fsl_ssd1289_unmap_video_memory(info);
dealloc_fb:
	platform_set_drvdata(pdev, NULL);
	framebuffer_release(info);
	return ret;
}

static int fsl_ssd1289_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	unregister_framebuffer(info);
	fsl_ssd1289_unmap_video_memory(info);
	framebuffer_release(info);
	return 0;
}

#ifdef CONFIG_PM
static int fsl_ssd1289_suspend(struct platform_device *dev, pm_message_t state)
{
	struct fb_info *info = platform_get_drvdata(dev);

	/* enter into sleep mode */
	ssd1289_set_reg(info, SSD1289_REG_SLEEP_MODE,
			SSD1289_SLEEP_MODE_ENABLE);
	return 0;
}

static int fsl_ssd1289_resume(struct platform_device *dev)
{
	struct fb_info *info = platform_get_drvdata(dev);

	/* leave sleep mode */
	ssd1289_set_reg(info, SSD1289_REG_SLEEP_MODE,
			SSD1289_SLEEP_MODE_DISABLE);
	return 0;
}
#else
#define fsl_ssd1289_suspend NULL
#define fsl_ssd1289_resume NULL
#endif

static struct platform_driver fsl_ssd1289_driver = {
	.probe = fsl_ssd1289_probe,
	.remove = fsl_ssd1289_remove,
	.suspend = fsl_ssd1289_suspend,
	.resume = fsl_ssd1289_resume,
	.driver = {
		.name = "ssd1289",
		.owner = THIS_MODULE,
	},
};

static int __init fsl_ssd1289_init(void)
{
	return platform_driver_register(&fsl_ssd1289_driver);
}

static void __exit fsl_ssd1289_exit(void)
{
	return platform_driver_unregister(&fsl_ssd1289_driver);
}

module_init(fsl_ssd1289_init);
module_exit(fsl_ssd1289_exit);

MODULE_AUTHOR("Alison Wang <b18965@freescale.com>");
MODULE_DESCRIPTION("Freescale SSD1289 TFT LCD Framebuffer Driver");
MODULE_LICENSE("GPL");
