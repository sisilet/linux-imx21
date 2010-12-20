/*
 *  linux/drivers/video/imx21fb.c
 *
 *  Freescale i.MX21 Frame Buffer device driver
 *
 *  Copyright (C) 2004 Sascha Hauer, Pengutronix
 *   Based on acornfb.c Copyright (C) Russell King.
 *
 *  Modified by: Stephen Donecker (sdonecker@sbcglobal.net)
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 * Please direct your questions and comments on this driver to the following
 * email address:
 *
 *	linux-arm-kernel@lists.arm.linux.org.uk
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/cpufreq.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>
#include <asm/arch/imx21fb.h>

/* List current LCDC register contents */
#define DEBUG_LCDC 0 

/* Complain if VAR is out of range */
#define DEBUG_VAR 1

#include "imx21fb.h"

static struct imxfb_rgb def_rgb_16 = {
	.red	= { .offset = 8,  .length = 4, },
	.green	= { .offset = 4,  .length = 4, },
	.blue	= { .offset = 0,  .length = 4, },
	.transp = { .offset = 0,  .length = 0, },
};

static struct imxfb_rgb def_rgb_8 = {
	.red	= { .offset = 0,  .length = 8, },
	.green	= { .offset = 0,  .length = 8, },
	.blue	= { .offset = 0,  .length = 8, },
	.transp = { .offset = 0,  .length = 0, },
};

static int imxfb_activate_var(struct fb_var_screeninfo *var, struct fb_info *info);

static inline u_int chan_to_field(u_int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

#define LCDC_PALETTE(x) __REG2(MX2ADS_LCDC_BASE+0x800, (x)<<2)
static int
imxfb_setpalettereg(u_int regno, u_int red, u_int green, u_int blue,
		       u_int trans, struct fb_info *info)
{
	struct imxfb_info *fbi = info->par;
	u_int val, ret = 1;

#define CNVT_TOHW(val,width) ((((val)<<(width))+0x7FFF-(val))>>16)
	if (regno < fbi->palette_size) {
		val = (CNVT_TOHW(red, 4) << 8) |
		      (CNVT_TOHW(green,4) << 4) |
		      CNVT_TOHW(blue,  4);

		LCDC_PALETTE(regno) = val;
		ret = 0;
	}
	return ret;
}

static int
imxfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
		   u_int trans, struct fb_info *info)
{
	struct imxfb_info *fbi = info->par;
	unsigned int val;
	int ret = 1;

	/*
	 * If inverse mode was selected, invert all the colours
	 * rather than the register number.  The register number
	 * is what you poke into the framebuffer to produce the
	 * colour you requested.
	 */
	if (fbi->cmap_inverse) {
		red   = 0xffff - red;
		green = 0xffff - green;
		blue  = 0xffff - blue;
	}

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no mater what visual we are using.
	 */
	if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
					7471 * blue) >> 16;

	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/*
		 * 12 or 16-bit True Colour.  We encode the RGB value
		 * according to the RGB bitfield information.
		 */
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;

			val  = chan_to_field(red, &info->var.red);
			val |= chan_to_field(green, &info->var.green);
			val |= chan_to_field(blue, &info->var.blue);

			pal[regno] = val;
			ret = 0;
		}
		break;

	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		ret = imxfb_setpalettereg(regno, red, green, blue, trans, info);
		break;
	}

	return ret;
}

/*
 *  imxfb_check_var():
 *    Round up in the following order: bits_per_pixel, xres,
 *    yres, xres_virtual, yres_virtual, xoffset, yoffset, grayscale,
 *    bitfields, horizontal timing, vertical timing.
 */
static int
imxfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct imxfb_info *fbi = info->par;
	int rgbidx;

	if (var->xres < MIN_XRES)
		var->xres = MIN_XRES;
	if (var->yres < MIN_YRES)
		var->yres = MIN_YRES;
	if (var->xres > fbi->max_xres)
		var->xres = fbi->max_xres;
	if (var->yres > fbi->max_yres)
		var->yres = fbi->max_yres;
	var->xres_virtual = max(var->xres_virtual, var->xres);
	var->yres_virtual = max(var->yres_virtual, var->yres);

	pr_debug("var->bits_per_pixel=%d\n", var->bits_per_pixel);
	switch (var->bits_per_pixel) {
	case 16:
		rgbidx = RGB_16;
		break;
	case 8:
		rgbidx = RGB_8;
		break;
	default:
		rgbidx = RGB_16;
	}

	/*
	 * Copy the RGB parameters for this display
	 * from the machine specific parameters.
	 */
	var->red    = fbi->rgb[rgbidx]->red;
	var->green  = fbi->rgb[rgbidx]->green;
	var->blue   = fbi->rgb[rgbidx]->blue;
	var->transp = fbi->rgb[rgbidx]->transp;

	pr_debug("RGBT length = %d:%d:%d:%d\n",
		var->red.length, var->green.length, var->blue.length,
		var->transp.length);

	pr_debug("RGBT offset = %d:%d:%d:%d\n",
		var->red.offset, var->green.offset, var->blue.offset,
		var->transp.offset);

	return 0;
}

/*
 * imxfb_set_par():
 *	Set the user defined part of the display for the specified console
 */
static int imxfb_set_par(struct fb_info *info)
{
	struct imxfb_info *fbi = info->par;
	struct fb_var_screeninfo *var = &info->var;

	pr_debug("set_par\n");

	if (var->bits_per_pixel == 16)
		info->fix.visual = FB_VISUAL_TRUECOLOR;
	else if (!fbi->cmap_static)
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	else {
		/*
		 * Some people have weird ideas about wanting static
		 * pseudocolor maps.  I suspect their user space
		 * applications are broken.
		 */
		info->fix.visual = FB_VISUAL_STATIC_PSEUDOCOLOR;
	}

	info->fix.line_length = var->xres_virtual *
				  var->bits_per_pixel / 8;
	fbi->palette_size = var->bits_per_pixel == 8 ? 256 : 16;

	imxfb_activate_var(var, info);

	return 0;
}

static void imxfb_enable_controller(struct imxfb_info *fbi)
{
	pr_debug("Enabling LCD controller\n");

	/* Disable LCDC */
	CRM_PCCR0 &= ~(PCCR0_HCLK_LCDC_EN | PCCR0_PERCLK3_EN);

	/* Disable self-refresh */
	LCDC_LRMCR &= ~LRMCR_SELF_REF;
	
	/* Physical screen start address */
	LCDC_LSSAR = fbi->screen_dma;
	
	LCDC_LVPWR = LVPWR_VPW(fbi->max_xres * fbi->max_bpp / 8 / 4);

	LCDC_LPOR = LPOR_POS(0);   /* panning offset 0 (0 pixel offset) */

	/* Disable hardware cursor */
	LCDC_LCPR &= LCPR_CC_DISABLED;

	/* Burst length */
	LCDC_LDCR = LDCR_BURST | LDCR_HM(8) | LDCR_TM(4);
	
	/* Enable LCDC */
	CRM_PCCR0 |= PCCR0_HCLK_LCDC_EN | PCCR0_PERCLK3_EN;
	
	if(fbi->backlight_power)
		fbi->backlight_power(1);
	if(fbi->lcd_power)
		fbi->lcd_power(1);
}

static void imxfb_disable_controller(struct imxfb_info *fbi)
{
	pr_debug("Disabling LCD controller\n");

	if(fbi->backlight_power)
		fbi->backlight_power(0);
	if(fbi->lcd_power)
		fbi->lcd_power(0);

	/* Disable LCDC */
	CRM_PCCR0 &= ~(PCCR0_HCLK_LCDC_EN | PCCR0_PERCLK3_EN);
}

static int imxfb_blank(int blank, struct fb_info *info)
{
	struct imxfb_info *fbi = info->par;

	pr_debug("imxfb_blank: blank=%d\n", blank);

	switch (blank) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		imxfb_disable_controller(fbi);
		break;

	case FB_BLANK_UNBLANK:
		imxfb_enable_controller(fbi);
		break;
	}
	return 0;
}

static struct fb_ops imxfb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= imxfb_check_var,
	.fb_set_par	= imxfb_set_par,
	.fb_setcolreg	= imxfb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_blank	= imxfb_blank,
	.fb_cursor	= soft_cursor, /* FIXME: i.MX can do hardware cursor */
};

/*
 * imxfb_activate_var():
 *	Configures LCD Controller based on entries in var parameter.  Settings are
 *	only written to the controller if changes were made.
 */
static int imxfb_activate_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct imxfb_info *fbi = info->par;
	pr_debug("var: xres=%d hslen=%d lm=%d rm=%d\n",
		var->xres, var->hsync_len,
		var->left_margin, var->right_margin);
	pr_debug("var: yres=%d vslen=%d um=%d bm=%d\n",
		var->yres, var->vsync_len,
		var->upper_margin, var->lower_margin);

#if DEBUG_VAR
	if (var->xres < 16        || var->xres > 1024)
		printk(KERN_ERR "%s: invalid xres %d\n",
			info->fix.id, var->xres);
	if (var->hsync_len < 1    || var->hsync_len > 64)
		printk(KERN_ERR "%s: invalid hsync_len %d\n",
			info->fix.id, var->hsync_len);
	if (var->left_margin > 255)
		printk(KERN_ERR "%s: invalid left_margin %d\n",
			info->fix.id, var->left_margin);
	if (var->right_margin > 255)
		printk(KERN_ERR "%s: invalid right_margin %d\n",
			info->fix.id, var->right_margin);
	if (var->yres < 1 || var->yres > 511)
		printk(KERN_ERR "%s: invalid yres %d\n",
			info->fix.id, var->yres);
	if (var->vsync_len > 100)
		printk(KERN_ERR "%s: invalid vsync_len %d\n",
			info->fix.id, var->vsync_len);
	if (var->upper_margin > 63)
		printk(KERN_ERR "%s: invalid upper_margin %d\n",
			info->fix.id, var->upper_margin);
	if (var->lower_margin > 255)
		printk(KERN_ERR "%s: invalid lower_margin %d\n",
			info->fix.id, var->lower_margin);
#endif

	/* Write to the various LCDC registers. */
	LCDC_LHCR	= LHCR_H_WIDTH(var->hsync_len) |
	                  LHCR_H_WAIT_1(var->left_margin) |
			  LHCR_H_WAIT_2(var->right_margin);

	LCDC_LVCR	= LVCR_V_WIDTH(var->vsync_len) |
	                  LVCR_V_WAIT_1(var->upper_margin) |
			  LVCR_V_WAIT_2(var->lower_margin);

	LCDC_LSR	= LSR_XMAX(var->xres/16) | LSR_YMAX(var->yres);
	LCDC_LPCR	= fbi->lpcr;
	LCDC_LPCCR	= fbi->lpccr;
	LCDC_LSCR	= fbi->lscr;

	return 0;
}

static void imxfb_setup_gpio(struct imxfb_info *fbi)
{
	u32 display_type;

	display_type = (fbi->lpcr & (LPCR_TFT | LPCR_COLOR));
	
	/* Active matrix color display (TFT) */
	if (display_type == (LPCR_TFT | LPCR_COLOR))
	{
		switch(fbi->lpcr & LPCR_BPIX_MASK)
		{ 
			case LPCR_BPIX(2):	/* 4 bpp */
				imx21_gpio_mode(PA18_PF_LD12);
				imx21_gpio_mode(PA6_PF_LD0);
			case LPCR_BPIX(3):	/* 8 bpp */
				imx21_gpio_mode(PA18_PF_LD12);
				imx21_gpio_mode(PA6_PF_LD0);
			case LPCR_BPIX(6):	/* 18 bpp */
				imx21_gpio_mode(PA18_PF_LD12);
				imx21_gpio_mode(PA6_PF_LD0);
			case LPCR_BPIX(5):	/* 16 bpp */
				imx21_gpio_mode(PA19_PF_LD13);
				imx21_gpio_mode(PA13_PF_LD7);
				imx21_gpio_mode(PA12_PF_LD6);
				imx21_gpio_mode(PA7_PF_LD1);
			case LPCR_BPIX(4):	/* 12 bpp */
				imx21_gpio_mode(PA23_PF_LD17);
				imx21_gpio_mode(PA22_PF_LD16);
				imx21_gpio_mode(PA21_PF_LD15);
				imx21_gpio_mode(PA20_PF_LD14);
				imx21_gpio_mode(PA17_PF_LD11);
				imx21_gpio_mode(PA16_PF_LD10);
				imx21_gpio_mode(PA15_PF_LD9);
				imx21_gpio_mode(PA14_PF_LD8);
				imx21_gpio_mode(PA11_PF_LD5);
				imx21_gpio_mode(PA10_PF_LD4);
				imx21_gpio_mode(PA9_PF_LD3);
				imx21_gpio_mode(PA8_PF_LD2);
		}
	}

	/* Passive matrix color display (CSTN) */
	else if (display_type == LPCR_COLOR)
	{
		/* All CSTN panels use a 12-bit interface */
		imx21_gpio_mode(PA17_PF_LD11);
		imx21_gpio_mode(PA16_PF_LD10);
		imx21_gpio_mode(PA15_PF_LD9);
		imx21_gpio_mode(PA14_PF_LD8);
		imx21_gpio_mode(PA13_PF_LD7);
		imx21_gpio_mode(PA12_PF_LD6);
		imx21_gpio_mode(PA11_PF_LD5);
		imx21_gpio_mode(PA10_PF_LD4);
		imx21_gpio_mode(PA9_PF_LD3);
		imx21_gpio_mode(PA8_PF_LD2);
		imx21_gpio_mode(PA7_PF_LD1);
		imx21_gpio_mode(PA6_PF_LD0);
	}

	/* Passive matrix monochrome display */
	else
	{
		switch(fbi->lpcr & LPCR_PBSIZ_MASK)
		{ 
			case LPCR_PBSIZ(3):	/* 8-bit bus */
				imx21_gpio_mode(PA13_PF_LD7);
				imx21_gpio_mode(PA12_PF_LD6);
				imx21_gpio_mode(PA11_PF_LD5);
				imx21_gpio_mode(PA10_PF_LD4);
			case LPCR_PBSIZ(2):	/* 4-bit bus */
				imx21_gpio_mode(PA9_PF_LD3);
				imx21_gpio_mode(PA8_PF_LD2);
			case LPCR_PBSIZ(1):	/* 2-bit bus */
				imx21_gpio_mode(PA7_PF_LD1);
			case LPCR_PBSIZ(0):	/* 1-bit bus */
				imx21_gpio_mode(PA6_PF_LD0);
		}
	}

	/* Enable required GPIO */
	imx21_gpio_mode(PA5_PF_LSCLK);
	imx21_gpio_mode(PA30_PF_CONTRAST);
	imx21_gpio_mode(PA31_PF_OE_ACD);
	imx21_gpio_mode(PA29_PF_VSYNC);
	imx21_gpio_mode(PA28_PF_HSYNC);
	
	/* Enable Sharp 240x320 HR-TFT additional GPIO */ 
	if (fbi->lpcr & LPCR_SHARP)
	{
		imx21_gpio_mode(PA24_PF_REV);
		imx21_gpio_mode(PA25_PF_CLS);
		imx21_gpio_mode(PA26_PF_PS);
		imx21_gpio_mode(PA27_PF_SPL_SPR);
	}
}

#ifdef CONFIG_PM
/*
 * Power management hooks.  Note that we won't be called from IRQ context,
 * unlike the blank functions above, so we may sleep.
 */
static int imxfb_suspend(struct device *dev, u32 state, u32 level)
{
	struct imxfb_info *fbi = dev_get_drvdata(dev);
	pr_debug("%s\n",__FUNCTION__);

	if (level == SUSPEND_DISABLE || level == SUSPEND_POWER_DOWN)
		imxfb_disable_controller(fbi);
	return 0;
}

static int imxfb_resume(struct device *dev, u32 level)
{
	struct imxfb_info *fbi = dev_get_drvdata(dev);
	pr_debug("%s\n",__FUNCTION__);

	if (level == RESUME_ENABLE)
		imxfb_enable_controller(fbi);
	return 0;
}
#else
#define imxfb_suspend	NULL
#define imxfb_resume	NULL
#endif

static int __init imxfb_init_fbinfo(struct device *dev)
{
	struct imxfb_mach_info *inf = dev->platform_data;
	struct fb_info *info = dev_get_drvdata(dev);
	struct imxfb_info *fbi = info->par;

	pr_debug("%s\n",__FUNCTION__);

	info->pseudo_palette = kmalloc( sizeof(u32) * 16, GFP_KERNEL);
	if (!info->pseudo_palette)
		return -ENOMEM;

	memset(fbi, 0, sizeof(struct imxfb_info));
	fbi->dev = dev;

	strlcpy(info->fix.id, IMX_NAME, sizeof(info->fix.id));

	info->fix.type	= FB_TYPE_PACKED_PIXELS;
	info->fix.type_aux		= 0;
	info->fix.xpanstep		= 0;
	info->fix.ypanstep		= 0;
	info->fix.ywrapstep		= 0;
	info->fix.accel	= FB_ACCEL_NONE;

	info->var.nonstd		= 0;
	info->var.activate		= FB_ACTIVATE_NOW;
	info->var.height		= -1;
	info->var.width	= -1;
	info->var.accel_flags		= 0;
	info->var.vmode	= FB_VMODE_NONINTERLACED;

	info->fbops			= &imxfb_ops;
	info->flags			= FBINFO_FLAG_DEFAULT;
	info->pseudo_palette		= (fbi + 1);

	fbi->rgb[RGB_16]		= &def_rgb_16;
	fbi->rgb[RGB_8]			= &def_rgb_8;

	fbi->max_xres			= inf->xres;
	info->var.xres			= inf->xres;
	info->var.xres_virtual		= inf->xres;
	fbi->max_yres			= inf->yres;
	info->var.yres			= inf->yres;
	info->var.yres_virtual		= inf->yres;
	fbi->max_bpp			= inf->bpp;
	info->var.bits_per_pixel	= inf->bpp;
	info->var.pixclock		= inf->pixclock;
	info->var.hsync_len		= inf->hsync_len;
	info->var.left_margin		= inf->left_margin;
	info->var.right_margin		= inf->right_margin;
	info->var.vsync_len		= inf->vsync_len;
	info->var.upper_margin		= inf->upper_margin;
	info->var.lower_margin		= inf->lower_margin;
	info->var.sync			= inf->sync;
	info->var.grayscale		= inf->cmap_greyscale;
	fbi->cmap_inverse		= inf->cmap_inverse;
	fbi->lpcr			= inf->lpcr;
	fbi->lscr			= inf->lscr;
	fbi->lpccr			= inf->lpccr;
	fbi->lcd_power			= inf->lcd_power;
	fbi->backlight_power		= inf->backlight_power;
	info->fix.smem_len		= fbi->max_xres * fbi->max_yres *
					  fbi->max_bpp / 8;

	return 0;
}

/*
 *      Allocates the DRAM memory for the frame buffer.  This buffer is
 *	remapped into a non-cached, non-buffered, memory region to
 *      allow pixel writes to occur without flushing the cache.
 *      Once this area is remapped, all virtual memory access to the
 *      video memory should occur at the new region.
 */
static int __init imxfb_map_video_memory(struct fb_info *info)
{
	struct imxfb_info *fbi = info->par;

	fbi->map_size = PAGE_ALIGN(info->fix.smem_len);
	fbi->map_cpu = dma_alloc_writecombine(fbi->dev, fbi->map_size,
					&fbi->map_dma,GFP_KERNEL);
	
	if (fbi->map_cpu) {
		info->screen_base = fbi->map_cpu;
		fbi->screen_cpu = fbi->map_cpu;
		fbi->screen_dma = fbi->map_dma;
		info->fix.smem_start = fbi->screen_dma;
	}

	return fbi->map_cpu ? 0 : -ENOMEM;
}

static int __init imxfb_probe(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct imxfb_info *fbi;
	struct fb_info *info;
	struct imxfb_mach_info *inf;
	struct resource *res;
	int ret;

	printk("i.MX21 Framebuffer driver\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res)
		return -ENODEV;
	
	inf = dev->platform_data;
	if(!inf) {
		dev_err(dev,"No platform_data available\n");
		return -ENOMEM;
	}

	info = framebuffer_alloc(sizeof(struct imxfb_info), dev);
	if(!info)
		return -ENOMEM;

	fbi = info->par;

	dev_set_drvdata(dev, info);

	ret = imxfb_init_fbinfo(dev);
	if( ret < 0 )
		goto failed_init;

	res = request_mem_region(res->start, res->end - res->start + 1, "IMXFB");
	if (!res) {
		ret = -EBUSY;
		goto failed_regs;
	}

	if (!inf->fixed_screen_cpu) {
		ret = imxfb_map_video_memory(info);
		if (ret) {
			dev_err(dev, "Failed to allocate video RAM: %d\n", ret);
			ret = -ENOMEM;
			goto failed_map;
		}
	} else {
		/* Fixed framebuffer mapping enables location of the screen in eSRAM */
		fbi->map_cpu = inf->fixed_screen_cpu;
		fbi->map_dma = inf->fixed_screen_dma;
		info->screen_base = fbi->map_cpu;
		fbi->screen_cpu = fbi->map_cpu;
		fbi->screen_dma = fbi->map_dma;
		info->fix.smem_start = fbi->screen_dma;
	}

	/*
	 * This makes sure that our colour bitfield
	 * descriptors are correctly initialised.
	 */
	imxfb_check_var(&info->var, info);

	ret = fb_alloc_cmap(&info->cmap, 1<<info->var.bits_per_pixel, 0);
	if (ret < 0)
		goto failed_cmap;

	imxfb_setup_gpio(fbi);

	imxfb_set_par(info);
	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(dev, "failed to register framebuffer\n");
		goto failed_register;
	}

	imxfb_enable_controller(fbi);

#if DEBUG_LCDC
	/* Let's see what is currently in the LCDC registers. */
	printk("LCDC_LSSAR: 0x%08x\n",LCDC_LSSAR);
	printk("LCDC_LSR: 0x%08x\n",LCDC_LSR);
	printk("LCDC_LVPWR: 0x%08x\n",LCDC_LVPWR);
	printk("LCDC_LCPR: 0x%08x\n",LCDC_LCPR);
	printk("LCDC_LCWHBR: 0x%08x\n",LCDC_LCWHBR);
	printk("LCDC_LCCMR: 0x%08x\n",LCDC_LCCMR);
	printk("LCDC_LPCR: 0x%08x\n",LCDC_LPCR);
	printk("LCDC_LHCR: 0x%08x\n",LCDC_LHCR);
	printk("LCDC_LVCR: 0x%08x\n",LCDC_LVCR);
	printk("LCDC_LPOR: 0x%08x\n",LCDC_LPOR);
	printk("LCDC_LSCR: 0x%08x\n",LCDC_LSCR);
	printk("LCDC_LPCCR: 0x%08x\n",LCDC_LPCCR);
	printk("LCDC_LDCR: 0x%08x\n",LCDC_LDCR);
	printk("LCDC_LRMCR: 0x%08x\n",LCDC_LRMCR);
	printk("LCDC_LICR: 0x%08x\n",LCDC_LICR);
	printk("LCDC_LIER: 0x%08x\n",LCDC_LIER);
	printk("LCDC_LISR: 0x%08x\n",LCDC_LISR);
#endif
	
	return 0;

failed_register:
	fb_dealloc_cmap(&info->cmap);
failed_cmap:
	if (!inf->fixed_screen_cpu)
		dma_free_writecombine(dev,fbi->map_size,fbi->map_cpu,
		           fbi->map_dma);
failed_map:
	kfree(info->pseudo_palette);
failed_regs:
	release_mem_region(res->start, res->end - res->start);
failed_init:
	dev_set_drvdata(dev, NULL);
	framebuffer_release(info);
	return ret;
}

static int imxfb_remove(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct fb_info *info = dev_get_drvdata(dev);
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	/* disable LCD controller */
	CRM_PCCR0 &= ~(PCCR0_HCLK_LCDC_EN | PCCR0_PERCLK3_EN);
	
	unregister_framebuffer(info);

	fb_dealloc_cmap(&info->cmap);
	kfree(info->pseudo_palette);
	framebuffer_release(info);

	release_mem_region(res->start, res->end - res->start + 1);
	dev_set_drvdata(dev, NULL);

	return 0;
}

void  imxfb_shutdown(struct device * dev)
{
	/* disable LCD Controller */
	CRM_PCCR0 &= ~(PCCR0_HCLK_LCDC_EN | PCCR0_PERCLK3_EN);
}

static struct device_driver imxfb_driver = {
	.name		= "imx-fb",
	.bus		= &platform_bus_type,
	.probe		= imxfb_probe,
	.suspend	= imxfb_suspend,
	.resume		= imxfb_resume,
	.remove		= imxfb_remove,
	.shutdown	= imxfb_shutdown,
};

int __init imxfb_init(void)
{
	return driver_register(&imxfb_driver);
}

static void __exit imxfb_cleanup(void)
{
	driver_unregister(&imxfb_driver);
}

module_init(imxfb_init);
module_exit(imxfb_cleanup);

MODULE_DESCRIPTION("Motorola i.MX framebuffer driver");
MODULE_AUTHOR("Sascha Hauer, Pengutronix");
MODULE_LICENSE("GPL");
