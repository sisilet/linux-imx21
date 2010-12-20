#define DEBUG 1
/*
 *  linux/drivers/video/voyager_gxfb.c -- voyager panel frame buffer driver
 *
 *     Copyright (C) 2003 Renesas Technology Sales Co.,Ltd.
 *     Copyright (C) 2003 Atom Create Engineering Co.,Ltd.
 *     Author : Atom Create Engineering Co.,Ltd.
 *                   Kenichi Sakuma
 *     Copyright (C) 2005 Paul Mundt and Bill Gatliff <bgat@billgatliff.com>
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 *
 * 1.00
 *  - initial version (ks)
 * 1.01
 *  - Kernel 2.6 correspondence
 */
#include <linux/config.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <video/voyagergx_reg.h>
#include <video/voyager.h>

static unsigned long regbase;
static unsigned int pseudo_palette[16];

static struct fb_var_screeninfo voyafb_var __initdata = {
  .xres		= XRES,
  .yres		= YRES,
  .xres_virtual	= XRES,
  .yres_virtual	= YRES,
  .bits_per_pixel	= BPP,
  .red		= { 11, 5, 0 },
  .green		= {  5, 6, 0 },
  .blue		= {  0, 5, 0 },
  .height		= -1,
  .width		= -1,
  .vmode		= FB_VMODE_NONINTERLACED,
  .pixclock	= 10000,
  .left_margin	= 0,
  .right_margin	= 0,
  .upper_margin	= 0,
  .lower_margin	= 0,
  .hsync_len	= 0,
  .vsync_len	= 0,
};

static inline unsigned long voya_read_reg(unsigned int index)
{
  unsigned long ret = __raw_readl(regbase + index);
  return ret;
}

static inline void voya_write_reg(unsigned long data, unsigned int index)
{
  __raw_writel(data, regbase + index);
  wmb();
  return;
}

static int voyafb_check_var(struct fb_var_screeninfo *var,
			    struct fb_info *info)
{
  if (var->xres > XRES || var->yres > YRES
      || var->xres_virtual > XRES || var->yres_virtual > YRES
      || var->bits_per_pixel != BPP
      || var->nonstd
      || (var->vmode & FB_VMODE_MASK) != FB_VMODE_NONINTERLACED)
    return -EINVAL;

  var->xres = var->xres_virtual = XRES;
  var->yres = var->yres_virtual = YRES;

  return 0;
}

static int voyafb_set_par(struct fb_info *info)
{
  info->fix.line_length = XRES*2;
  info->fix.visual = FB_VISUAL_TRUECOLOR;

  info->var.bits_per_pixel = 16;
  info->var.red.offset = 11;
  info->var.green.offset = 5;
  info->var.blue.offset = 0;
  info->var.red.length = info->var.blue.length = 5;
  info->var.green.length = 6;
  return 0;
}

static int voyafb_blank(int blank, struct fb_info *info)
{
  return 1;
}

static int voyafb_setcolreg(unsigned regno, unsigned red, unsigned green,
			    unsigned blue, unsigned transp,
			    struct fb_info* info)
{
  red   >>= 11;
  green >>= 11;
  blue  >>= 10;

  if (regno < 16)
    ((u32 *)(info->pseudo_palette))[regno] = ((red & 31) << 6) |
      ((green & 31) << 11) |
      ((blue & 63));
  return 0;
}

static int voyafb_ioctl(struct inode* inode, struct file* file,
			unsigned int cmd, unsigned long arg,
			struct fb_info* info)
{
  static long *po;
  int *wk;

  if (cmd == VOYAGER_IOCTL_DEBUG_ADD) {
    po = (long *)arg;
    return 0;
  } else if (cmd == VOYAGER_IOCTL_DEBUG_GET) {
    wk = (int *)arg;
    *wk = *po;
    return 0;
  } else if (cmd == VOYAGER_IOCTL_DEBUG_PUT) {
    *po = arg;
    return 0;
  } else if (cmd == VOYAGER_IOCTL_ENABLE) {
    unsigned long l = voya_read_reg(PANEL_DISPLAY_CTRL);

    if (arg == 0) {
      l &= 0xfffffffb;
    } else {
      l |= 0x04;
    }

    voya_write_reg(l, PANEL_DISPLAY_CTRL);
    return 0;
  } else if (cmd == VOYAGER_IOCTL_TYPE) {
    unsigned long l = voya_read_reg(PANEL_DISPLAY_CTRL);

    l &= 0xfffffffc;
    l |= (arg & 0x03);

    voya_write_reg(l, PANEL_DISPLAY_CTRL);
    return 0;
  }

  return -EINVAL;
}

static void vsyncwait(int delay)
{
  int reg;
  
  while(delay-- > 0) {
    do {
      reg = voya_read_reg(CMD_INTPR_STATUS);
    } while(reg & 0x1000);
    
    do {
      reg = voya_read_reg(CMD_INTPR_STATUS);
    } while(!reg & 0x1000);
  }
}

static void voya_print_regs (void)
{
  pr_info("power_mode0_clock: %08lx\n", voya_read_reg(POWER_MODE0_CLOCK));
  pr_info("panel_fb_width: %08lx\n", voya_read_reg(PANEL_FB_WIDTH));
  pr_info("panel_window_width: %08lx\n", voya_read_reg(PANEL_WINDOW_WIDTH));
  pr_info("panel_window_height: %08lx\n", voya_read_reg(PANEL_WINDOW_HEIGHT));
  pr_info("panel_plane_tl: %08lx\n", voya_read_reg(PANEL_PLANE_TL));
  pr_info("panel_plane_br: %08lx\n", voya_read_reg(PANEL_PLANE_BR));
  pr_info("panel_horizontal_total: %08lx\n", voya_read_reg(PANEL_HORIZONTAL_TOTAL));
  pr_info("panel_vertical_total: %08lx\n", voya_read_reg(PANEL_VERTICAL_TOTAL));
  pr_info("panel_horizontal_sync: %08lx\n", voya_read_reg(PANEL_HORIZONTAL_SYNC));
  pr_info("panel_vertical_sync: %08lx\n", voya_read_reg(PANEL_VERTICAL_SYNC));
  pr_info("crt_horizontal_total: %08lx\n", voya_read_reg(CRT_HORIZONTAL_TOTAL));
  pr_info("crt_vertical_total: %08lx\n", voya_read_reg(CRT_VERTICAL_TOTAL));
  pr_info("crt_horizontal_sync: %08lx\n", voya_read_reg(CRT_HORIZONTAL_SYNC));
  pr_info("crt_vertical_sync: %08lx\n", voya_read_reg(CRT_VERTICAL_SYNC));
  pr_info("panel_display_ctrl: 0x%08lx\n", voya_read_reg(PANEL_DISPLAY_CTRL));
  pr_info("crt_display_ctrl:   0x%08lx\n", voya_read_reg(CRT_DISPLAY_CTRL));
}


static char cmdline_options[256] __initdata = "";

static void __init voya_hw_init(struct fb_info *info)
{
  int i;
  int id;
  unsigned long misc_ctrl;
  unsigned long crt_display_ctrl;
  unsigned long power_mode0_clock;

  unsigned long panel_display_ctrl;
  unsigned long panel_fb_address;
  unsigned long panel_horizontal_sync;
  unsigned long panel_vertical_sync;

  int width, height, bpp;

  unsigned long panel_fb_width;
  unsigned long panel_window_width;
  unsigned long panel_window_height;
  unsigned long panel_plane_tl;
  unsigned long panel_plane_br;
  unsigned long panel_horizontal_total;
  unsigned long panel_vertical_total;

  unsigned long crt_horizontal_sync;
  unsigned long crt_vertical_sync;
  unsigned long crt_horizontal_total;
  unsigned long crt_vertical_total;
  unsigned long crt_fb_width;


#warning TODO: find better DRAM settings
  voya_write_reg(0x80800, DRAM_CTRL);
  udelay(1000);
  udelay(1000);
  udelay(1000);

  /* probe for chip */
  id = voya_read_reg(DEVICE_ID) >> 16;
  if (0x0501 != id)
    {
      pr_info("did not detect sm501 chip: %x\n", id);
      goto out;
    }

  /* DAC enable */
  /* TODO: P4 has 24 MHz clock; P2 has 12 MHz.  Needs to be cmdline */
  misc_ctrl = voya_read_reg(MISC_CTRL);
#warning TODO: make pixel clock configurable on the cmdline
#if (CONFIG_MACH_CSB437TL_REV == 4)
#warning Note: selecting 24 MHz pixclk, appropriate for CSB437TL_P4 hardware.
  misc_ctrl |= (0 << 24); /* 1 == 12 MHz; 0 == 24 MHz */
#else
#warning Note: selecting 12 MHz pixclk, appropriate for CSB437TL_P2 hardware.
  misc_ctrl |= (1 << 24); /* 1 == 12 MHz; 0 == 24 MHz */
#endif
  misc_ctrl &= ~(1 << 12);
  voya_write_reg(misc_ctrl & 0xff7fffff, MISC_CTRL);

  /* Power Gate */
  voya_write_reg(voya_read_reg(POWER_MODE0_GATE) | 0x7f,
		 POWER_MODE0_GATE);
  voya_write_reg(voya_read_reg(POWER_MODE1_GATE) | 0x7f,
		 POWER_MODE1_GATE);

  /* Power Clock */
  //power_mode0_clock = 0x10021801;
  power_mode0_clock = 0x01090801;
  voya_write_reg(power_mode0_clock, POWER_MODE0_CLOCK);
  voya_write_reg(power_mode0_clock, POWER_MODE1_CLOCK);

  /* Power Mode Control */
  voya_write_reg(0, POWER_MODE_CTRL);

  /* Miscellaneous Timing */
  voya_write_reg(voya_read_reg(VIDEO_DISPLAY_CTRL) & 0xfffffffb,
		 VIDEO_DISPLAY_CTRL);
  voya_write_reg(voya_read_reg(VIDEO_ALPHA_DISPLAY_CTRL) & 0xfffffffb,
		 VIDEO_ALPHA_DISPLAY_CTRL);
  voya_write_reg(voya_read_reg(ALPHA_DISPLAY_CTRL) & 0xfffffffb,
		 ALPHA_DISPLAY_CTRL);
  voya_write_reg(voya_read_reg(PANEL_HWC_ADDRESS) & 0x7fffffff,
		 PANEL_HWC_ADDRESS);
  voya_write_reg(voya_read_reg(CRT_HWC_ADDRESS) & 0x7fffffff,
		 CRT_HWC_ADDRESS);

  crt_display_ctrl = voya_read_reg(CRT_DISPLAY_CTRL);
  crt_display_ctrl &= 0x3fffb;
  crt_display_ctrl = 0x10100;
  voya_write_reg(crt_display_ctrl, CRT_DISPLAY_CTRL);

  panel_fb_address = (unsigned long)info->fix.smem_start;
  voya_write_reg(panel_fb_address & 0x03fffff0, PANEL_FB_ADDRESS);
  voya_write_reg(panel_fb_address & 0x03fffff0, CRT_FB_ADDRESS);

#if 0
  /* 640x480-16 */
  panel_fb_width = 0x5000500;
  panel_window_width = 0x2800000;
  panel_window_height = 0x1e00000;

  panel_plane_tl = 0;
  panel_plane_br = 0x1df027f;
  panel_horizontal_total = 0x033f027f;
  panel_vertical_total = 0x20c01df;

  panel_horizontal_sync = 0x4a028b;
  panel_vertical_sync = 0x201e9;

#else

  /* 1024x768-16 */
  width = 1024; height = 768; bpp = 16;

  /* turn into "bytes per pixel" */
  bpp /= 8;

  panel_fb_width = (width * bpp) << 16 | (width * bpp);
  crt_fb_width = panel_fb_width;

  panel_window_width = width << 16;
  panel_window_height = height << 16;
  
  panel_plane_tl = 0;
  panel_plane_br = ((height - 1) << 16) | (width - 1);
  panel_horizontal_total = (((width - 1) + 241) << 16) | (width - 1);
  panel_vertical_total = (((height - 1) + 28) << 16) | (height - 1);
  panel_horizontal_sync = 0x00600400;
  panel_vertical_sync = 0x00030300;

  crt_horizontal_total = panel_horizontal_total;
  crt_vertical_total = panel_vertical_total;
  crt_horizontal_sync = panel_horizontal_sync;
  crt_vertical_sync = panel_vertical_sync;

#endif

  voya_write_reg(panel_fb_width & 0x0fff0fff, PANEL_FB_WIDTH);
  voya_write_reg(panel_window_width & 0x3ff03ff0, PANEL_WINDOW_WIDTH);
  voya_write_reg(panel_window_height & 0x0fff0fff, PANEL_WINDOW_HEIGHT);

  voya_write_reg(panel_plane_tl & 0x07ff07ff, PANEL_PLANE_TL);
  voya_write_reg(panel_plane_br & 0x07ff07ff,  PANEL_PLANE_BR);
  voya_write_reg(panel_horizontal_total & 0x0fff0fff, PANEL_HORIZONTAL_TOTAL);
  voya_write_reg(panel_vertical_total & 0x07ff07ff, PANEL_VERTICAL_TOTAL);

  voya_write_reg(panel_horizontal_sync & 0x00ff0fff, PANEL_HORIZONTAL_SYNC);
  voya_write_reg(panel_vertical_sync & 0x003f07ff, PANEL_VERTICAL_SYNC);

  voya_write_reg(crt_fb_width & 0x03ff03ff0, CRT_FB_WIDTH);
  voya_write_reg(crt_horizontal_total & 0x0fff0fff, CRT_HORIZONTAL_TOTAL);
  voya_write_reg(crt_vertical_total & 0x07ff07ff, CRT_VERTICAL_TOTAL);

  voya_write_reg(crt_horizontal_sync & 0x00ff0fff, CRT_HORIZONTAL_SYNC);
  voya_write_reg(crt_vertical_sync & 0x003f07ff, CRT_VERTICAL_SYNC);

  voya_write_reg(0, PANEL_PAN_CTRL);
  voya_write_reg(0, PANEL_COLOR_KEY);

  panel_display_ctrl = voya_read_reg(PANEL_DISPLAY_CTRL);
  panel_display_ctrl &= 0xffffcea8;
  panel_display_ctrl |= 0x3105;
  voya_write_reg(panel_display_ctrl, PANEL_DISPLAY_CTRL);
  vsyncwait(4);
  panel_display_ctrl |= 0x1000000;
  voya_write_reg(panel_display_ctrl, PANEL_DISPLAY_CTRL);
  vsyncwait(4);
  panel_display_ctrl |= 0x3000000;
  voya_write_reg(panel_display_ctrl, PANEL_DISPLAY_CTRL);
  vsyncwait(4);
  panel_display_ctrl |= 0xf000000;
  voya_write_reg(panel_display_ctrl, PANEL_DISPLAY_CTRL);

  for (i = 0; i < 256; i++)
    voya_write_reg((i << 16) + (i << 8) + i,
		   PANEL_PALETTE_RAM + (i * 4));
  
 out:
  voya_print_regs();

  return;
}

static struct fb_fix_screeninfo voyafb_fix __devinitdata = {
  .id		= "voyager_panel_fb",
  .type		= FB_TYPE_PACKED_PIXELS,
  .visual	= FB_VISUAL_TRUECOLOR,
  .accel	= FB_ACCEL_NONE,
  .line_length	= XRES * 2,
  .smem_len	= MAX_FRAMEBUFFER_MEM_SIZE,
};

static struct fb_ops voyafb_ops = {
  .owner	= THIS_MODULE,
  .fb_check_var	= voyafb_check_var,
  .fb_set_par	= voyafb_set_par,
  .fb_setcolreg	= voyafb_setcolreg,
  .fb_blank	= voyafb_blank,
  .fb_fillrect	= cfb_fillrect,
  .fb_copyarea	= cfb_copyarea,
  .fb_imageblit	= cfb_imageblit,
  .fb_cursor	= soft_cursor,
  .fb_ioctl     = voyafb_ioctl,
};

static int __devinit voyager_panel_fb_probe(struct device *dev)
{
  struct platform_device *pdev = to_platform_device(dev);
  struct fb_info *info;
  int ret;

  if (pdev->num_resources != 2) {
    dev_err(&pdev->dev, "invalid num_resources: %d\n",
	    pdev->num_resources);
    return -ENODEV;
  }

  /* 0: VRAM, 1: regs */
  if (pdev->resource[0].flags != IORESOURCE_MEM
      || pdev->resource[1].flags != IORESOURCE_MEM) {
    dev_err(&pdev->dev, "invalid resource type\n");
    return -EINVAL;
  }

  if (!request_mem_region(pdev->resource[0].start,
			  pdev->resource[0].end -
			  pdev->resource[0].start + 1,
			  "voyayer_panel_fb mem")) {
    dev_dbg(dev, "request_mem_region failed\n");
    return -EBUSY;
  }

  if (!request_mem_region(pdev->resource[1].start,
			  pdev->resource[1].end -
			  pdev->resource[1].start + 1,
			  "voyager_panel_fb regs")) {
    dev_dbg(dev, "request_mem_region failed\n");
    ret = -EBUSY;
    goto err_release0;
  }

  info = framebuffer_alloc(sizeof(u32) * 16, &pdev->dev);
  if (!info) {
    ret = -ENOMEM;
    goto err_release;
  }

  info->fix = voyafb_fix;

  info->fix.mmio_start = pdev->resource[1].start;
  info->fix.mmio_len = pdev->resource[1].end - pdev->resource[1].start + 1;

  regbase = (unsigned long)ioremap(info->fix.mmio_start, info->fix.mmio_len);
  if (!regbase) {
    ret = -EINVAL;
    goto err_release;
  }

  info->fix.smem_start = pdev->resource[0].start;
  info->fix.smem_len = pdev->resource[0].end - pdev->resource[0].start + 1;

  info->screen_base = ioremap(info->fix.smem_start, info->fix.smem_len);
  if (info->screen_base == NULL) {
    ret = -EINVAL;
    goto err_release;
  }

  info->var = voyafb_var;

  info->fbops = &voyafb_ops;
  info->flags = FBINFO_FLAG_DEFAULT;

  info->pseudo_palette = pseudo_palette;

  fb_alloc_cmap(&info->cmap, 16, 0);

  if (register_framebuffer(info) < 0) {
    printk(KERN_ERR "VOYAGER GX PANEL framebuffer failed to register\n");
    ret = -EINVAL;
    goto err_release;
  }

  printk(KERN_INFO "fb%d: VOYAGER GX_PANEL frame buffer (%dK RAM)\n",
	 info->node, info->fix.smem_len / 1024);

  voya_hw_init(info);

  return 0;

 err_release:
  if (info->screen_base)
    iounmap(info->screen_base);
  if (regbase)
    iounmap((void *)regbase);

  release_mem_region(pdev->resource[1].start,
		     pdev->resource[1].end - pdev->resource[1].start + 1);

 err_release0:
  release_mem_region(pdev->resource[0].start,
		     pdev->resource[0].end - pdev->resource[0].start + 1);

  return ret;
}

static void __exit voyafb_exit(void)
{
  voya_write_reg(voya_read_reg(PANEL_DISPLAY_CTRL) & 0xfffffffb,
		 PANEL_DISPLAY_CTRL);
}

static struct device_driver voyager_panel_fb_driver = {
  .name		= "voyager_panel_fb",
  .bus		= &platform_bus_type,
  .probe	= voyager_panel_fb_probe,
};

static int __init voyager_panel_fb_init(void)
{
  int ret;
  char *options = NULL;

  if (fb_get_options("voyager_panel_fb", &options))
    return -ENODEV;

  strlcpy(cmdline_options, options, sizeof(cmdline_options));

  ret = driver_register(&voyager_panel_fb_driver);
  if (ret)
    printk(KERN_ERR "%s: returning %d\n", __FUNCTION__, ret);
  else
    printk(KERN_ERR "voyager_gxfb "
	   "(c) Paul Mundt and Bill Gatliff <bgat@billgatliff.com>\n");
  return ret;
}

static void __exit voyager_panel_fb_exit(void)
{
  driver_unregister(&voyager_panel_fb_driver);
}

module_init(voyager_panel_fb_init);
module_exit(voyager_panel_fb_exit);

