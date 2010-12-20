/* drivers/video/s1d13xxxfb.c
 *
 * (c) 2004 Simtec Electronics
 * (c) 2005 Thibaut VARENE <varenet@parisc-linux.org>
 *
 * Driver for Epson S1D13xxx series framebuffer chips
 *
 * Adapted from
 *  linux/drivers/video/skeletonfb.c
 *  linux/drivers/video/epson1355fb.c
 *  linux/drivers/video/epson/s1d13xxxfb.c (2.4 driver by Epson)
 *
 * Note, currently only tested on S1D13806 with 16bit CRT.
 * As such, this driver might still contain some hardcoded bits relating to
 * S1D13806.
 * Making it work on other S1D13XXX chips should merely be a matter of adding
 * a few switch()s, some missing glue here and there maybe, and split header
 * files.
 *
 * TODO: - handle dual screen display (CRT and LCD at the same time).
 *	 - check_var(), mode change, etc.
 *	 - PM untested.
 *	 - Accelerated interfaces.
 *	 - Probably not SMP safe :)
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/string.h>

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/fb.h>

#include <asm/io.h>

#include <video/s1d13xxxfb.h>

#define PFX "s1d13xxxfb: "


#undef DEBUG
#if defined(DEBUG)
#define dbg(format, arg...) \
        printk(KERN_ERR "%s(%s:%d): " format "\n" , __FUNCTION__, __FILE__, __LINE__, ## arg)
#else
#define dbg(format, arg...)
#endif

#define err(format, arg...) \
        printk(KERN_ERR "%s(%s:%d): " format "\n" , __FUNCTION__, __FILE__, __LINE__, ## arg)
#define info(format, arg...) printk(KERN_INFO PFX ": " format "\n" , ## arg)


#define FB_PARAMETER_SIZE 256
static char s1d13xxxfb_options[FB_PARAMETER_SIZE] __initdata = "";


/*
 * Here we define the default struct fb_fix_screeninfo
 */
static struct fb_fix_screeninfo __devinitdata s1d13xxxfb_fix = {
  .id		= S1D_DEVICENAME,
  .type		= FB_TYPE_PACKED_PIXELS,
  .visual	= FB_VISUAL_PSEUDOCOLOR,
  .xpanstep	= 0,
  .ypanstep	= 1,
  .ywrapstep	= 0,
  .accel	= FB_ACCEL_NONE,
};

static inline u8
s1d13xxxfb_readreg(struct s1d13xxxfb_par *par, u16 regno)
{
  u16 rval = readw(par->regs + (regno & ~1));
  u16 value;

  if (regno & 1) value = rval >> 8;
  else value = rval & 0xff;

  return (u8)value;
}

static inline void
s1d13xxxfb_writereg(struct s1d13xxxfb_par *par, u16 regno, u16 value)
{
  u16 rval = readw(par->regs + (regno & ~1));

  if (regno & 1) rval = ((value << 8) | (rval & 0xff));
  else rval = (rval & 0xff00) | (value & 0xff);

  writew(rval, par->regs + (regno & ~1));
}

static void
s1d13xxxfb_display_reg(struct s1d13xxxfb_par *par, u16 regno)
{
  u16 regval;
  regval = s1d13xxxfb_readreg(par, regno);
  printk(KERN_ERR PFX " %02x: %02x\n", regno, regval);
  return;
}

static void
s1d13xxxfb_display_regs(const char *prompt, struct s1d13xxxfb_par *par)
{
  int r;
  
  if (prompt)
    printk(KERN_ERR PFX "%s\n", prompt);

  for (r = S1DREG_REV_CODE; r <= S1DREG_COM_DISP_MODE; r++)
    {
      switch (r) {
      case S1DREG_REV_CODE:
      case S1DREG_MISC:
      case S1DREG_CNF_STATUS:
      case S1DREG_CLK_CNF:
      case S1DREG_LCD_CLK_CNF:
      case S1DREG_CRT_CLK_CNF:
      case S1DREG_MPLUG_CLK_CNF:
      case S1DREG_CPU2MEM_WST_SEL:
      case S1DREG_MEM_CNF:
      case S1DREG_SDRAM_REF_RATE:
      case S1DREG_SDRAM_TC0:
      case S1DREG_SDRAM_TC1:
      case S1DREG_PANEL_TYPE:
      case S1DREG_MOD_RATE:
      case S1DREG_LCD_DISP_HWIDTH:
      case S1DREG_LCD_NDISP_HPER:
      case S1DREG_LCD_DISP_VHEIGHT0:
      case S1DREG_LCD_DISP_VHEIGHT1:
      case S1DREG_LCD_DISP_START0:
      case S1DREG_LCD_DISP_START1:
      case S1DREG_LCD_DISP_START2:
      case S1DREG_LCD_NDISP_VPER:
      case S1DREG_TFT_FPFRAME_START:
      case S1DREG_TFT_FPFRAME_PWIDTH:
      case S1DREG_LCD_DISP_MODE:
      case S1DREG_LCD_MISC:
      case S1DREG_TFT_FPLINE_START:
      case S1DREG_TFT_FPLINE_PWIDTH:
      case S1DREG_LCD_MEM_OFF0:
      case S1DREG_LCD_MEM_OFF1:
      case S1DREG_LCD_PIX_PAN:
      case S1DREG_LCD_DISP_FIFO_HTC:
      case S1DREG_LCD_DISP_FIFO_LTC:

        s1d13xxxfb_display_reg(par, r);
        break;
      }
    }
  return;
}

static inline void
s1d13xxxfb_runinit(struct s1d13xxxfb_par *par,
                   const struct s1d13xxxfb_regval *initregs,
                   const unsigned int size)
{
  int i;
   
  for (i = 0; i < size; i++) {
    if ((initregs[i].addr == S1DREG_DELAYOFF) ||
        (initregs[i].addr == S1DREG_DELAYON))
      mdelay((int)initregs[i].value);
    else {
      s1d13xxxfb_writereg(par, initregs[i].addr, initregs[i].value);
    }
  }

  /* make sure the hardware can cope with us */
  mdelay(1);

#if defined(DEBUG)
  s1d13xxxfb_display_regs(NULL, par);
#endif
}

static inline void
lcd_enable(struct s1d13xxxfb_par *par, int enable)
{
  u8 mode = s1d13xxxfb_readreg(par, S1DREG_COM_DISP_MODE);

  if (enable)
    mode |= 0x01;
  else
    mode &= ~0x01;

  s1d13xxxfb_writereg(par, S1DREG_COM_DISP_MODE, mode);
}

static inline void
crt_enable(struct s1d13xxxfb_par *par, int enable)
{
  u8 mode = s1d13xxxfb_readreg(par, S1DREG_COM_DISP_MODE);

  if (enable)
    mode |= 0x02;
  else
    mode &= ~0x02;

  s1d13xxxfb_writereg(par, S1DREG_COM_DISP_MODE, mode);
}

/* framebuffer control routines */

static inline void
s1d13xxxfb_setup_pseudocolour(struct fb_info *info)
{
  info->fix.visual = FB_VISUAL_PSEUDOCOLOR;

  info->var.red.length = 4;
  info->var.green.length = 4;
  info->var.blue.length = 4;
}

static inline void
s1d13xxxfb_setup_truecolour(struct fb_info *info)
{
  info->fix.visual = FB_VISUAL_TRUECOLOR;
  info->var.bits_per_pixel = 16;

  info->var.red.length = 5;
  info->var.red.offset = 11;

  info->var.green.length = 6;
  info->var.green.offset = 5;

  info->var.blue.length = 5;
  info->var.blue.offset = 0;
}

static int
s1d13xxxfb_set_lcd_par(struct fb_info *info)
{
  /* TODO: figure out LCD settings from fb_info */
  return 0;
}

/**
 *      s1d13xxxfb_set_par - Alters the hardware state.
 *      @info: frame buffer structure
 *
 *	Using the fb_var_screeninfo in fb_info we set the depth of the
 *	framebuffer. This function alters the par AND the
 *	fb_fix_screeninfo stored in fb_info. It doesn't not alter var in
 *	fb_info since we are using that data. This means we depend on the
 *	data in var inside fb_info to be supported by the hardware.
 *	xxxfb_check_var is always called before xxxfb_set_par to ensure this.
 *
 *	XXX TODO: write proper s1d13xxxfb_check_var(), without which that
 *	function is quite useless.
 */
static int
s1d13xxxfb_set_par(struct fb_info *info)
{
  struct s1d13xxxfb_par *s1dfb = info->par;
  unsigned int val;
  int is_lcd = (s1dfb->display & 1);
  int retval = 0;

  if (is_lcd)
    val = s1d13xxxfb_readreg(s1dfb, S1DREG_LCD_DISP_MODE);   /* read colour control */
  else
    val = s1d13xxxfb_readreg(s1dfb, S1DREG_CRT_DISP_MODE);   /* read colour control */

  val &= ~0x07;

  switch (info->var.bits_per_pixel) {
  case 4:
    dbg("pseudo colour 4");
    s1d13xxxfb_setup_pseudocolour(info);
    val |= 2;
    break;
  case 8:
    dbg("pseudo colour 8");
    s1d13xxxfb_setup_pseudocolour(info);
    val |= 3;
    break;
  case 16:
    dbg("true colour");
    s1d13xxxfb_setup_truecolour(info);
    val |= 5;
    break;
          
  default:
    dbg("bpp not supported!");
    return -EINVAL;
  }
        
  if (is_lcd)
    retval = s1d13xxxfb_set_lcd_par(info);

  if (is_lcd)
    s1d13xxxfb_writereg(s1dfb, S1DREG_LCD_DISP_MODE, val);
  else
    s1d13xxxfb_writereg(s1dfb, S1DREG_CRT_DISP_MODE, val);
        
  info->fix.line_length  = info->var.xres * info->var.bits_per_pixel;
  info->fix.line_length /= 8;
        
  return retval;
}

/**
 *  	s1d13xxxfb_setcolreg - sets a color register.
 *      @regno: Which register in the CLUT we are programming
 *      @red: The red value which can be up to 16 bits wide
 *	@green: The green value which can be up to 16 bits wide
 *	@blue:  The blue value which can be up to 16 bits wide.
 *	@transp: If supported the alpha value which can be up to 16 bits wide.
 *      @info: frame buffer info structure
 *
 *	Returns negative errno on error, or zero on success.
 */
static int
s1d13xxxfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
                     u_int transp, struct fb_info *info)
{
  struct s1d13xxxfb_par *s1dfb = info->par;
  unsigned int pseudo_val;

  if (regno >= S1D_PALETTE_SIZE)
    return -EINVAL;

#if 0
  dbg("s1d13xxxfb_setcolreg: %d: rgb=%d,%d,%d, tr=%d",
      regno, red, green, blue, transp);
#endif

  if (info->var.grayscale)
    red = green = blue = (19595*red + 38470*green + 7471*blue) >> 16;

  switch (info->fix.visual) {
  case FB_VISUAL_TRUECOLOR:
    if (regno >= 16)
      return -EINVAL;

    /* deal with creating pseudo-palette entries */

    pseudo_val  = (red   >> 11) << info->var.red.offset;
    pseudo_val |= (green >> 10) << info->var.green.offset;
    pseudo_val |= (blue  >> 11) << info->var.blue.offset;

#if 0
    dbg("s1d13xxxfb_setcolreg: pseudo %d, val %08x",
        regno, pseudo_val);
#endif

    ((u32 *)info->pseudo_palette)[regno] = pseudo_val;

    break;
  case FB_VISUAL_PSEUDOCOLOR:
    s1d13xxxfb_writereg(s1dfb, S1DREG_LKUP_ADDR, regno);
    s1d13xxxfb_writereg(s1dfb, S1DREG_LKUP_DATA, red);
    s1d13xxxfb_writereg(s1dfb, S1DREG_LKUP_DATA, green);
    s1d13xxxfb_writereg(s1dfb, S1DREG_LKUP_DATA, blue);

    break;
  default:
    return -ENOSYS;
  }

  return 0;
}

/**
 *      s1d13xxxfb_blank - blanks the display.
 *      @blank_mode: the blank mode we want.
 *      @info: frame buffer structure that represents a single frame buffer
 *
 *      Blank the screen if blank_mode != 0, else unblank. Return 0 if
 *      blanking succeeded, != 0 if un-/blanking failed due to e.g. a
 *      video mode which doesn't support it. Implements VESA suspend
 *      and powerdown modes on hardware that supports disabling hsync/vsync:
 *      blank_mode == 2: suspend vsync
 *      blank_mode == 3: suspend hsync
 *      blank_mode == 4: powerdown
 *
 *      Returns negative errno on error, or zero on success.
 */
static int
s1d13xxxfb_blank(int blank_mode, struct fb_info *info)
{
  struct s1d13xxxfb_par *par = info->par;

  dbg("s1d13xxxfb_blank: blank=%d, info=%p", blank_mode, info);

  switch (blank_mode) {
  case FB_BLANK_UNBLANK:
  case FB_BLANK_NORMAL:
    if ((par->display & 0x01) != 0)
      lcd_enable(par, 1);
    if ((par->display & 0x02) != 0)
      crt_enable(par, 1);
    break;
  case FB_BLANK_VSYNC_SUSPEND:
  case FB_BLANK_HSYNC_SUSPEND:
    break;
  case FB_BLANK_POWERDOWN:
    lcd_enable(par, 0);
    crt_enable(par, 0);
    break;
  default:
    return -EINVAL;
  }

  /* let fbcon do a soft blank for us */
  return ((blank_mode == FB_BLANK_NORMAL) ? 1 : 0);
}

/**
 *      s1d13xxxfb_pan_display - Pans the display.
 *      @var: frame buffer variable screen structure
 *      @info: frame buffer structure that represents a single frame buffer
 *
 *	Pan (or wrap, depending on the `vmode' field) the display using the
 *  	`yoffset' field of the `var' structure (`xoffset'  not yet supported).
 *  	If the values don't fit, return -EINVAL.
 *
 *      Returns negative errno on error, or zero on success.
 */
static int
s1d13xxxfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
  struct s1d13xxxfb_par *par = info->par;
  u32 start;

  if (var->xoffset != 0)	/* not yet ... */
    return -EINVAL;

  if (var->yoffset + info->var.yres > info->var.yres_virtual)
    return -EINVAL;

  start = (info->fix.line_length >> 1) * var->yoffset;

  if ((par->display & 0x01)) {
    /* LCD */
    s1d13xxxfb_writereg(par, S1DREG_LCD_DISP_START0, (start & 0xff));
    s1d13xxxfb_writereg(par, S1DREG_LCD_DISP_START1, ((start >> 8) & 0xff));
    s1d13xxxfb_writereg(par, S1DREG_LCD_DISP_START2, ((start >> 16) & 0x0f));
  } else {
    /* CRT */
    s1d13xxxfb_writereg(par, S1DREG_CRT_DISP_START0, (start & 0xff));
    s1d13xxxfb_writereg(par, S1DREG_CRT_DISP_START1, ((start >> 8) & 0xff));
    s1d13xxxfb_writereg(par, S1DREG_CRT_DISP_START2, ((start >> 16) & 0x0f));
  }

  return 0;
}


/* framebuffer information structures */

static struct fb_ops s1d13xxxfb_fbops = {
  .owner	= THIS_MODULE,
  .fb_set_par	= s1d13xxxfb_set_par,
  .fb_setcolreg	= s1d13xxxfb_setcolreg,
  .fb_blank	= s1d13xxxfb_blank,

  .fb_pan_display = s1d13xxxfb_pan_display,

  /* to be replaced by any acceleration we can */
  .fb_fillrect	= cfb_fillrect,
  .fb_copyarea	= cfb_copyarea,
  .fb_imageblit	= cfb_imageblit,
  .fb_cursor	= soft_cursor
};

static int s1d13xxxfb_width_tab[2][4] __devinitdata = {
  {4, 8, 16, -1},
  {9, 12, 18, -1},
};

/**
 *      s1d13xxxfb_fetch_hw_state - Configure the framebuffer according to
 *	hardware setup.
 *      @info: frame buffer structure
 *
 *	We setup the framebuffer structures according to the current
 *	hardware setup. On some machines, the BIOS will have filled
 *	the chip registers with such info, on others, these values will
 *	have been written in some init procedure. In any case, the
 *	software values needs to match the hardware ones. This is what
 *	this function ensures.
 *
 *	Note: some of the hardcoded values here might need some love to
 *	work on various chips, and might need to no longer be hardcoded.
 */
static void __devinit
s1d13xxxfb_fetch_hw_state(struct fb_info *info)
{
  struct fb_var_screeninfo *var = &info->var;
  struct fb_fix_screeninfo *fix = &info->fix;
  struct s1d13xxxfb_par *par = info->par;
  u8 panel, display;
  u16 offset;
  u32 xres, yres;
  u32 xres_virtual, yres_virtual;
  int bpp, lcd_bpp;
  int is_color, is_dual, is_tft;
  int lcd_enabled, crt_enabled;

  fix->type = FB_TYPE_PACKED_PIXELS;

  /* general info */
  par->display = s1d13xxxfb_readreg(par, S1DREG_COM_DISP_MODE);
  crt_enabled = (par->display & 0x02) != 0;
  lcd_enabled = (par->display & 0x01) != 0;

  if (lcd_enabled && crt_enabled)
    printk(KERN_WARNING PFX "Warning: LCD and CRT detected, using LCD\n");

  if (lcd_enabled)
    display = s1d13xxxfb_readreg(par, S1DREG_LCD_DISP_MODE);
  else	/* CRT */
    display = s1d13xxxfb_readreg(par, S1DREG_CRT_DISP_MODE);

  bpp = display & 0x07;

  switch (bpp) {
  case 2:	/* 4 bpp */
  case 3:	/* 8 bpp */
    var->bits_per_pixel = 8;
    var->red.offset = var->green.offset = var->blue.offset = 0;
    var->red.length = var->green.length = var->blue.length = 8;
    break;
  case 5:	/* 16 bpp */
    s1d13xxxfb_setup_truecolour(info);
    break;
  default:
    dbg("bpp: %i", bpp);
  }
  fb_alloc_cmap(&info->cmap, 256, 0);

  /* LCD info */
  panel = s1d13xxxfb_readreg(par, S1DREG_PANEL_TYPE);
  is_color = (panel & 0x04) != 0;
  is_dual = (panel & 0x02) != 0;
  is_tft = (panel & 0x01) != 0;
  lcd_bpp = s1d13xxxfb_width_tab[is_tft][(panel >> 4) & 3];

  if (lcd_enabled) {
    xres = (s1d13xxxfb_readreg(par, S1DREG_LCD_DISP_HWIDTH) + 1) * 8;
    yres = (s1d13xxxfb_readreg(par, S1DREG_LCD_DISP_VHEIGHT0) +
            ((s1d13xxxfb_readreg(par, S1DREG_LCD_DISP_VHEIGHT1) & 0x03) << 8) + 1);

    offset = (s1d13xxxfb_readreg(par, S1DREG_LCD_MEM_OFF0) +
              ((s1d13xxxfb_readreg(par, S1DREG_LCD_MEM_OFF1) & 0x7) << 8));
  } else { /* crt */
    xres = (s1d13xxxfb_readreg(par, S1DREG_CRT_DISP_HWIDTH) + 1) * 8;
    yres = (s1d13xxxfb_readreg(par, S1DREG_CRT_DISP_VHEIGHT0) +
            ((s1d13xxxfb_readreg(par, S1DREG_CRT_DISP_VHEIGHT1) & 0x03) << 8) + 1);

    offset = (s1d13xxxfb_readreg(par, S1DREG_CRT_MEM_OFF0) +
              ((s1d13xxxfb_readreg(par, S1DREG_CRT_MEM_OFF1) & 0x7) << 8));
  }

#if 0
  xres_virtual = offset * 16 / var->bits_per_pixel;
  yres_virtual = fix->smem_len / (offset * 2);
#else
#warning TODO: turn the above code back on
  xres_virtual = xres;
  yres_virtual = yres;
#endif

  var->xres		= xres;
  var->yres		= yres;
  var->xres_virtual	= xres_virtual;
  var->yres_virtual	= yres_virtual;
  var->xoffset		= var->yoffset = 0;

  fix->line_length	= offset * 2;

  var->grayscale	= !is_color;

  var->activate		= FB_ACTIVATE_NOW;

  info( "bpp=%d, lcd_bpp=%d, "
        "crt_enabled=%d, lcd_enabled=%d",
        var->bits_per_pixel, lcd_bpp, crt_enabled, lcd_enabled);
  info( "xres=%d, yres=%d, offset=%d, vxres=%d, vyres=%d "
        "is_color=%d, is_dual=%d, is_tft=%d",
        xres, yres, offset, xres_virtual, yres_virtual, is_color, is_dual, is_tft);

#if defined(DEBUG)
  s1d13xxxfb_display_regs("register values:", par);
#endif
}

static int __devexit
s1d13xxxfb_remove(struct device *dev)
{
  struct fb_info *info = dev_get_drvdata(dev);
  struct platform_device *pdev = to_platform_device(dev);
  struct s1d13xxxfb_par *par = NULL;

  if (info) {
    par = info->par;
    if (par && par->regs) {
      /* disable output & enable powersave */
      s1d13xxxfb_writereg(par, S1DREG_COM_DISP_MODE, 0x00);
      s1d13xxxfb_writereg(par, S1DREG_PS_CNF, 0x11);
      iounmap(par->regs);
    }

    fb_dealloc_cmap(&info->cmap);

    if (info->screen_base)
      iounmap(info->screen_base);

    framebuffer_release(info);
  }

  release_mem_region(pdev->resource[0].start,
                     pdev->resource[0].end - pdev->resource[0].start +1);
  release_mem_region(pdev->resource[1].start,
                     pdev->resource[1].end - pdev->resource[1].start +1);
  return 0;
}

static int __init s1d13xxxfb_parse_options(struct device *dev, char *options)
{
  struct fb_info *info = dev_get_drvdata(dev);

  if (!info) 
    {
      info("no parameters");
      return 0;
    }

  /* TODO: actually parse options */
  return 0;
}

static int __devinit
s1d13xxxfb_alloc_resources(struct device *dev)
{
  struct platform_device *pdev = to_platform_device(dev);
  int ret = 0;

  if (pdev->num_resources != 2) {
    dev_err(&pdev->dev, "invalid num_resources: %i",
            pdev->num_resources);
    ret = -ENODEV;
    goto bail;
  }

  /* resource[0] is VRAM, resource[1] is registers */
  if (pdev->resource[0].flags != IORESOURCE_MEM
      || pdev->resource[1].flags != IORESOURCE_MEM) {
    dev_err(&pdev->dev, "invalid resource type");
    ret = -ENODEV;
    goto bail;
  }

  if (!request_mem_region(pdev->resource[0].start,
                          pdev->resource[0].end - pdev->resource[0].start +1,
                          S1D_DEVICENAME " vram")) {
    dev_dbg(dev, "request_mem_region failed");
    ret = -EBUSY;
    goto bail;
  }

  if (!request_mem_region(pdev->resource[1].start,
                          pdev->resource[1].end - pdev->resource[1].start +1,
                          S1D_DEVICENAME " regs")) {
    dev_dbg(dev, "request_mem_region failed");
    ret = -EBUSY;
    goto bail;
  }

 bail:
  return ret;
}

static int __devinit
s1d13xxxfb_probe(struct device *dev)
{
  struct platform_device *pdev = to_platform_device(dev);
  struct s1d13xxxfb_par *default_par;
  struct fb_info *info;
  struct s1d13xxxfb_pdata *pdata = NULL;
  int ret = 0;
  u8 revision;

  s1d13xxxfb_parse_options(dev, s1d13xxxfb_options);

  if (dev->platform_data)
    pdata = dev->platform_data;

  if (pdata && pdata->platform_init_video)
    pdata->platform_init_video();

  if (s1d13xxxfb_alloc_resources(dev)) {
      ret = -ENODEV;
      goto bail;
    }

  info = framebuffer_alloc(sizeof(struct s1d13xxxfb_par)
                           + sizeof(u32) * 256, &pdev->dev);
  if (!info) {
    ret = -ENOMEM;
    goto bail;
  }

  default_par = info->par;
  default_par->regs = ioremap_nocache(pdev->resource[1].start,
                                      pdev->resource[1].end - pdev->resource[1].start + 1);
  if (!default_par->regs) {
    printk(KERN_ERR PFX "unable to map registers\n");
    ret = -ENOMEM;
    goto bail;
  }
  info->pseudo_palette = default_par->pseudo_palette;

  info->screen_base = ioremap_nocache(pdev->resource[0].start,
                                      pdev->resource[0].end - pdev->resource[0].start + 1);

  if (!info->screen_base) {
    printk(KERN_ERR PFX "unable to map framebuffer\n");
    ret = -ENOMEM;
    goto bail;
  }

  revision = s1d13xxxfb_readreg(default_par, S1DREG_REV_CODE) >> 2;
  {
    const char *revstr = "unknown";
    switch (revision) {
    case S1D13506_CHIP_REV: revstr = "s1d13506"; break;
    case S1D13806_CHIP_REV: revstr = "s1d13806"; break;
    default: info("%s (%i)", revstr, revision);
      ret = -ENODEV;
      goto bail;
    }

    info("%s detected", revstr);
  }
               
  info->fix = s1d13xxxfb_fix;
  info->fix.mmio_start = pdev->resource[1].start;
  info->fix.mmio_len = pdev->resource[1].end - pdev->resource[1].start +1;
  info->fix.smem_start = pdev->resource[0].start;
  info->fix.smem_len = pdev->resource[0].end - pdev->resource[0].start +1;

  info("regs mapped at 0x%p, fb %d KiB mapped at 0x%p",
      default_par->regs, info->fix.smem_len / 1024, info->screen_base);

  info->par = default_par;
  info->fbops = &s1d13xxxfb_fbops;
  info->flags = FBINFO_DEFAULT | FBINFO_HWACCEL_YPAN;

  /* perform "manual" chip initialization, if needed */
  if (pdata && pdata->initregs) {
    info("initializing registers");
    s1d13xxxfb_runinit(info->par, pdata->initregs, pdata->initregssize);
  } else info("using present register settings");

  s1d13xxxfb_fetch_hw_state(info);

  if (register_framebuffer(info) < 0) {
    ret = -EINVAL;
    goto bail;
  }

  dev_set_drvdata(&pdev->dev, info);

  info("fb%d: %s frame buffer device", info->node, info->fix.id);

  return 0;

 bail:
  s1d13xxxfb_remove(dev);
  return ret;

}

#ifdef CONFIG_PM
static int s1d13xxxfb_suspend(struct device *dev, u32 state, u32 level)
{
  struct fb_info *info = dev_get_drvdata(dev);
  struct s1d13xxxfb_par *s1dfb = info->par;
  struct s1d13xxxfb_pdata *pdata = NULL;

  /* disable display */
  lcd_enable(s1dfb, 0);
  crt_enable(s1dfb, 0);

  if (dev->platform_data)
    pdata = dev->platform_data;

#if 0
  if (!s1dfb->disp_save)
    s1dfb->disp_save = kmalloc(info->fix.smem_len, GFP_KERNEL);

  if (!s1dfb->disp_save) {
    printk(KERN_ERR PFX "no memory to save screen");
    return -ENOMEM;
  }

  memcpy_fromio(s1dfb->disp_save, info->screen_base, info->fix.smem_len);
#else
  s1dfb->disp_save = NULL;
#endif

  if (!s1dfb->regs_save)
    s1dfb->regs_save = kmalloc(info->fix.mmio_len, GFP_KERNEL);

  if (!s1dfb->regs_save) {
    printk(KERN_ERR PFX "no memory to save registers");
    return -ENOMEM;
  }

  /* backup all registers */
  memcpy_fromio(s1dfb->regs_save, s1dfb->regs, info->fix.mmio_len);

  /* now activate power save mode */
  s1d13xxxfb_writereg(s1dfb, S1DREG_PS_CNF, 0x11);

  if (pdata && pdata->platform_suspend_video)
    return pdata->platform_suspend_video();
  else
    return 0;
}

static int s1d13xxxfb_resume(struct device *dev, u32 level)
{
  struct fb_info *info = dev_get_drvdata(dev);
  struct s1d13xxxfb_par *s1dfb = info->par;
  struct s1d13xxxfb_pdata *pdata = NULL;

  if (level != RESUME_ENABLE)
    return 0;

  /* awaken the chip */
  s1d13xxxfb_writereg(s1dfb, S1DREG_PS_CNF, 0x10);

  /* do not let go until SDRAM "wakes up" */
  while ((s1d13xxxfb_readreg(s1dfb, S1DREG_PS_STATUS) & 0x01))
    udelay(10);

  if (dev->platform_data)
    pdata = dev->platform_data;

  if (s1dfb->regs_save) {
    /* will write RO regs, *should* get away with it :) */
    memcpy_toio(s1dfb->regs, s1dfb->regs_save, info->fix.mmio_len);
    kfree(s1dfb->regs_save);
  }

  if (s1dfb->disp_save) {
    memcpy_toio(info->screen_base, s1dfb->disp_save,
                info->fix.smem_len);
    kfree(s1dfb->disp_save);	/* XXX kmalloc()'d when? */
  }

  if ((s1dfb->display & 0x01) != 0)
    lcd_enable(s1dfb, 1);
  if ((s1dfb->display & 0x02) != 0)
    crt_enable(s1dfb, 1);

  if (pdata && pdata->platform_resume_video)
    return pdata->platform_resume_video();
  else
    return 0;
}
#endif /* CONFIG_PM */

static struct device_driver s1d13xxxfb_driver = {
  .name		= S1D_DEVICENAME,
  .bus		= &platform_bus_type,
  .probe	= s1d13xxxfb_probe,
  .remove	= s1d13xxxfb_remove,
#ifdef CONFIG_PM
  .suspend	= s1d13xxxfb_suspend,
  .resume	= s1d13xxxfb_resume
#endif
};

int __devinit s1d13xxxfb_setup (char *options)
{
  if (NULL != options)
    strlcpy(s1d13xxxfb_options, options, sizeof(s1d13xxxfb_options));
  return 0;
}


static int __init
s1d13xxxfb_init(void)
{
  char *options;
  if (fb_get_options(S1D_DEVICENAME, &options))
    return -ENODEV;
  s1d13xxxfb_setup(options);
  
  return driver_register(&s1d13xxxfb_driver);
}


static void __exit
s1d13xxxfb_exit(void)
{
  driver_unregister(&s1d13xxxfb_driver);
}

module_init(s1d13xxxfb_init);
module_exit(s1d13xxxfb_exit);


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Framebuffer driver for S1D13xxx devices");
MODULE_AUTHOR("Ben Dooks <ben@simtec.co.uk>, Thibaut VARENE <varenet@parisc-linux.org>");