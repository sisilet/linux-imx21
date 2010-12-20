/*
 *  Driver for Wolfson WM8731 on CSB536FS
 *  Copyright (C) 2005 Jay Monkman <jtm@lopingdog.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License.
 *
 *  WARNING:
 *     Normally the frame/bit clock is related to the CODEC's main
 *     clock. That is not the case for the CSB535FS. On the CSB536FS,
 *     the CPU generates the frame and data clocks, but the WM8731
 *     generates its own master clock. It doesn't seem to cause any
 *     problems at 8Khz, but who knows?
 *
 *  TODO:
 *     handle power management
 */

#include <linux/config.h>
#include <sound/driver.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/dma.h>
#include <asm/arch/imx-dma.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/control.h>
#include <sound/initval.h>

#include <sound/wm8731.h>

/***********************************************************************
 * Module stuff
 ***********************************************************************/
static char *id = NULL;
module_param(id, charp, 0444);

MODULE_AUTHOR("Jay Monkman <jtm@lopingdog.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CSB536FS Audio driver (WM8731 CODEC)");
MODULE_PARM_DESC(id, "ID string for CSB536FS/WM8731.");



/***********************************************************************
 * Data structures
 ***********************************************************************/
typedef struct {
    snd_card_t *card;
    snd_pcm_t *pcm;
    struct i2c_client *wm8731;
    imx_dmach_t play_dma;
    imx_dmach_t capture_dma;
    snd_pcm_substream_t *play_substream;
    snd_pcm_substream_t *capture_substream;
    int play_pos;
    u32 play_start;
    u32 play_cur;
    u32 play_size;
    u32 play_end;
    u32 play_chunk;
    int capture_pos;
    u32 capture_start;
    u32 capture_cur;
    u32 capture_size;
    u32 capture_end;
    u32 capture_chunk;
} csb536_chip_t;


/***********************************************************************
 * Global Variables
 ***********************************************************************/
static snd_device_ops_t g_mixer_ops = {
    .dev_free = NULL,
};

static unsigned int rates[] = {8000};
static snd_pcm_hw_constraint_list_t hw_constraints_rates = {
	.count	=  sizeof(rates) / sizeof(rates[0]),
	.list	= rates,
	.mask	= 0,
};

static snd_pcm_hardware_t snd_csb536_playback_hw = {
    .info = (SNDRV_PCM_INFO_INTERLEAVED    |
             SNDRV_PCM_INFO_BLOCK_TRANSFER |
             SNDRV_PCM_INFO_MMAP           |
             SNDRV_PCM_INFO_MMAP_VALID),
    .formats = SNDRV_PCM_FMTBIT_S16_LE,
    .rates = SNDRV_PCM_RATE_8000,
    .rate_min = 8000,
    .rate_max = 8000,
    .channels_min = 2,
    .channels_max = 2,
    .buffer_bytes_max = 32768,
    .period_bytes_min = 4096,
    .period_bytes_max = 32768,
    .periods_min = 1,
    .periods_max = 1024,
};

static snd_pcm_hardware_t snd_csb536_capture_hw = {
    .info = (SNDRV_PCM_INFO_INTERLEAVED    |
             SNDRV_PCM_INFO_BLOCK_TRANSFER |
             SNDRV_PCM_INFO_MMAP           |
             SNDRV_PCM_INFO_MMAP_VALID),
    .formats = SNDRV_PCM_FMTBIT_S16_LE,
    .rates = SNDRV_PCM_RATE_8000,
    .rate_min = 8000,
    .rate_max = 8000,
    .channels_min = 2,
    .channels_max = 2,
    .buffer_bytes_max = 32768,
    .period_bytes_min = 4096,
    .period_bytes_max = 32768,
    .periods_min = 1,
    .periods_max = 1024,
};


/************************************************************
 * PCM Functions
 ************************************************************/
static int snd_csb536_playback_open(snd_pcm_substream_t *substream)
{
    int rc;
    csb536_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

    runtime->hw = snd_csb536_playback_hw;
    chip->play_substream = substream;

    rc = snd_pcm_hw_constraint_list(runtime, 0, 
                                    SNDRV_PCM_HW_PARAM_RATE,
                                    &hw_constraints_rates);
    if (rc < 0) {
        printk("Error setting rates constraint\n");
        return rc;
    }

    return 0;
}

static int snd_csb536_playback_close(snd_pcm_substream_t *substream)
{
    csb536_chip_t *chip = snd_pcm_substream_chip(substream);

    chip->play_substream = NULL;

    return 0;
}

static int snd_csb536_pcm_hw_params(snd_pcm_substream_t *substream,
                                    snd_pcm_hw_params_t *hw_params)
{
    csb536_chip_t *chip = snd_pcm_substream_chip(substream);


    return snd_pcm_lib_malloc_pages(substream,
                                    params_buffer_bytes(hw_params));
}


static int snd_csb536_pcm_hw_free(snd_pcm_substream_t *substream)
{
    return snd_pcm_lib_free_pages(substream);
}

static int snd_csb536_pcm_prepare(snd_pcm_substream_t *substream)
{
    csb536_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

    /* Get physical address of data buffer */
    runtime->dma_addr = __pa(runtime->dma_area);

    chip->play_pos = 0;
    chip->play_start = runtime->dma_addr;
    chip->play_cur = runtime->dma_addr;
    chip->play_size = runtime->dma_bytes;
    chip->play_end = chip->play_start + chip->play_size;

    SAR(imx_dma_c2n(chip->play_dma)) = chip->play_cur;
    DAR(imx_dma_c2n(chip->play_dma)) = 0x218000;
    chip->play_chunk = frames_to_bytes(runtime, runtime->period_size);
    CNTR(imx_dma_c2n(chip->play_dma)) = chip->play_chunk;

    CCR(imx_dma_c2n(chip->play_dma)) = (CCR_DMOD_FIFO |
                      CCR_SMOD_LINEAR |
                      CCR_DSIZ_16 |
                      CCR_SSIZ_32 |
                      CCR_REN);

    RSSR(imx_dma_c2n(chip->play_dma)) = DMA_REQ_SSI_T;
    BLR(imx_dma_c2n(chip->play_dma)) = 8;
    BUCR(imx_dma_c2n(chip->play_dma)) = 0;
    CCR(imx_dma_c2n(chip->play_dma)) |= (CCR_CEN);


    /* 
     * NOTE: If the driver ever supports varying the sample rate,
     *       add code to tell the CODEC here.
     */

    return 0;
}

static int snd_csb536_pcm_trigger(snd_pcm_substream_t *substream, int cmd)
{
    csb536_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

    switch(cmd) {
    case SNDRV_PCM_TRIGGER_START:
        SSI_STCR |= SSI_STCR_TDMAE;
        imx_dma_enable(chip->play_dma);
        break;
    case SNDRV_PCM_TRIGGER_STOP:
        imx_dma_disable(chip->play_dma);
        SSI_STCR &= ~SSI_STCR_TDMAE;
        break;
    default:
        return -EINVAL;
        break;
    }

    return 0;
}

static snd_pcm_uframes_t snd_csb536_pcm_pointer(
    snd_pcm_substream_t *substream)
{
    csb536_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;
    unsigned int offset;


    /* current_ptr = csb536_get_hw_point/home/jtm/er(chip) */
    if (chip->play_pos != 0) {
        offset = bytes_to_frames(runtime, chip->play_pos);
        if (offset >= runtime->buffer_size) {
            offset = 0;
        }
        return offset;
    } else {
        return 0;
    }

}

static int snd_csb536_capture_open(snd_pcm_substream_t *substream)
{
    int rc;
    csb536_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

    runtime->hw = snd_csb536_capture_hw;
    chip->capture_substream = substream;

    rc = snd_pcm_hw_constraint_list(runtime, 0, 
                                    SNDRV_PCM_HW_PARAM_RATE,
                                    &hw_constraints_rates);
    if (rc < 0) {
        printk("Error setting rates constraint\n");
        return rc;
    }

    return 0;
}

static int snd_csb536_capture_close(snd_pcm_substream_t *substream)
{
    csb536_chip_t *chip = snd_pcm_substream_chip(substream);

    chip->capture_substream = NULL;

    return 0;
}

static int snd_csb536_pcm_capture_prepare(snd_pcm_substream_t *substream)
{
    csb536_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

    /* Get physical address of data buffer */
    runtime->dma_addr = __pa(runtime->dma_area);

    chip->capture_pos = 0;
    chip->capture_start = runtime->dma_addr;
    chip->capture_cur = runtime->dma_addr;
    chip->capture_size = runtime->dma_bytes;
    chip->capture_end = chip->capture_start + chip->capture_size;

    SAR(imx_dma_c2n(chip->capture_dma)) = 0x218004;
    DAR(imx_dma_c2n(chip->capture_dma)) = chip->capture_cur;
    chip->capture_chunk = frames_to_bytes(runtime, runtime->period_size);
    CNTR(imx_dma_c2n(chip->capture_dma)) = chip->capture_chunk;

    CCR(imx_dma_c2n(chip->capture_dma)) = (CCR_SMOD_FIFO |
                      CCR_DMOD_LINEAR |
                      CCR_SSIZ_16 |
                      CCR_DSIZ_32 |
                      CCR_REN);

    RSSR(imx_dma_c2n(chip->capture_dma)) = DMA_REQ_SSI_R;
    BLR(imx_dma_c2n(chip->capture_dma)) = 8;
    BUCR(imx_dma_c2n(chip->capture_dma)) = 0;
    CCR(imx_dma_c2n(chip->capture_dma)) |= (CCR_CEN);


    /* 
     * NOTE: If the driver ever supports varying the sample rate,
     *       add code to tell the CODEC here.
     */

    return 0;
}

static int snd_csb536_pcm_capture_trigger(snd_pcm_substream_t *substream, int cmd)
{
    csb536_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

    switch(cmd) {
    case SNDRV_PCM_TRIGGER_START:
        SSI_SRCR |= SSI_SRCR_RDMAE;
        imx_dma_enable(chip->capture_dma);
        break;
    case SNDRV_PCM_TRIGGER_STOP:
        imx_dma_disable(chip->capture_dma);
        SSI_SRCR &= ~SSI_SRCR_RDMAE;
        break;
    default:
        return -EINVAL;
        break;
    }

    return 0;
}

static snd_pcm_uframes_t snd_csb536_pcm_capture_pointer(
    snd_pcm_substream_t *substream)
{
    csb536_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;
    unsigned int offset;

    /* current_ptr = csb536_get_hw_point/home/jtm/er(chip) */
    if (chip->capture_pos != 0) {
        offset = bytes_to_frames(runtime, chip->capture_pos);
        if (offset >= runtime->buffer_size) {
            offset = 0;
        }
        return offset;
    } else {
        return 0;
    }

}

static snd_pcm_ops_t snd_csb536_playback_ops = {
    .open = snd_csb536_playback_open,
    .close = snd_csb536_playback_close,
    .ioctl = snd_pcm_lib_ioctl,
    .hw_params = snd_csb536_pcm_hw_params,
    .hw_free = snd_csb536_pcm_hw_free,
    .prepare = snd_csb536_pcm_prepare,
    .trigger = snd_csb536_pcm_trigger,
    .pointer = snd_csb536_pcm_pointer,
};

static snd_pcm_ops_t snd_csb536_capture_ops = {
    .open = snd_csb536_capture_open,
    .close = snd_csb536_capture_close,
    .ioctl = snd_pcm_lib_ioctl,
    .hw_params = snd_csb536_pcm_hw_params,
    .hw_free = snd_csb536_pcm_hw_free,
    .prepare = snd_csb536_pcm_capture_prepare,
    .trigger = snd_csb536_pcm_capture_trigger,
    .pointer = snd_csb536_pcm_capture_pointer,
};
 
static int __init snd_csb536_pcm_new(csb536_chip_t *chip)
{
    snd_pcm_t *pcm;
    int rc;

    rc = snd_pcm_new(chip->card, "CSB536FS PCM", 0, 1, 1, &pcm);
    if (rc < 0) {
        return rc;
    }

    pcm->private_data = chip;
    strcpy(pcm->name, "CSB536FS PCM");
    chip->pcm = pcm;

    snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS,
                                          snd_dma_continuous_data(GFP_KERNEL),
                                          64*1024, 64*1024);

    snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
                    &snd_csb536_playback_ops);

    snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
                    &snd_csb536_capture_ops);

    return 0;
}
 

/************************************************************
 * Mixer Functions
 ************************************************************/
/* 
 * reg is 4 bits
 * shift is 4 bit
 * mask is 9 bits
 */
#define WM8731_CONTROL(n, reg, shift, mask)  \
{ \
    .iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
    .name = n,                           \
    .info = snd_csb536_mixer_info,       \
    .get = snd_csb536_mixer_get,         \
    .put = snd_csb536_mixer_put,         \
    .private_value = ((reg) | ((shift) << 4) | ((mask) << 8)),  \
}

typedef struct {
    char *name;
    enum sndrv_ctl_elem_type type;
    int min;
    int max;
    int reg;
    u16 mask;
    int shift;
} csb536_mixer_info_t;

static int snd_csb536_mixer_info(snd_kcontrol_t *kcontrol,
                                 snd_ctl_elem_info_t *uinfo)
{
    int mask = (kcontrol->private_value >> 8) & 0x1ff;

    if (mask == 1) {
        uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
    } else {
        uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    }

    uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = mask;
    return 0;
}

static int snd_csb536_mixer_get(snd_kcontrol_t *kcontrol,
                                snd_ctl_elem_value_t *uctrl)
{
    csb536_chip_t *chip = snd_kcontrol_chip(kcontrol);
    csb536_mixer_info_t *info;

    info = (csb536_mixer_info_t *) kcontrol->private_value;


    return 0;
}


static int snd_csb536_mixer_put(snd_kcontrol_t *kcontrol,
                                snd_ctl_elem_value_t *uctrl)
{
    csb536_chip_t *chip = snd_kcontrol_chip(kcontrol);
    csb536_mixer_info_t *info;
    int changed = 0;
    info = (csb536_mixer_info_t *) kcontrol->private_value;

    changed = 1;
    return changed;
}

static snd_kcontrol_new_t csb536_mixer_controls[] = {
    WM8731_CONTROL("Master Playback Volume", 0, 0, 8),
};
    
    
static int __init snd_csb536_mixer_new(csb536_chip_t *chip)
{
    int rc;
    snd_card_t *card = chip->card;
    int i;
    extern struct i2c_client *csb536_wm8731_client;

    rc = snd_device_new(card, SNDRV_DEV_CODEC, csb536_wm8731_client, &g_mixer_ops);

    for (i = 0; 
         i < (sizeof(csb536_mixer_controls)/sizeof(csb536_mixer_controls[0]));
         i++) {
        rc = snd_ctl_add(card, snd_ctl_new1(&csb536_mixer_controls[i], chip));
        if (rc < 0) {
            return rc;
        }
    }
    return rc;
}



/************************************************************
 * Misc Functions
 ************************************************************/
static void csb536_dma_play_isr(int irq, void *data, struct pt_regs *regs)
{
    csb536_chip_t *chip = (csb536_chip_t *)data;
    snd_pcm_substream_t *substream = chip->play_substream;
    snd_pcm_runtime_t *runtime = substream->runtime;
    u32 cntr;

    if (DBTOSR != 0) {
        printk("DMA Error: DBTOSR = 0x%x\n", DBTOSR);
    }

    if (DRTOSR != 0) {
        printk("DMA Error: DTROSR = 0x%x\n", DRTOSR);
    }

    if (DSESR != 0) {
        printk("DMA Error: DTROSR = 0x%x\n", DSESR);
    }

    if (DBOSR != 0) {
        printk("DMA Error: DBOSR = 0x%x\n", DBOSR);
    }


//    DISR |= 1 << imx_dma_c2n(chip->play_dma);
    
    cntr = CNTR(imx_dma_c2n(chip->play_dma));

    chip->play_pos += (cntr);
    if (((chip->play_cur - chip->play_start) + cntr) >= chip->play_size) {
        /* Done */
        snd_pcm_period_elapsed(substream);
        chip->play_pos = 0;
        chip->play_cur = chip->play_start;
        SAR(imx_dma_c2n(chip->play_dma)) = chip->play_cur;
        CCR(imx_dma_c2n(chip->play_dma)) &= ~CCR_CEN;
        CCR(imx_dma_c2n(chip->play_dma)) |= CCR_CEN;

        return;
    } else {
        chip->play_cur = chip->play_cur + cntr;
        SAR(imx_dma_c2n(chip->play_dma)) = chip->play_cur;

        if ((chip->play_cur + cntr) > chip->play_end) {
            CNTR(imx_dma_c2n(chip->play_dma)) = chip->play_end - chip->play_cur;
        }

    snd_pcm_period_elapsed(substream);
        CCR(imx_dma_c2n(chip->play_dma)) &= ~CCR_CEN;
        CCR(imx_dma_c2n(chip->play_dma)) |= CCR_CEN;

    return;
    }
}
    
static void csb536_dma_capture_isr(int irq, void *data, struct pt_regs *regs)
{
    csb536_chip_t *chip = (csb536_chip_t *)data;
    snd_pcm_substream_t *substream = chip->capture_substream;
    snd_pcm_runtime_t *runtime = substream->runtime;
    u32 cntr;

    if (DBTOSR != 0) {
        printk("DMA Error: DBTOSR = 0x%x\n", DBTOSR);
    }

    if (DRTOSR != 0) {
        printk("DMA Error: DTROSR = 0x%x\n", DRTOSR);
    }

    if (DSESR != 0) {
        printk("DMA Error: DTROSR = 0x%x\n", DSESR);
    }

    if (DBOSR != 0) {
        printk("DMA Error: DBOSR = 0x%x\n", DBOSR);
    }


//    DISR |= 1 << imx_dma_c2n(chip->capture_dma);
    
    cntr = CNTR(imx_dma_c2n(chip->capture_dma));

    chip->capture_pos += (cntr);
    if (((chip->capture_cur - chip->capture_start) + cntr) >= chip->capture_size) {
        /* Done */
        snd_pcm_period_elapsed(substream);
        chip->capture_pos = 0;
        chip->capture_cur = chip->capture_start;
        DAR(imx_dma_c2n(chip->capture_dma)) = chip->capture_cur;
        CCR(imx_dma_c2n(chip->capture_dma)) &= ~CCR_CEN;
        CCR(imx_dma_c2n(chip->capture_dma)) |= CCR_CEN;

        return;
    } else {
        chip->capture_cur = chip->capture_cur + cntr;
        DAR(imx_dma_c2n(chip->capture_dma)) = chip->capture_cur;

        if ((chip->capture_cur + cntr) > chip->capture_end) {
            CNTR(imx_dma_c2n(chip->capture_dma)) = chip->capture_end - chip->capture_cur;
        }

    snd_pcm_period_elapsed(substream);
        CCR(imx_dma_c2n(chip->capture_dma)) &= ~CCR_CEN;
        CCR(imx_dma_c2n(chip->capture_dma)) |= CCR_CEN;

    return;
    }
}
    
static int __init csb536_wm8731_init(void)
{
    int rc = -ENODEV;
    snd_card_t *card;
    csb536_chip_t *chip;
    extern struct i2c_client *csb536_wm8731_client;

    /* Make sure we're running on the right system */
    if (machine_is_csb536() == 0) {
        return -ENODEV;
    }

    /* Make sure we have a CODEC */
    if (csb536_wm8731_client == NULL) {
        return -ENODEV;
    }

    /* Register the sound card */
    card = snd_card_new(-1, id, THIS_MODULE, sizeof(csb536_chip_t));
    if (card == NULL) {
        return -ENOMEM;
    }

    chip = (csb536_chip_t *)card->private_data;

    chip->card = card;
    chip->wm8731 = csb536_wm8731_client;

    if (imx_dma_request_by_prio(&chip->play_dma,
                                "CSB536-ALSA", 
                                DMA_PRIO_MEDIUM) < 0) {
        goto init_err0;
    }
    imx_dma_setup_handlers(chip->play_dma, csb536_dma_play_isr, 
                           NULL, chip);

    if (imx_dma_request_by_prio(&chip->capture_dma, 
                                "CSB536-ALSA", 
                                DMA_PRIO_MEDIUM) < 0) {
        goto init_err1;
    }
    imx_dma_setup_handlers(chip->capture_dma, csb536_dma_capture_isr, 
                           NULL, chip);


    SSI_SOR   = (SSI_SOR_RXCLR | SSI_SOR_TXCLR |SSI_SOR_SYNRST);
    udelay(1000);
    SSI_SOR   = 0;
    udelay(1000);
    SSI_SCSR  = (SSI_SCSR_EN);
    udelay(1000);

    SSI_SCSR  = (SSI_SCSR_TE | SSI_SCSR_RE | 
                 SSI_SCSR_EN | SSI_SCSR_NET);

    SSI_STCR  = (SSI_STCR_TFEN | SSI_STCR_TSCKP | 
                 SSI_STCR_TFSL | SSI_STCR_TXDIR | SSI_STCR_TFDIR);
    /* TX & RX are connected, so TX clock drives RX clock */
    SSI_SRCR  = (SSI_SRCR_RFEN | SSI_SRCR_RSCKP |
//                 SSI_SRCR_RFSL | SSI_SRCR_RXDIR | SSI_SRCR_RFDIR);
                 SSI_SRCR_RFSL | SSI_SRCR_RFDIR);
    /* Divide clock by 12 -> ((96MHz/8)/4)/12 = 250Khz */
    SSI_STCCR = (SSI_STCCR_WL16 | SSI_STCCR_DC(1) | SSI_STCCR_PM(11));
    SSI_SRCCR = (SSI_SRCCR_WL16 | SSI_SRCCR_DC(1) | SSI_SRCCR_PM(11));
    SSI_SFCSR = (SSI_SFCSR & ~0xf) | 4;


    rc = snd_csb536_mixer_new(chip);
    if (rc != 0) {
        goto init_err2;
    }

    rc = snd_csb536_pcm_new(chip);
    if (rc != 0) {
        goto init_err2;
    }

    rc = snd_card_register(card);
    if (rc != 0) {
        goto init_err2;
    }


    wm8731_write_reg(chip->wm8731, WM8731_REG_LLI,
                     (WM8731_LIN_VOL(0x18) | WM8731_LIN_BOTH));
    wm8731_write_reg(chip->wm8731, WM8731_REG_LHO,
                     (WM8731_HPO_VOL(0x60) | WM8731_HPO_BOTH));
    wm8731_write_reg(chip->wm8731, WM8731_REG_AAP,
                     (WM8731_AAP_BYPASS | WM8731_AAP_DACSEL));
    wm8731_write_reg(chip->wm8731, WM8731_REG_DAP, 0);
                     
    /* FIXME: Should turn features off, until needed */
    wm8731_write_reg(chip->wm8731, WM8731_REG_PDC, 0);

    wm8731_write_reg(chip->wm8731, WM8731_REG_DAI,
                     (WM8731_DAI_FMT_DSP | 
                      WM8731_DAI_IWL16 | 
                      WM8731_DAI_LRP1));

    /* Set for 8Khz */
    wm8731_write_reg(chip->wm8731, WM8731_REG_SMP, WM8731_SMP_SR(3));
                     
    wm8731_write_reg(chip->wm8731, WM8731_REG_ACT, 1);

    printk(KERN_INFO "CSB536FS WM8731 audio support initialized\n");

    return rc;

 init_err2:
    imx_dma_free(chip->capture_dma);

 init_err1:
    imx_dma_free(chip->play_dma);

 init_err0:
    snd_card_free(card);

    return rc;
}

static void __exit csb536_wm8731_exit(void)
{
    return;
}


module_init(csb536_wm8731_init);
module_exit(csb536_wm8731_exit);
