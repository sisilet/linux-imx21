/*
 *  Driver for Wolfson WM8731 on CSB536FS
 *  Copyright (C) 2005 Jay Monkman <jtm@lopingdog.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License.
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

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/control.h>
#include <sound/initval.h>

#define _cp printk("%s:%d (%u)\n", __FUNCTION__, __LINE__, jiffies);

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
    int dma;
    snd_pcm_substream_t *play_substream;
    int play_pos;
    int play_last_period;
    u32 d_start;
    u32 d_cur;
    u32 d_size;
    u32 d_end;
    u32 d_chunk;
} csb536_chip_t;


/***********************************************************************
 * Global Variables
 ***********************************************************************/
static snd_device_ops_t g_mixer_ops = {
    .dev_free = NULL,
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

int g_ptr = 0;

/************************************************************
 * PCM Functions
 ************************************************************/
static int snd_csb536_playback_open(snd_pcm_substream_t *substream)
{
    csb536_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

//    _cp;
//    printk(KERN_INFO "----------------------------------------------\n");
//    printk(KERN_INFO "open\n");

    runtime->hw = snd_csb536_playback_hw;
    chip->play_substream = substream;

#warning "Put HW stuff here"
    return 0;
}

static int snd_csb536_playback_close(snd_pcm_substream_t *substream)
{
    csb536_chip_t *chip = snd_pcm_substream_chip(substream);

//    _cp;
//    printk(KERN_INFO "close\n\n\n\n\n");

#warning "Put HW stuff here"
    return 0;
}

static int snd_csb536_pcm_hw_params(snd_pcm_substream_t *substream,
                                    snd_pcm_hw_params_t *hw_params)
{
    csb536_chip_t *chip = snd_pcm_substream_chip(substream);

//    _cp;
#warning "Put HW stuff here?"
    chip->play_pos = 0;
    chip->play_last_period = 0;

    return snd_pcm_lib_malloc_pages(substream,
                                    params_buffer_bytes(hw_params));
}


static int snd_csb536_pcm_hw_free(snd_pcm_substream_t *substream)
{
//    _cp;
#warning "Put HW stuff here?"
    return snd_pcm_lib_free_pages(substream);
}

static int snd_csb536_pcm_prepare(snd_pcm_substream_t *substream)
{
    csb536_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

//    _cp;
//    printk("   Sample rate is %d\n", runtime->rate);
//    printk("   format is %d\n", runtime->format);
//    printk("   channels is %d\n", runtime->channels);

//    printk("   dma_area: %#x\n", runtime->dma_area);
    /* Since memory type is CONTINUOUS, dma_addr isn't set */
    runtime->dma_addr = __pa(runtime->dma_area);
//    printk("   dma_addr: %#x\n", runtime->dma_addr);
//    printk("   dma_bytes: %#x\n", runtime->dma_bytes);
//    printk("   periods is %d\n", runtime->periods);
//    printk("   period_size(bytes) is %d\n", 
//           frames_to_bytes(runtime, runtime->period_size));

//    printk(KERN_INFO "prepare rate:%d  area:0x%x  addr:0x%x  bytes:%d\n", 
//           runtime->rate, runtime->dma_area, runtime->dma_addr, runtime->dma_bytes);
    chip->d_start = runtime->dma_addr;
    chip->d_cur = runtime->dma_addr;
    chip->d_size = runtime->dma_bytes;
    chip->d_end = chip->d_start + chip->d_size;


    SAR(chip->dma) = chip->d_cur;
    DAR(chip->dma) = 0x218000;
    chip->d_chunk = frames_to_bytes(runtime, runtime->period_size);
    CNTR(chip->dma) = chip->d_chunk;

    CCR(chip->dma) = (CCR_DMOD_FIFO |
                      CCR_SMOD_LINEAR |
                      CCR_DSIZ_16 |
                      CCR_SSIZ_32 |
                      CCR_REN);

    RSSR(chip->dma) = DMA_REQ_SSI_T;
    BLR(chip->dma) = 8;
    BUCR(chip->dma) = 0;
    CCR(chip->dma) |= (CCR_CEN);


    chip->play_pos = 0;
#warning "Put HW stuff here"
    g_ptr = 0;
    /* set codec/ssi to sample rate, init DMA */
    return 0;
}

static int snd_csb536_pcm_trigger(snd_pcm_substream_t *substream, int cmd)
{
    csb536_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;
//    int i;
//    static sine_pos = 0;

//    _cp;
    switch(cmd) {
    case SNDRV_PCM_TRIGGER_START:
#warning "Put HW stuff here"
//        SSI_STCR |= SSI_STCR_TIE;
        SSI_STCR |= SSI_STCR_TDMAE;
        break;
    case SNDRV_PCM_TRIGGER_STOP:
#warning "Put HW stuff here"
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
    unsigned int current_ptr;
    unsigned int offset;

    u32 dar;
    u32 sar;
    u32 cntr;
    u16 data = 0;
//    _cp;

#warning "Put HW stuff here"

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
 
static int __init snd_csb536_pcm_new(csb536_chip_t *chip)
{
    snd_pcm_t *pcm;
    int rc;

//    _cp;

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

    _cp;
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

    _cp;
    info = (csb536_mixer_info_t *) kcontrol->private_value;

#warning "do HW access here"

    return 0;
}


static int snd_csb536_mixer_put(snd_kcontrol_t *kcontrol,
                                snd_ctl_elem_value_t *uctrl)
{
    csb536_chip_t *chip = snd_kcontrol_chip(kcontrol);
    csb536_mixer_info_t *info;
    int changed = 0;
    _cp;
    info = (csb536_mixer_info_t *) kcontrol->private_value;

#warning "do HW access here"
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
    _cp;

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
static void csb536_dma_isr(int irq, void *data, struct pt_regs *regs)
{
    csb536_chip_t *chip = (csb536_chip_t *)data;
    snd_pcm_substream_t *substream = chip->play_substream;
    snd_pcm_runtime_t *runtime = substream->runtime;
    u32 cntr;
//    _cp;

    if (DBTOSR != 0) {
        printk("DBTOSR = 0x%x\n", DBTOSR);
    }

    if (DRTOSR != 0) {
        printk("DTROSR = 0x%x\n", DRTOSR);
    }

    if (DSESR != 0) {
        printk("DTROSR = 0x%x\n", DSESR);
    }

    if (DBOSR != 0) {
        printk("DBOSR = 0x%x\n", DBOSR);
    }



    DISR |= 1 << chip->dma;
    
    cntr = CNTR(chip->dma);

    chip->play_pos += (cntr);
    if (((chip->d_cur - chip->d_start) + cntr) >= chip->d_size) {
        /* Done */
        snd_pcm_period_elapsed(substream);
        chip->play_pos = 0;
        chip->d_cur = chip->d_start;
        SAR(chip->dma) = chip->d_cur;
        CCR(chip->dma) &= ~CCR_CEN;
        CCR(chip->dma) |= CCR_CEN;

        return;
    } else {
        chip->d_cur = chip->d_cur + cntr;
        SAR(chip->dma) = chip->d_cur;

        if ((chip->d_cur + cntr) > chip->d_end) {
            CNTR(chip->dma) = chip->d_end - chip->d_cur;
        }

    snd_pcm_period_elapsed(substream);
        CCR(chip->dma) &= ~CCR_CEN;
        CCR(chip->dma) |= CCR_CEN;

    return;
    }
}
    
static irqreturn_t csb536_tx_isr(int irq, void *data, struct pt_regs *regs)
{
    csb536_chip_t *chip = (csb536_chip_t *)data;
    snd_pcm_substream_t *substream = chip->play_substream;
    snd_pcm_runtime_t *runtime = substream->runtime;
    int i;
    int frame_size = frames_to_bytes(runtime, 1) / 2;

    _cp;

//    for (i = 0; i < frame_size; i++) {
//        int idx = (chip->play_pos * frame_size) + i;
//        SSI_STX = ((u16*)runtime->dma_area)[idx];
//    }
//    chip->play_pos++;
//
//    if ((chip->play_pos - chip->play_last_period) > runtime->period_size) {
//        chip->play_last_period = chip->play_pos;
//        snd_pcm_period_elapsed(substream);
//    }
//
//    if (chip->play_pos >= runtime->dma_bytes) {
//        chip->play_pos = 0;
//    }
        
    return IRQ_RETVAL(1);

}


static int __init csb536_wm8731_init(void)
{
    int rc;
    snd_card_t *card;
    csb536_chip_t *chip;
    extern struct i2c_client *csb536_wm8731_client;
    int dma;

    _cp;
    /* Make sure we're running on the right system */
    if (machine_is_csb536() == 0) {
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

//    /* Install interrupt handler */
//    rc = request_irq(SSI_TX_INT, csb536_tx_isr, 0, "csb536-alsa", chip);
//    if (rc != 0) {
//        printk("Error installing interrupt handler\n");
//        return rc;
//    }

    dma = imx_request_dma("CSB536-ALSA", DMA_PRIO_MEDIUM, 
                          csb536_dma_isr, NULL, chip);
    if (dma < 0) {
        printk("Could not allocate DMA channel for sound driver\n");
        return dma;
    }
    chip->dma = dma;
    

    
                          


    SSI_SOR   = (SSI_SOR_RXCLR | SSI_SOR_TXCLR |SSI_SOR_SYNRST);
    udelay(1000);
    SSI_SOR   = 0;
    udelay(1000);
    SSI_SCSR  = (SSI_SCSR_EN);
    udelay(1000);

    SSI_SCSR  = (SSI_SCSR_TE | //SSI_SCSR_RE | 
                 SSI_SCSR_EN | SSI_SCSR_NET);

    SSI_STCR  = (SSI_STCR_TFEN | SSI_STCR_TSCKP | 
                 SSI_STCR_TFSL | SSI_STCR_TXDIR | SSI_STCR_TFDIR);
//    SSI_SRCR  = (SSI_SRCR_RFEN | SSI_SRCR_RSCKP |
//                 SSI_SRCR_RFSL | SSI_SRCR_RXDIR | SSI_SRCR_RFDIR);
    /* Divide clock by 12 -> ((96MHz/8)/4)/12 = 250Khz */
    SSI_STCCR = (SSI_STCCR_WL16 | SSI_STCCR_DC(1) | SSI_STCCR_PM(11));
//    SSI_SRCCR = (SSI_SRCCR_WL16 | SSI_SRCCR_DC(1) | SSI_SRCCR_PM(11));
    SSI_SFCSR = (SSI_SFCSR & ~0xf) | 4;


    rc = snd_csb536_mixer_new(chip);
    if (rc != 0) {
        goto init_err1;
    }

    rc = snd_csb536_pcm_new(chip);
    if (rc != 0) {
        goto init_err2;
    }

    rc = snd_card_register(card);
    if (rc == 0) {
        printk(KERN_INFO "CSB536FS WM8731 audio support initialized\n");
        return 0;
    }

    /* undo pcm_new() */

 init_err2:
    /* undo mixer_new() */
    _cp;

 init_err1:

    _cp;
    snd_card_free(card);

    return rc;
}

static void __exit csb536_wm8731_exit(void)
{
    return;
}


module_init(csb536_wm8731_init);
module_exit(csb536_wm8731_exit);
