#if 0
/*
 * voyagergx_sound.c -- voyager sound driver
 *
 *     Copyright (c) 2005 Bill Gatliff <bgat@billgatliff.com>
 *
 * A complete rewrite of code originally developed by:
 *
 *     Copyright (C) 2003 Renesas Technology Sales Co.,Ltd.
 *     Copyright (C) 2003 Atom Create Engineering Co.,Ltd.
 *     Anthor : Atom Create Engineering Co.,Ltd.
 *                   Kenichi Sakuma
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 *
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/sound.h>
#include <linux/slab.h>
#include <linux/soundcard.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/pci.h>
#include <linux/bitops.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/smp_lock.h>
#include <linux/ac97_codec.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/hardirq.h>

/* TODO: for now, share the video header file even though we don't
   really care about (move header file to somewhere more generic) */
#include <video/voyagergx_reg.h>

#include <asm/irq.h>
#include "voyagergx_sound.h"

#define PFX

#define err(format, arg...) printk(KERN_ERR PFX ": " format "\n" , ## arg)
#define info(format, arg...) printk(KERN_INFO PFX ": " format "\n" , ## arg)

/* --------------------------------------------------------------------- */
#undef OSS_DOCUMENTED_MIXER_SEMANTICS

#define VOYAGERGX_MODULE_NAME "Voyagergx audio"
#define PFX VOYAGERGX_MODULE_NAME

#define err(format, arg...) printk(KERN_ERR PFX ": " format "\n" , ## arg)
#define info(format, arg...) printk(KERN_INFO PFX ": " format "\n" , ## arg)
//#define info(format, arg...) printk(": " format "\n" , ## arg)
//#define warn(format, arg...) printk(KERN_WARNING PFX ": " format "\n" , ## arg)

//�Хåե�������
#define	VOYAGER_SOUND_SIZE	0x100000
//�ǡ����Хåե����ݣ֣ң��;�
#define	VOYAGER_SOUND_BUF1	0xb0200000
//�ǡ����Хåե����ݣ֣ң��;�
#define	VOYAGER_SOUND_BUF2	VOYAGER_SOUND_BUF1+VOYAGER_SOUND_SIZE
//WAIT TIME OUT
//#define	WAIT_TIMEOUT	((VOYAGER_SOUND_SIZE / 48000) * HZ + 10)
#define	WAIT_TIMEOUT	1100

DECLARE_WAIT_QUEUE_HEAD(int_wq);

/* Boot options */
static int      vra = 0;	// 0 = no VRA, 1 = use VRA if codec supports it
MODULE_PARM(vra, "i");
MODULE_PARM_DESC(vra, "if 1 use VRA if codec supports it");

struct voyager_setting {
	int             ch;
	int		fmt;
	int		rate;
} setting;

//��������������ؿ�
int	init_8051(void);
//�����������ޥ�ɵ�ư�ؿ�
int	command_8051(int com,int *data,int *data2);

//�ģͣ����ѥ����ͥ�ݣģͣ��λ��ѥ����ͥ뤬����ʣ������
static	int	sh7751_dmasound_play_irq = 0;
//�Хåե������ѥե饰�ݻ�����λ���ž�������������åȤ����
static	int	buff_flg1 = 0;
//�Хåե������ѥե饰�ݻ�����λ���ž�������������åȤ����
static	int	buff_flg2 = 0;
//�Хåե�Ƚ��ե饰�ݣģͣ����ɤ���ΥХåե���ȤäƤ���Τ�������
static	int	proc_flg = 0;
//���Ƚ��ե饰�ݥǡ���ž���ν���Ƚ�Ǥ���
static	int	first_flg = 0;
//�ǽ�Ƚ��ե饰�ݥǡ���ž���κǸ��Ƚ�Ǥ���
static	int	last_flg = 0;
//�����ߥե饰�ݳ����ߤ�ȯ������ȣ����ꥢ�����
static	int	wari_flg;
//�֥쥤���ե饰�ݥ�����ɺ�������ߡ��ޤ��Ͻ�λ����Ƚ�Ǥ򤹤�
static	int	break_flg;
//�������ǥե饰����٤��Ť��ƽ������ɤ��Ĥ��ʤ����ޤ��Ͻ�λ����Ω��
static	int	abnml_flg;
//�񤭹��ߥХåե��ֹ�ݤɤ���ΥХåե����Ф��ƽ񤭹��ߤ򤹤�Τ�������
static	int	next_write;
//�Хåե��ĥ������ݥХåե��λĤ�Υ�����������
static	int	next_size;
//�ģͣ�ž��������
static	int	dma_req[2];
//�ģͣ����ɥ쥹�ơ��֥�ݥ����ͥ뤴�ȤΥ��ɥ쥹
static	int	dma_tbl[] = {
	0xffa00000, 0xffa00010, 0xffa00020, 0xffa00030,
	0xffa00040, 0xffa00050, 0xffa00060, 0xffa00070
};

/* --------------------------------------------------------------------- */
//�ģͣ���������
//�ģͣ��λ�������ͥ��ͭ���ˤ���
void	dma2_start(int irq)
{
int	base;

	base = dma_tbl[irq];
	*(volatile unsigned long *)(base + 0xc) |= 0x00000001;
}


//�ģͣ����ȥå�
//�ģͣ��λ�������ͥ��̵���ˤ���
void	dma2_stop(int irq)
{
int	base;

	base = dma_tbl[irq];
	*(volatile unsigned long *)(base + 0xc) &= 0xfffffffc;
}


//�ģͣ�ž������
//�ģͣ���ž�������ɥ쥹��ž���������򥻥åȤ���
void	dma2_queue_buffer(int irq,int src, int cnt)
{
int	base;

	base = dma_tbl[irq];
	*(volatile unsigned long *)(base + 0x0) = src & 0x1fffffff;
	//��Υ�뤫���ƥ쥪����ž��ñ�̤��Ѥ���
	if(setting.ch == 1) {
		//���ƥ쥪�ϣ��Х���ñ��
		*(volatile unsigned long *)(base + 0x8) = cnt;
	}
	else {
		//��Υ��ϣ��Х���ñ��
		*(volatile unsigned long *)(base + 0x8) = cnt * 2;
	}
}


//�ģͣ���Ͽ
//�����ͥ�ˣģͣ��γ����ߤ���Ͽ����
int	request_dma2(int irq,char *str,irqreturn_t (*callback)(int, void *, struct pt_regs *))
{
int	ret;

	make_ipr_irq(DMTE0_IRQ+irq, DMA_IPR_ADDR, DMA_IPR_POS,DMA_PRIORITY);
	ret = request_irq(DMTE0_IRQ+irq,callback,SA_INTERRUPT,str,0);
	return(ret);
}


//�ģͣ����ѽ���
//�ģͣ���ž���襢�ɥ쥹��ž��ñ�̡�ž�����ʤɤ򥻥åȤ���
void	dma2_set_device(int irq)
{
int	base;

	//DMA initialize
	base = dma_tbl[irq];
	*(volatile unsigned long *)(base + 0x0) = 0;
	*(volatile unsigned long *)(base + 0x4) = VOYAGER_8051_FIFO & 0x1fffffff;
	*(volatile unsigned long *)(base + 0x8) = 0;
	//��Υ�뤫���ƥ쥪����ž��ñ�̤��Ѥ���
	if(setting.ch == 1) {
		//���ƥ쥪�ϣ��Х���ñ��
		*(volatile unsigned long *)(base + 0xc) = 0xb1034;
	}
	else {
		//��Υ��ϣ��Х���ñ��
		*(volatile unsigned long *)(base + 0xc) = 0xb1024;
	}
	*(volatile unsigned long *)(base + 0x40) = 0x01;
}

/* --------------------------------------------------------------------- */


//������ɺ�����ߡʽ�λ��
//
static	void	voy_break(void)
{
int	data,data2;

	//��������¦�������
	data = 0x00;
	command_8051(0x07,&data,&data2);
	//�ģͣ����
	dma2_stop(sh7751_dmasound_play_irq);
	break_flg = 1;
}
/* --------------------------------------------------------------------- */
//�ģͣ��γ����ߥ롼����
//�ģͣ���ž����λ��˸ƤФ��
//Ϣ³ž����Ϥ����Ǽ���ž���ؼ���Ф�
static	irqreturn_t do_irq(int irq, void *dev_id, struct pt_regs *regs)
{
int	flags;

	save_flags(flags);
	cli();
	//�ģͣ����
	dma2_stop(sh7751_dmasound_play_irq);
	//�����ߥե饰���ꥢ
	wari_flg = 0;
	//�Хåե������ܤκ�����λ
	if((proc_flg == 1)&&(buff_flg1 == 1)) {
		//�Хåե�����������
		buff_flg1 = 0;
		//�Хåե����κ�����
		proc_flg = 2;
		//���˺�������ǡ������Ѱդ���Ƥ��롩
		if(buff_flg2 == 1) {
			//�ģͣ��˥ꥯ������
			dma2_queue_buffer(sh7751_dmasound_play_irq,VOYAGER_SOUND_BUF2,dma_req[1]);
			dma2_start(sh7751_dmasound_play_irq);
		}
		else {
			//����Ƥ��ʤ��ʤ�������ǥե饰��Ω�Ƥ�
			abnml_flg = 1;
		}
	}
	//�Хåե������ܤκ�����λ
	else if((proc_flg == 2)&&(buff_flg2 == 1)) {
		//�Хåե�����������
		buff_flg2 = 0;
		//�Хåե����κ�����
		proc_flg = 1;
		//���˺�������ǡ������Ѱդ���Ƥ��롩
		if(buff_flg1 == 1) {
			//�ģͣ��˥ꥯ������
			dma2_queue_buffer(sh7751_dmasound_play_irq,VOYAGER_SOUND_BUF1,dma_req[0]);
			dma2_start(sh7751_dmasound_play_irq);
		}
		else {
			//����Ƥ��ʤ��ʤ�������ǥե饰��Ω�Ƥ�
			abnml_flg = 1;
		}
	}
	//ü���ǡ����κ����ʺǸ�Υǡ����������ʥǡ�����
	else {
		//�ǽ�ž����λ
		last_flg = 0;
	}
	//���꡼�����write�롼����򵯤���
	wake_up_interruptible(&int_wq);
	restore_flags(flags);

	return IRQ_HANDLED;
}

/* --------------------------------------------------------------------- */
//�����ӥåȥ���꡼��
static inline u32 voyager_readl(u32 addr)
{
	return *(volatile unsigned long *)(addr);
}

//�����ӥåȥ���饤��
static inline void voyager_writel(u32 val,u32 addr)
{
	*(volatile unsigned long *)(addr) = val;
}

/* --------------------------------------------------------------------- */

struct voyagergx_state {
	/* soundcore stuff */
	int             dev_audio;

	struct ac97_codec codec;
	unsigned        codec_base_caps;	// AC'97 reg 00h, "Reset Register"
	unsigned        codec_ext_caps;		// AC'97 reg 28h, "Extended Audio ID"
	int             no_vra;			// do not use VRA

	spinlock_t      lock;
	struct semaphore open_sem;
	mode_t          open_mode;

} voyagergx_state;


/* --------------------------------------------------------------------- */
//�꡼�ɥ����ǥå�
static	u16 rdcodec(struct ac97_codec *codec, u8 addr)
{
	u32             cmd,ret;
	u16             data;


	//�ɤ߽Ф������쥸�������ɥ쥹�����
	cmd = (u32) (addr & AC97C_INDEX_MASK) << 12;
	cmd |= AC97C_READ;	// read command
	command_8051(1,&cmd,&ret);
	mdelay(1);
	//�쥸�������ɤ߽Ф�
	command_8051(4,&cmd,&ret);
	mdelay(1);
	//���եȤ��줿���֤ʤΤǸ����᤹
	data = (cmd >> 4) & 0xffff;
	return data;
}


//�饤�ȥ����ǥå�
static	void wrcodec(struct ac97_codec *codec, u8 addr, u16 data)
{
	u32             cmd,ret;
	int	i;

	//�ꥻ�åȤ��ä����
	if(addr == 0) {
		//�ꥻ�åȤ����ޤ������ʤ��ä����ϥ�ȥ饤����
		for(i=0;i<10;i++) {
			//�񤭹���ǡ����򥻥å�
			cmd = (u32) data << AC97C_WD_BIT;	// OR in the data word
			command_8051(3,&cmd,&ret);
			//�񤭹��ॢ�ɥ쥹�򥻥å�
			cmd = (u32) (addr & AC97C_INDEX_MASK) << 12;
			cmd &= ~AC97C_READ;	// write command
			command_8051(1,&cmd,&ret);
			//�ꥻ�åȤ����ޤ����ä�����ǧ
			ret = rdcodec(codec,0);
			if((ret & 0x8000) == 0) {
				break;
			}
		}
	}
	//����ʳ����ä����
	else {
		//�񤭹���ǡ����򥻥å�
		cmd = (u32) data << AC97C_WD_BIT;	// OR in the data word
		command_8051(3,&cmd,&ret);
		//�񤭹��ॢ�ɥ쥹�򥻥å�
		cmd = (u32) (addr & AC97C_INDEX_MASK) << 12;
		cmd &= ~AC97C_READ;	// write command
		command_8051(1,&cmd,&ret);
		mdelay(1);
		//�ܥ�塼��ط����ͤ��ä��ʤ��褦���ɤ߽Ф�
		rdcodec(codec,2);
		rdcodec(codec,24);
		rdcodec(codec,22);
		rdcodec(codec,0x2c);
	}
}

/* --------------------------------------------------------------------- */

static loff_t voyagergx_llseek(struct file *file, loff_t offset, int origin)
{
	return -ESPIPE;
}


static int voyagergx_open_mixdev(struct inode *inode, struct file *file)
{
	file->private_data = &voyagergx_state;
	return 0;
}

static int voyagergx_release_mixdev(struct inode *inode, struct file *file)
{
	return 0;
}

static int mixdev_ioctl(struct ac97_codec *codec, unsigned int cmd,
                        unsigned long arg)
{
	return codec->mixer_ioctl(codec, cmd, arg);
}

static int voyagergx_ioctl_mixdev(struct inode *inode, struct file *file,
			       unsigned int cmd, unsigned long arg)
{
	struct voyagergx_state *s = (struct voyagergx_state *)file->private_data;
	struct ac97_codec *codec = &s->codec;

	return mixdev_ioctl(codec, cmd, arg);
}

static /*const */ struct file_operations voyagergx_mixer_fops = {
	owner:THIS_MODULE,
	llseek:voyagergx_llseek,
	ioctl:voyagergx_ioctl_mixdev,
	open:voyagergx_open_mixdev,
	release:voyagergx_release_mixdev,
};

/* --------------------------------------------------------------------- */
//�꡼�ɥ롼����
//����ϥ쥳���ɡ�Ͽ���˽����˻���
//����ϥ쥳���ɡ�Ͽ���˽������ʤ��Τ���ȤϤʤ�
static ssize_t voyagergx_read(struct file *file, char *buffer,
			   size_t count, loff_t *ppos)
{
	if (ppos != &file->f_pos)
		return -ESPIPE;
	if (!access_ok(VERIFY_WRITE, buffer, count))
		return -EFAULT;
	return 0;
}

//�饤�ȥ롼����
//������ɺ����˻���
//������ɺ����ϣ����������Ԥ����ǡ������Τϣģͣ���ž������
//�����ǤϤ��ΰ٤ν�����ԤäƤ���
static ssize_t voyagergx_write(struct file *file, const char *buffer,
	     		    size_t count, loff_t * ppos)
{
int	i,data,data2,data_size;
int	ret;


	if (ppos != &file->f_pos)
		return -ESPIPE;
	if (!access_ok(VERIFY_READ, buffer, count))
		return -EFAULT;
	//��ߥե饰��Ω�äƤ�����¨��λ
	if(break_flg) {
		return count;
	}
	//�Хåե������ܤȤ���դξ������������ޤ��Ԥ�
	if((buff_flg1 == 1)&&(buff_flg2 == 1)) {
		wari_flg = 1;
		i = 0;
		while(wari_flg) {
			//���˺���������äƤ�����
			if(abnml_flg) {
				break;
			}
			//�Хåե��������ޤǵ�̲����
			ret = interruptible_sleep_on_timeout(&int_wq,WAIT_TIMEOUT);
			if(ret == 0) {
				break;
			}
			//����ȥ���å����å�
			if (signal_pending(current)) {
				voy_break();
				return -ERESTARTSYS;
			}
			i++;
			if(i > 10000000) {
				printk("DMA endcheck-1 error\n");
				voy_break();
				return -EFAULT;
			}
		}
	}
	data_size = count;
	//ʣ��ž���ξ��
	if(data_size >= next_size) {
		//�Хåե��λĤ�������ǡ����������礭���֥롼��
		while(data_size >= next_size) {
			//�Хåե���¦�������Ƥ���
			if((next_write == 1)&&(buff_flg1 == 0)) {
				//�֣ң��ͤ˥ǡ����򥳥ԡ�
				copy_from_user((long *)(VOYAGER_SOUND_BUF1+VOYAGER_SOUND_SIZE-next_size),
					(long *)buffer,next_size);
				//�Хåե���������
				buff_flg1 = 1;
				next_write = 2;
				//�ݥ��󥿹���
				buffer += next_size;
				//�����ǡ����Ĥꥵ�����򸺤餹
				data_size -= next_size;
				//�Хåե��Ĥꥵ�����ꥻ�å�
				next_size = VOYAGER_SOUND_SIZE;
				//�ǡ��������Ǥ��Ƥ�����Ϻ��٣ģͣ��˥��å�����
				if(abnml_flg == 1) {
					dma2_queue_buffer(sh7751_dmasound_play_irq,
							VOYAGER_SOUND_BUF1,dma_req[0]);
					dma2_start(sh7751_dmasound_play_irq);
					abnml_flg = 0;
				}
			}
			//�Хåե���¦�������Ƥ���
			else if((next_write == 2)&&(buff_flg2 == 0)) {
				//�֣ң��ͤ˥ǡ����򥳥ԡ�
				copy_from_user((long *)(VOYAGER_SOUND_BUF2+VOYAGER_SOUND_SIZE-next_size),
					(long *)buffer,next_size);
				//�Хåե���������
				buff_flg2 = 1;
				next_write = 1;
				//�ݥ��󥿹���
				buffer += next_size;
				//�����ǡ����Ĥꥵ�����򸺤餹
				data_size -= next_size;
				//�Хåե��Ĥꥵ�����ꥻ�å�
				next_size = VOYAGER_SOUND_SIZE;
				//�ǡ��������Ǥ��Ƥ�����Ϻ��٣ģͣ��˥��å�����
				if(abnml_flg == 1) {
					dma2_queue_buffer(sh7751_dmasound_play_irq,
						VOYAGER_SOUND_BUF2,dma_req[1]);
					dma2_start(sh7751_dmasound_play_irq);
					abnml_flg = 0;
				}
			}
			//���Ƥκ�����
			if(first_flg == 0) {
				//�Хåե��������ա��ʽ��Ƥ�ž����ɬ����¦��
				if(buff_flg1 == 1) {
					//�ģͣ��˥ꥯ������
					dma2_queue_buffer(sh7751_dmasound_play_irq,
       							VOYAGER_SOUND_BUF1,
       							VOYAGER_SOUND_SIZE/4);
					//��������¦��������
					data = 0x01;
					command_8051(0x07,&data,&data2);
					//�ģͣ�ž������
					dma2_start(sh7751_dmasound_play_irq);
					//���ե饰���å�
					first_flg = 1;
				}
			}
			//�Хåե������ܤȤ���դξ������������ޤ��Ԥ�
			if((buff_flg1 == 1)&&(buff_flg2 == 1)) {
				wari_flg = 1;
				i = 0;
				while(wari_flg) {
					//���˺���������äƤ�����
					if(abnml_flg) {
						break;
					}
					//�Хåե��������ޤǵ�̲����
					ret = interruptible_sleep_on_timeout(&int_wq,WAIT_TIMEOUT);
					if(ret == 0) {
						break;
					}
					//����ȥ���å����å�
					if (signal_pending(current)) {
						voy_break();
						return -ERESTARTSYS;
					}
					i++;
					if(i > 10000000) {
						printk("DMA endcheck-2 error\n");
						voy_break();
						return -EFAULT;
					}
				}
			}
		}
	}
	//�����ǡ������������Хåե��Ĥꥵ�������⾮�������
	if((data_size != 0)&&(data_size < next_size)) {
		//�Хåե���¦�������Ƥ���
		if((next_write == 1)&&(buff_flg1 == 0)) {
			//�֣ң��ͤ˥ǡ����򥳥ԡ�
			copy_from_user((long *)(VOYAGER_SOUND_BUF1+VOYAGER_SOUND_SIZE-next_size),
				(void *)buffer,data_size);
			//�Хåե��Ĥꥵ�����ꥻ�å�
			next_size -= data_size;
			//�Хåե������ա�
			if(next_size == 0) {
				//�Хåե���������
				buff_flg1 = 1;
				next_write = 2;
				//�Хåե��Ĥꥵ�����ꥻ�å�
				next_size = VOYAGER_SOUND_SIZE;
				//�ǡ��������Ǥ��Ƥ�����Ϻ��٣ģͣ��˥��å�����
				if(abnml_flg == 1) {
					dma2_queue_buffer(sh7751_dmasound_play_irq,
						VOYAGER_SOUND_BUF1,dma_req[0]);
					dma2_start(sh7751_dmasound_play_irq);
					abnml_flg = 0;
				}
			}
		}
		//�Хåե���¦�������Ƥ���
		else if((next_write == 2)&&(buff_flg2 == 0)) {
			//�֣ң��ͤ˥ǡ����򥳥ԡ�
			copy_from_user((long *)(VOYAGER_SOUND_BUF2+VOYAGER_SOUND_SIZE-next_size),
				(void *)buffer,data_size);
			//�Хåե��Ĥꥵ�����ꥻ�å�
			next_size -= data_size;
			//�Хåե������ա�
			if(next_size == 0) {
				//�Хåե���������
				buff_flg2 = 1;
				next_write = 1;
				//�Хåե��Ĥꥵ�����ꥻ�å�
				next_size = VOYAGER_SOUND_SIZE;
				//�ǡ��������Ǥ��Ƥ�����Ϻ��٣ģͣ��˥��å�����
				if(abnml_flg == 1) {
					dma2_queue_buffer(sh7751_dmasound_play_irq,
						VOYAGER_SOUND_BUF2,dma_req[1]);
					dma2_start(sh7751_dmasound_play_irq);
					abnml_flg = 0;
				}
			}
		}
	}
	//���Ƥκ�����
	if(first_flg == 0) {
		//�Хåե��������ա��ʽ��Ƥ�ž����ɬ����¦��
		if(buff_flg1 == 1) {
			//�ģͣ��˥ꥯ������
			dma2_queue_buffer(sh7751_dmasound_play_irq,
       					VOYAGER_SOUND_BUF1,
       					VOYAGER_SOUND_SIZE/4);
			//��������¦��������
			data = 0x01;
			command_8051(0x07,&data,&data2);
			dma2_start(sh7751_dmasound_play_irq);
			//���ե饰���å�
			first_flg = 1;
		}
	}
	return(count);
}

//�ݡ���󥰡�̤����
static unsigned int voyagergx_poll(struct file *file,
				struct poll_table_struct *wait)
{
	return 0;
}

//������̤����
static int voyagergx_mmap(struct file *file, struct vm_area_struct *vma)
{
	return 0;
}

//�ɣϥ���ȥ���롼����
//�ǡ���������ѹ���ܥ�塼������ʤɤ�Ԥ�
static int voyagergx_ioctl(struct inode *inode, struct file *file,
                        unsigned int cmd, unsigned long arg)
{
	struct voyagergx_state *s = (struct voyagergx_state *)file->private_data;
	int	data,data2,rate;


	switch (cmd) {
//Sound Stop control
	case -1:
		voy_break();
		return 0;
	case OSS_GETVERSION:
		return put_user(SOUND_VERSION, (int *) arg);

	case SNDCTL_DSP_SYNC:
		return 0;

	case SNDCTL_DSP_SETDUPLEX:
		return 0;

	case SNDCTL_DSP_GETCAPS:
		return put_user(DSP_CAP_DUPLEX | DSP_CAP_REALTIME |
				DSP_CAP_TRIGGER | DSP_CAP_MMAP, (int *)arg);

	case SNDCTL_DSP_RESET:
		return 0;

	case SNDCTL_DSP_SPEED:		//����ץ�󥰥졼������
		if((*(int *)arg <= 0xbb80)&&(*(int *)arg >= 0x1b80)) {
			setting.rate = *(int *)arg;
		}
		if(setting.ch == 0) {
			rate = setting.rate / 2;
		}
		else {
			rate = setting.rate;
		}
		return setting.rate;

	case SNDCTL_DSP_STEREO:		//���������ͥ������ʼºݤϥ��ƥ쥪����Υ�뤫�������
		setting.ch = *(int *)arg;
		//��������¦�˥����ͥ�����Τ餻��
		data = setting.ch;
		command_8051(0x05,&data,&data2);
        	dma2_set_device(sh7751_dmasound_play_irq);
		return(setting.ch);

	case SNDCTL_DSP_CHANNELS:
		return 0;

	case SNDCTL_DSP_GETFMTS:	//�����ǡ����ե����ޥåȥ��åȡʣ��ӥåȥǡ��� or �����ӥåȥǡ�����
		if(setting.fmt == 16) {
			(int *)arg = AFMT_S16_LE;
		}
		else {
			(int *)arg = AFMT_U8;
		}
		return 0;

	case SNDCTL_DSP_SETFMT:		//�����ǡ����ե����ޥåȥ��åȡʣ��ӥåȥǡ��� or �����ӥåȥǡ�����
		if(*(int *)arg == AFMT_S16_LE) {
			setting.fmt = 16;
		}
		else {
			setting.fmt = 8;
		}
		return 0;

	case SNDCTL_DSP_POST:
		return 0;

	case SNDCTL_DSP_GETTRIGGER:
		return 0;

	case SNDCTL_DSP_SETTRIGGER:
		return 0;

	case SNDCTL_DSP_GETOSPACE:
		return 0;

	case SNDCTL_DSP_GETISPACE:
		return 0;

	case SNDCTL_DSP_NONBLOCK:
		return 0;

	case SNDCTL_DSP_GETODELAY:
		return 0;

	case SNDCTL_DSP_GETIPTR:
		return 0;

	case SNDCTL_DSP_GETOPTR:
		return 0;

	case SNDCTL_DSP_GETBLKSIZE:
		return 0;

	case SNDCTL_DSP_SETFRAGMENT:
		return 0;

	case SNDCTL_DSP_SUBDIVIDE:
		return 0;

	case SOUND_PCM_READ_RATE:
		return 0;

	case SOUND_PCM_READ_CHANNELS:
		return 0;

	case SOUND_PCM_READ_BITS:
		return 0;

	case SOUND_PCM_WRITE_FILTER:
	case SNDCTL_DSP_SETSYNCRO:
	case SOUND_PCM_READ_FILTER:
		return -EINVAL;
	}

	return mixdev_ioctl(&s->codec, cmd, arg);
}

//�����ץ����
//�Ƽ�ե饰�����
//���������Ϻ�����ߡʵ�ư�ϥ饤�Ȥν��ǳݤ����
static int  voyagergx_open(struct inode *inode, struct file *file)
{
struct voyagergx_state *s = &voyagergx_state;
int	data,data2;

	file->private_data = s;

	s->open_mode |= file->f_mode & (FMODE_READ | FMODE_WRITE);
	//���ž���ե饰���ꥢ
	first_flg = 0;
	//�ǽ�ž���ե饰���ꥢ
	last_flg = 0;
	//�Хåե��ե饰���ꥢ
	buff_flg1 = buff_flg2 = 0;
	//�������ϥХåե���¦����
	proc_flg = 1;
	//�Хåե��Ĥꥵ�����ꥻ�å�
	next_size = VOYAGER_SOUND_SIZE;
	//�ģͣ�ž�������������
	dma_req[0] = VOYAGER_SOUND_SIZE/4;
	dma_req[1] = VOYAGER_SOUND_SIZE/4;
	//�֥쥤���ե饰���ꥢ
	break_flg = 0;
	//�������ǥե饰���ꥢ
	abnml_flg = 0;
	//�񤭹��ߥХåե��ֹ�����
	next_write = 1;
	//��������¦��������
	data = 0x00;
	command_8051(0x07,&data,&data2);

	return 0;
}

//��꡼���ʥ������˽���
//�Хåե��˺����ǡ�����¸�ߤ�����Ϻ���
//�ģͣ����
//���������������
static int voyagergx_release(struct inode *inode, struct file *file)
{
int	data,data2,i;
int	ret;

	//�֥쥤���ե饰��Ω�äƤ���ʤ�¨��λ
	if(break_flg) {
		return 0;
	}
	//�Хåե������ܤȤ���դξ������������ޤ��Ԥ�
	if((buff_flg1 == 1)&&(buff_flg2 == 1)) {
		wari_flg = 1;
		i = 0;
		while(wari_flg) {
			//���˺���������äƤ�����
			if(abnml_flg) {
				break;
			}
			//�Хåե��������ޤǵ�̲����
			ret = interruptible_sleep_on_timeout(&int_wq,WAIT_TIMEOUT);
			if(ret == 0) {
				break;
			}
			//����ȥ���å����å�
			if (signal_pending(current)) {
				voy_break();
				return -ERESTARTSYS;
			}
			i++;
			if(i > 10000000) {
				printk("DMA endcheck-3 error\n");
				voy_break();
				return -EFAULT;
			}
		}
	}
	//���������������Х��Ȥ������ʤ����Ϻ��������˽�λ
	if((VOYAGER_SOUND_SIZE - next_size) < 4) {
		voy_break();
		return 0;
	}
	//�Хåե��˶��������롩
	if(next_size != 0) {
		
		//���Ƥκ�����
		if(first_flg == 0) {
			//�����ǡ����ʤ���
			if(next_size == VOYAGER_SOUND_SIZE) {
				//��λ����
				voy_break();
				return 0;
			}
			//�����ϥХåե���¦��
			if(proc_flg == 1) {
      				dma2_queue_buffer(sh7751_dmasound_play_irq,
       						VOYAGER_SOUND_BUF1,
       						(VOYAGER_SOUND_SIZE-next_size)/4);
				//�ǽ�Ƚ��ե饰��Ω�Ƥ�
				last_flg = 1;
			}
			//��������¦��������
			data = 0x01;
			command_8051(0x07,&data,&data2);
			dma2_start(sh7751_dmasound_play_irq);
		}
		//�Хåե����ܤ������ʤ��ǡ����κ���
		else {
			if(proc_flg == 2) {
				dma_req[0] = (VOYAGER_SOUND_SIZE-next_size)/4;
				buff_flg1 = 1;
				//�ǽ�Ƚ��ե饰��Ω�Ƥ�
				last_flg = 1;
				//�ǡ��������Ǥ��Ƥ�����Ϻ��٣ģͣ��˥��å�����
				if(abnml_flg == 1) {
					dma2_queue_buffer(sh7751_dmasound_play_irq,
						VOYAGER_SOUND_BUF1,dma_req[0]);
					dma2_start(sh7751_dmasound_play_irq);
					abnml_flg = 0;
				}
			}
			else {
				//�ģͣ��˥ꥯ������
				dma_req[1] = (VOYAGER_SOUND_SIZE-next_size)/4;
				buff_flg2 = 1;
				//�ǽ�Ƚ��ե饰��Ω�Ƥ�
				last_flg = 1;
				//�ǡ��������Ǥ��Ƥ�����Ϻ��٣ģͣ��˥��å�����
				if(abnml_flg == 1) {
					dma2_queue_buffer(sh7751_dmasound_play_irq,
						VOYAGER_SOUND_BUF2,dma_req[1]);
					dma2_start(sh7751_dmasound_play_irq);
					abnml_flg = 0;
				}
			}
		}
	}
	//��������ž���ˤ������ޤ��Ԥ�
	while(1) {
		if(abnml_flg) {
			break;
		}
		//�ǽ�Ƚ��ե饰�����ʤ���������ž���˽�λ
		if(last_flg == 0) {
			break;
		}
		wari_flg = 1;
		i = 0;
		while(wari_flg) {
			//���˺���������äƤ�����
			if(abnml_flg) {
				break;
			}
			//�ǽ�Ƚ��ե饰�����ʤ���������ž���˽�λ
			if(last_flg == 0) {
				break;
			}
			//�Хåե��������ޤǵ�̲����
			ret = interruptible_sleep_on_timeout(&int_wq,WAIT_TIMEOUT);
			if(ret == 0) {
				voy_break();
				return -EFAULT;
			}
			//����ȥ���å����å�
			if (signal_pending(current)) {
				voy_break();
				return -ERESTARTSYS;
			}
			i++;
			if(i > 10000000) {
				printk("DMA endcheck-last error\n");
				voy_break();
				return -EFAULT;
			}
		}
	}
	//��λ
	voy_break();

	return 0;
}

static /*const */ struct file_operations voyagergx_audio_fops = {
	owner:		THIS_MODULE,
	llseek:		voyagergx_llseek,
	read:		voyagergx_read,
	write:		voyagergx_write,
	poll:		voyagergx_poll,
	ioctl:		voyagergx_ioctl,
	mmap:		voyagergx_mmap,
	open:		voyagergx_open,
	release:	voyagergx_release,
};


/* --------------------------------------------------------------------- */
MODULE_AUTHOR("Atom Create Engineering Co.,Ltd.");
MODULE_DESCRIPTION("DSP audio and mixer driver for Silicon Motion VoyagerGX audio device"); 
  
/* --------------------------------------------------------------------- */

//�ץ��ֽ���
//�ɥ饤�н����
//�����ǥå������
//���ã����Υ��åפ�ɽ��
static int __devinit voyagergx_probe(void)
{
struct voyagergx_state *s = &voyagergx_state;
int             val;

	memset(s, 0, sizeof(struct voyagergx_state));

	init_MUTEX(&s->open_sem);
	s->codec.private_data = s;
	s->codec.id = 0;
	s->codec.codec_read = rdcodec;
	s->codec.codec_write = wrcodec;
	s->codec.codec_wait = NULL;

	/* register devices */

	if ((s->dev_audio = register_sound_dsp(&voyagergx_audio_fops, -1)) < 0)
		goto err_dev1;
	if ((s->codec.dev_mixer =
	     register_sound_mixer(&voyagergx_mixer_fops, -1)) < 0)
		goto err_dev2;


	/* codec init */
	if (!ac97_probe_codec(&s->codec))
		goto err_dev3;

	s->codec_base_caps = rdcodec(&s->codec, AC97_RESET);
	s->codec_ext_caps = rdcodec(&s->codec, AC97_EXTENDED_ID);
	info("AC'97 Base/Extended ID = %04x/%04x",
	     s->codec_base_caps, s->codec_ext_caps);

	s->codec.supported_mixers |= SOUND_MASK_ALTPCM;
	val = 0x4343;
	mixdev_ioctl(&s->codec, SOUND_MIXER_WRITE_ALTPCM,
		     (unsigned long) &val);
	
	if (!(s->codec_ext_caps & AC97_EXTID_VRA)) {
		// codec does not support VRA
		s->no_vra = 1;
	} else if (!vra) {
		// Boot option says disable VRA
		u16 ac97_extstat = rdcodec(&s->codec, AC97_EXTENDED_STATUS);
		wrcodec(&s->codec, AC97_EXTENDED_STATUS,
			ac97_extstat & ~AC97_EXTSTAT_VRA);
		s->no_vra = 1;
	}
	if (s->no_vra)
		info("no VRA, interpolating and decimating");

	//����ץ�󥰥졼�Ƚ�����ʣ������
	setting.rate  = 48000;
	wrcodec(&s->codec, 0x2a, 1);
	wrcodec(&s->codec, 0x2c, setting.rate);
	//volume set
	wrcodec(&s->codec, 2, 0);
	wrcodec(&s->codec, 24, 0);
	wrcodec(&s->codec, 22, 0);

	return 0;

 err_dev3:
	unregister_sound_mixer(s->codec.dev_mixer);
 err_dev2:
	unregister_sound_dsp(s->dev_audio);
 err_dev1:
	return -1;
}

//��ࡼ�ֽ���
static void __devinit voyagergx_remove(void)
{
	struct voyagergx_state *s = &voyagergx_state;

	if (!s)
		return;
	unregister_sound_dsp(s->dev_audio);
	unregister_sound_mixer(s->codec.dev_mixer);
}

//���������
static	int __init init_voyagergx(void)
{
unsigned long	value;
int	err;

	info("sakuma@ace-jp.com, built " __TIME__ " on " __DATE__);

	//�ǣУɣϤ���ã����ȣ���������Ȥ�����ˤ���
	value = *(volatile unsigned long *)(GPIO_MUX_LOW);
	value |= GPIO_MUX_LOW_AC97 | GPIO_MUX_LOW_8051;
	*(volatile unsigned long *)(GPIO_MUX_LOW) = value;

	//�ģͣ����
	dma2_stop(sh7751_dmasound_play_irq);

	//DMA interrupt request
	//�ģͣ��γ����ߤ���Ͽ
        err = request_dma2(sh7751_dmasound_play_irq, "voyager DMA",do_irq);
        if (err) {
                return 0;
        }

	//���ã����γ����ߤ�ͭ���ˤ���
	value = *(volatile unsigned long *)(VOYAGER_INT_MASK);
	value |= VOYAGER_INT_MASK_AC;
	*(volatile unsigned long *)(VOYAGER_INT_MASK) = value;

	//���ã���������
	value = *(volatile unsigned long *)(POWER_MODE0_GATE);
	value |= POWER_MODE0_GATE_AC;
	*(volatile unsigned long *)(POWER_MODE0_GATE) = value;

	//���ã���������
	value = *(volatile unsigned long *)(POWER_MODE1_GATE);
	value |= POWER_MODE1_GATE_AC;
	*(volatile unsigned long *)(POWER_MODE1_GATE) = value;

	//���ã�����ͭ���ˤ���
	value = *(volatile unsigned long *)(AC97_CONTROL_STATUS);
	value |= 0x0000000F;
	*(volatile unsigned long *)(AC97_CONTROL_STATUS) = value;
	//��������Ԥ�
	mdelay(2);
	//�ꥻ�åȲ��
	value &= 0xFFFFFFF9;
	*(volatile unsigned long *)(AC97_CONTROL_STATUS) = value;

	//�ԣ��Ǥν�����ʣӣ̣ϣԣ�������ͭ���ˤ����
	value = *(volatile unsigned long *)(AC97_TX_SLOT0);
	value |= 0x0000F800;
	*(volatile unsigned long *)(AC97_TX_SLOT0) = value;

	//������֤ϥ�Υ�롢�����ӥåȡ�������
	setting.ch = 0;
	setting.fmt = 16;
	setting.rate = 48000;

	//�ģͣ������
        dma2_set_device(sh7751_dmasound_play_irq);

	//�������������
	init_8051();

	return voyagergx_probe();
}

//������ɽ���
static void __exit cleanup_voyagergx(void)
{
	info("unloading");
	voyagergx_remove();
}

module_init(init_voyagergx);
module_exit(cleanup_voyagergx);
MODULE_LICENSE("GPL");
#endif

/* ############################################################### */
/* ############################################################### */
/* ############################################################### */
/* ############################################################### */
/* ############################################################### */
/* ############################################################### */
/* ############################################################### */
/* ############################################################### */
/* ############################################################### */
/* ############################################################### */
/* ############################################################### */

/*
 * sm501_ac97_sound.c -- voyager sound driver
 *
 *     Copyright (c) 2005 Bill Gatliff <bgat@billgatliff.com>
 *
 * A complete rewrite of code originally developed by:
 *
 *     Copyright (C) 2003 Renesas Technology Sales Co.,Ltd.
 *     Copyright (C) 2003 Atom Create Engineering Co.,Ltd.
 *     Anthor : Atom Create Engineering Co.,Ltd.
 *                   Kenichi Sakuma
 *
 *  This file is subject to the terms and conditions of the GNU
 *  General Public License. See the file COPYING in the main directory
 *  of this archive for more details.
 *
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/sound.h>
#include <linux/slab.h>
#include <linux/soundcard.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/pci.h>
#include <linux/bitops.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/smp_lock.h>
#include <linux/ac97_codec.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/hardirq.h>

/* TODO: for now, share the video header file even though we don't
   really care about (move header file to somewhere more generic) */
#include <video/voyagergx_reg.h>

#include <asm/irq.h>
#include "voyagergx_sound.h"

#define SM501_AC97_DEVICENAME "sm501_ac97"
#define PFX SM501_AC97_DEVICENAME

#define DEBUG
#if defined(DEBUG)
#define dbg(format, arg...) \
        printk(KERN_DEBUG "%s(%s:%d): " format "\n" , __FUNCTION__, __FILE__, __LINE__, ## arg)
#else
#define dbg(format, arg...)
#endif

#define err(format, arg...) \
        printk(KERN_ERR "%s(%s:%d): " format "\n" , __FUNCTION__, __FILE__, __LINE__, ## arg)
#define info(format, arg...) printk(KERN_INFO PFX ": " format "\n" , ## arg)



#if 0

static u16 rdcodec (struct ac97_codec *codec, u8 addr)
{
	u32             cmd,ret;
	u16             data;

	cmd = (u32) (addr & AC97C_INDEX_MASK) << 12;
	cmd |= AC97C_READ;
	command_8051(1,&cmd,&ret);
	mdelay(1);
	command_8051(4,&cmd,&ret);
	mdelay(1);
	data = (cmd >> 4) & 0xffff;
	return data;
}


//�饤�ȥ����ǥå�
static	void wrcodec(struct ac97_codec *codec, u8 addr, u16 data)
{
	u32             cmd,ret;
	int	i;

	//�ꥻ�åȤ��ä����
	if(addr == 0) {
		//�ꥻ�åȤ����ޤ������ʤ��ä����ϥ�ȥ饤����
		for(i=0;i<10;i++) {
			//�񤭹���ǡ����򥻥å�
			cmd = (u32) data << AC97C_WD_BIT;	// OR in the data word
			command_8051(3,&cmd,&ret);
			//�񤭹��ॢ�ɥ쥹�򥻥å�
			cmd = (u32) (addr & AC97C_INDEX_MASK) << 12;
			cmd &= ~AC97C_READ;	// write command
			command_8051(1,&cmd,&ret);
			//�ꥻ�åȤ����ޤ����ä�����ǧ
			ret = rdcodec(codec,0);
			if((ret & 0x8000) == 0) {
				break;
			}
		}
	}
	//����ʳ����ä����
	else {
		//�񤭹���ǡ����򥻥å�
		cmd = (u32) data << AC97C_WD_BIT;	// OR in the data word
		command_8051(3,&cmd,&ret);
		//�񤭹��ॢ�ɥ쥹�򥻥å�
		cmd = (u32) (addr & AC97C_INDEX_MASK) << 12;
		cmd &= ~AC97C_READ;	// write command
		command_8051(1,&cmd,&ret);
		mdelay(1);
		//�ܥ�塼��ط����ͤ��ä��ʤ��褦���ɤ߽Ф�
		rdcodec(codec,2);
		rdcodec(codec,24);
		rdcodec(codec,22);
		rdcodec(codec,0x2c);
	}
}

#endif


static loff_t sm501_ac97_llseek (struct file *file, loff_t offset, int origin)
{
	return -ESPIPE;
}


static int sm501_ac97_open_mixdev (struct inode *inode, struct file *file)
{
	return 0;
}

static int sm501_ac97_release_mixdev (struct inode *inode, struct file *file)
{
	return 0;
}

static int mixdev_ioctl (struct ac97_codec *codec, unsigned int cmd,
                         unsigned long arg)
{
	return 0;
}

static int sm501_ac97_ioctl_mixdev (struct inode *inode, struct file *file,
                                   unsigned int cmd, unsigned long arg)
{
	return 0;
}

static struct file_operations sm501_ac97_mixer_fops = {
  owner:        THIS_MODULE,
  llseek:       sm501_ac97_llseek,
  ioctl:        sm501_ac97_ioctl_mixdev,
  open:         sm501_ac97_open_mixdev,
  release:      sm501_ac97_release_mixdev,
};

static ssize_t sm501_ac97_read (struct file *file, char *buffer,
                               size_t count, loff_t *ppos)
{
	if (ppos != &file->f_pos)
		return -ESPIPE;
	if (!access_ok(VERIFY_WRITE, buffer, count))
		return -EFAULT;
	return 0;
}

static ssize_t sm501_ac97_write (struct file *file, const char *buffer,
                                size_t count, loff_t * ppos)
{
	if (ppos != &file->f_pos)
		return -ESPIPE;
	if (!access_ok(VERIFY_READ, buffer, count))
		return -EFAULT;
	return count;
}

static unsigned int sm501_ac97_poll (struct file *file,
                                    struct poll_table_struct *wait)
{
	return 0;
}

static int sm501_ac97_mmap (struct file *file, struct vm_area_struct *vma)
{
	return 0;
}

static int sm501_ac97_ioctl (struct inode *inode, struct file *file,
                            unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case -1:
		return 0;
	case OSS_GETVERSION:
		return put_user(SOUND_VERSION, (int *) arg);

	case SNDCTL_DSP_SYNC:
		return 0;

	case SNDCTL_DSP_SETDUPLEX:
		return 0;

	case SNDCTL_DSP_GETCAPS:
		return put_user(DSP_CAP_DUPLEX | DSP_CAP_REALTIME |
				DSP_CAP_TRIGGER | DSP_CAP_MMAP, (int *)arg);

	case SNDCTL_DSP_RESET:
		return 0;

	case SNDCTL_DSP_SPEED:
		return 0;

	case SNDCTL_DSP_STEREO:
		return 0;

	case SNDCTL_DSP_CHANNELS:
		return 0;

	case SNDCTL_DSP_GETFMTS:
		return 0;

	case SNDCTL_DSP_SETFMT:
		return 0;

	case SNDCTL_DSP_POST:
		return 0;

	case SNDCTL_DSP_GETTRIGGER:
		return 0;

	case SNDCTL_DSP_SETTRIGGER:
		return 0;

	case SNDCTL_DSP_GETOSPACE:
		return 0;

	case SNDCTL_DSP_GETISPACE:
		return 0;

	case SNDCTL_DSP_NONBLOCK:
		return 0;

	case SNDCTL_DSP_GETODELAY:
		return 0;

	case SNDCTL_DSP_GETIPTR:
		return 0;

	case SNDCTL_DSP_GETOPTR:
		return 0;

	case SNDCTL_DSP_GETBLKSIZE:
		return 0;

	case SNDCTL_DSP_SETFRAGMENT:
		return 0;

	case SNDCTL_DSP_SUBDIVIDE:
		return 0;

	case SOUND_PCM_READ_RATE:
		return 0;

	case SOUND_PCM_READ_CHANNELS:
		return 0;

	case SOUND_PCM_READ_BITS:
		return 0;

	case SOUND_PCM_WRITE_FILTER:
	case SNDCTL_DSP_SETSYNCRO:
	case SOUND_PCM_READ_FILTER:
		return -EINVAL;
	}

	return 0;
}

static int  sm501_ac97_open (struct inode *inode, struct file *file)
{
	return 0;
}

static int sm501_ac97_release (struct inode *inode, struct file *file)
{
	return 0;
}

static struct file_operations sm501_ac97_fops = {
	owner:		THIS_MODULE,
	llseek:		sm501_ac97_llseek,
	read:		sm501_ac97_read,
	write:		sm501_ac97_write,
	poll:		sm501_ac97_poll,
	ioctl:		sm501_ac97_ioctl,
	mmap:		sm501_ac97_mmap,
	open:		sm501_ac97_open,
	release:	sm501_ac97_release,
};







static int __devinit
sm501_ac97_probe (struct device *dev)
{
  int ret;

  err("unimplemented\n");

  ret = register_sound_dsp(&sm501_ac97_fops, -1);
  dbg("register_sound_dsp returned: %d\n", ret);
  ret = register_sound_mixer(&sm501_ac97_mixer_fops, -1);
  dbg("register_sound_mixer returned: %d\n", ret);

  return 0;
}

static int sm501_ac97_suspend (struct device *dev, u32 state, u32 level)
{
  err("unimplemented\n");
  return 0;
}

static int sm501_ac97_resume (struct device *dev, u32 level)
{
  err("unimplemented\n");
  return 0;
}

static void __devinit sm501_ac97_remove (void)
{
  return;
}


static struct device_driver sm501_ac97_driver = {
  .name = SM501_AC97_DEVICENAME,
  .bus = &platform_bus_type,
  .probe = sm501_ac97_probe,
  .remove = sm501_ac97_remove,
  .suspend = sm501_ac97_suspend,
  .resume = sm501_ac97_resume,
};


static	int __init init_sm501_ac97 (void)
{
  /* TODO: for now, no command line parameters */
  int ret = driver_register(&sm501_ac97_driver);
  dbg("returning %d\n", ret);
  return ret;
}

static void __exit cleanup_sm501_ac97(void)
{
  sm501_ac97_remove();
  driver_unregister(&sm501_ac97_driver);
}


module_init(init_sm501_ac97);
module_exit(cleanup_sm501_ac97);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Audio driver for SM501 AC97");
MODULE_AUTHOR("Bill Gatliff <bgat@billgatliff.com>");


