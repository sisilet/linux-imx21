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

//バッファサイズ
#define	VOYAGER_SOUND_SIZE	0x100000
//データバッファ１−ＶＲＡＭ上
#define	VOYAGER_SOUND_BUF1	0xb0200000
//データバッファ２−ＶＲＡＭ上
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

//８０５１初期化関数
int	init_8051(void);
//８０５１コマンド起動関数
int	command_8051(int com,int *data,int *data2);

//ＤＭＡ使用チャンネル−ＤＭＡの使用チャンネルが入る（０固定）
static	int	sh7751_dmasound_play_irq = 0;
//バッファ１使用フラグ−使用中の時は転送サイズがセットされる
static	int	buff_flg1 = 0;
//バッファ２使用フラグ−使用中の時は転送サイズがセットされる
static	int	buff_flg2 = 0;
//バッファ判定フラグ−ＤＭＡがどちらのバッファを使っているのかが入る
static	int	proc_flg = 0;
//初回判定フラグ−データ転送の初回を判断する
static	int	first_flg = 0;
//最終判定フラグ−データ転送の最後を判断する
static	int	last_flg = 0;
//割り込みフラグ−割り込みが発生すると０クリアされる
static	int	wari_flg;
//ブレイクフラグ−サウンド再生の中止、または終了時の判断をする
static	int	break_flg;
//再生中断フラグ−負荷が重くて処理が追いつかない、または終了時に立つ
static	int	abnml_flg;
//書き込みバッファ番号−どちらのバッファに対して書き込みをするのかが入る
static	int	next_write;
//バッファ残サイズ−バッファの残りのサイズが入る
static	int	next_size;
//ＤＭＡ転送サイズ
static	int	dma_req[2];
//ＤＭＡアドレステーブル−チャンネルごとのアドレス
static	int	dma_tbl[] = {
	0xffa00000, 0xffa00010, 0xffa00020, 0xffa00030,
	0xffa00040, 0xffa00050, 0xffa00060, 0xffa00070
};

/* --------------------------------------------------------------------- */
//ＤＭＡスタート
//ＤＭＡの指定チャンネルを有効にする
void	dma2_start(int irq)
{
int	base;

	base = dma_tbl[irq];
	*(volatile unsigned long *)(base + 0xc) |= 0x00000001;
}


//ＤＭＡストップ
//ＤＭＡの指定チャンネルを無効にする
void	dma2_stop(int irq)
{
int	base;

	base = dma_tbl[irq];
	*(volatile unsigned long *)(base + 0xc) &= 0xfffffffc;
}


//ＤＭＡ転送準備
//ＤＭＡに転送元アドレスと転送サイズをセットする
void	dma2_queue_buffer(int irq,int src, int cnt)
{
int	base;

	base = dma_tbl[irq];
	*(volatile unsigned long *)(base + 0x0) = src & 0x1fffffff;
	//モノラルかステレオかで転送単位を変える
	if(setting.ch == 1) {
		//ステレオは４バイト単位
		*(volatile unsigned long *)(base + 0x8) = cnt;
	}
	else {
		//モノラルは２バイト単位
		*(volatile unsigned long *)(base + 0x8) = cnt * 2;
	}
}


//ＤＭＡ登録
//カーネルにＤＭＡの割り込みを登録する
int	request_dma2(int irq,char *str,irqreturn_t (*callback)(int, void *, struct pt_regs *))
{
int	ret;

	make_ipr_irq(DMTE0_IRQ+irq, DMA_IPR_ADDR, DMA_IPR_POS,DMA_PRIORITY);
	ret = request_irq(DMTE0_IRQ+irq,callback,SA_INTERRUPT,str,0);
	return(ret);
}


//ＤＭＡ使用準備
//ＤＭＡの転送先アドレスや転送単位、転送条件などをセットする
void	dma2_set_device(int irq)
{
int	base;

	//DMA initialize
	base = dma_tbl[irq];
	*(volatile unsigned long *)(base + 0x0) = 0;
	*(volatile unsigned long *)(base + 0x4) = VOYAGER_8051_FIFO & 0x1fffffff;
	*(volatile unsigned long *)(base + 0x8) = 0;
	//モノラルかステレオかで転送単位を変える
	if(setting.ch == 1) {
		//ステレオは４バイト単位
		*(volatile unsigned long *)(base + 0xc) = 0xb1034;
	}
	else {
		//モノラルは２バイト単位
		*(volatile unsigned long *)(base + 0xc) = 0xb1024;
	}
	*(volatile unsigned long *)(base + 0x40) = 0x01;
}

/* --------------------------------------------------------------------- */


//サウンド再生中止（終了）
//
static	void	voy_break(void)
{
int	data,data2;

	//８０５１側再生中止
	data = 0x00;
	command_8051(0x07,&data,&data2);
	//ＤＭＡ停止
	dma2_stop(sh7751_dmasound_play_irq);
	break_flg = 1;
}
/* --------------------------------------------------------------------- */
//ＤＭＡの割り込みルーチン
//ＤＭＡの転送終了後に呼ばれる
//連続転送中はここで次の転送指示を出す
static	irqreturn_t do_irq(int irq, void *dev_id, struct pt_regs *regs)
{
int	flags;

	save_flags(flags);
	cli();
	//ＤＭＡ停止
	dma2_stop(sh7751_dmasound_play_irq);
	//割り込みフラグクリア
	wari_flg = 0;
	//バッファ１本目の再生終了
	if((proc_flg == 1)&&(buff_flg1 == 1)) {
		//バッファ１が空いた
		buff_flg1 = 0;
		//バッファ２の再生中
		proc_flg = 2;
		//次に再生するデータが用意されている？
		if(buff_flg2 == 1) {
			//ＤＭＡにリクエスト
			dma2_queue_buffer(sh7751_dmasound_play_irq,VOYAGER_SOUND_BUF2,dma_req[1]);
			dma2_start(sh7751_dmasound_play_irq);
		}
		else {
			//されていないなら再生中断フラグを立てる
			abnml_flg = 1;
		}
	}
	//バッファ２本目の再生終了
	else if((proc_flg == 2)&&(buff_flg2 == 1)) {
		//バッファ２が空いた
		buff_flg2 = 0;
		//バッファ１の再生中
		proc_flg = 1;
		//次に再生するデータが用意されている？
		if(buff_flg1 == 1) {
			//ＤＭＡにリクエスト
			dma2_queue_buffer(sh7751_dmasound_play_irq,VOYAGER_SOUND_BUF1,dma_req[0]);
			dma2_start(sh7751_dmasound_play_irq);
		}
		else {
			//されていないなら再生中断フラグを立てる
			abnml_flg = 1;
		}
	}
	//端数データの再生（最後のデータか小さなデータ）
	else {
		//最終転送終了
		last_flg = 0;
	}
	//スリープ中のwriteルーチンを起こす
	wake_up_interruptible(&int_wq);
	restore_flags(flags);

	return IRQ_HANDLED;
}

/* --------------------------------------------------------------------- */
//３２ビットメモリリード
static inline u32 voyager_readl(u32 addr)
{
	return *(volatile unsigned long *)(addr);
}

//３２ビットメモリライト
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
//リードコーデック
static	u16 rdcodec(struct ac97_codec *codec, u8 addr)
{
	u32             cmd,ret;
	u16             data;


	//読み出したいレジスタアドレスを指定
	cmd = (u32) (addr & AC97C_INDEX_MASK) << 12;
	cmd |= AC97C_READ;	// read command
	command_8051(1,&cmd,&ret);
	mdelay(1);
	//レジスタを読み出す
	command_8051(4,&cmd,&ret);
	mdelay(1);
	//シフトされた状態なので元に戻す
	data = (cmd >> 4) & 0xffff;
	return data;
}


//ライトコーデック
static	void wrcodec(struct ac97_codec *codec, u8 addr, u16 data)
{
	u32             cmd,ret;
	int	i;

	//リセットだった場合
	if(addr == 0) {
		//リセットがうまくいかなかった場合はリトライする
		for(i=0;i<10;i++) {
			//書き込むデータをセット
			cmd = (u32) data << AC97C_WD_BIT;	// OR in the data word
			command_8051(3,&cmd,&ret);
			//書き込むアドレスをセット
			cmd = (u32) (addr & AC97C_INDEX_MASK) << 12;
			cmd &= ~AC97C_READ;	// write command
			command_8051(1,&cmd,&ret);
			//リセットがうまくいったか確認
			ret = rdcodec(codec,0);
			if((ret & 0x8000) == 0) {
				break;
			}
		}
	}
	//それ以外だった場合
	else {
		//書き込むデータをセット
		cmd = (u32) data << AC97C_WD_BIT;	// OR in the data word
		command_8051(3,&cmd,&ret);
		//書き込むアドレスをセット
		cmd = (u32) (addr & AC97C_INDEX_MASK) << 12;
		cmd &= ~AC97C_READ;	// write command
		command_8051(1,&cmd,&ret);
		mdelay(1);
		//ボリューム関係の値が消えないように読み出す
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
//リードルーチン
//本来はレコード（録音）処理に使用
//今回はレコード（録音）処理がないので中身はなし
static ssize_t voyagergx_read(struct file *file, char *buffer,
			   size_t count, loff_t *ppos)
{
	if (ppos != &file->f_pos)
		return -ESPIPE;
	if (!access_ok(VERIFY_WRITE, buffer, count))
		return -EFAULT;
	return 0;
}

//ライトルーチン
//サウンド再生に使用
//サウンド再生は８０５１が行い、データ自体はＤＭＡが転送する
//ここではその為の準備を行っている
static ssize_t voyagergx_write(struct file *file, const char *buffer,
	     		    size_t count, loff_t * ppos)
{
int	i,data,data2,data_size;
int	ret;


	if (ppos != &file->f_pos)
		return -ESPIPE;
	if (!access_ok(VERIFY_READ, buffer, count))
		return -EFAULT;
	//中止フラグが立っていたら即終了
	if(break_flg) {
		return count;
	}
	//バッファが２本とも一杯の場合空きが出来るまで待つ
	if((buff_flg1 == 1)&&(buff_flg2 == 1)) {
		wari_flg = 1;
		i = 0;
		while(wari_flg) {
			//既に再生が終わっている場合
			if(abnml_flg) {
				break;
			}
			//バッファが空くまで休眠する
			ret = interruptible_sleep_on_timeout(&int_wq,WAIT_TIMEOUT);
			if(ret == 0) {
				break;
			}
			//コントロールＣチェック
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
	//複数転送の場合
	if(data_size >= next_size) {
		//バッファの残りより再生データの方が大きい間ループ
		while(data_size >= next_size) {
			//バッファ１側が空いている
			if((next_write == 1)&&(buff_flg1 == 0)) {
				//ＶＲＡＭにデータをコピー
				copy_from_user((long *)(VOYAGER_SOUND_BUF1+VOYAGER_SOUND_SIZE-next_size),
					(long *)buffer,next_size);
				//バッファ１使用中
				buff_flg1 = 1;
				next_write = 2;
				//ポインタ更新
				buffer += next_size;
				//再生データ残りサイズを減らす
				data_size -= next_size;
				//バッファ残りサイズリセット
				next_size = VOYAGER_SOUND_SIZE;
				//データが中断している場合は再度ＤＭＡにキックする
				if(abnml_flg == 1) {
					dma2_queue_buffer(sh7751_dmasound_play_irq,
							VOYAGER_SOUND_BUF1,dma_req[0]);
					dma2_start(sh7751_dmasound_play_irq);
					abnml_flg = 0;
				}
			}
			//バッファ２側が空いている
			else if((next_write == 2)&&(buff_flg2 == 0)) {
				//ＶＲＡＭにデータをコピー
				copy_from_user((long *)(VOYAGER_SOUND_BUF2+VOYAGER_SOUND_SIZE-next_size),
					(long *)buffer,next_size);
				//バッファ２使用中
				buff_flg2 = 1;
				next_write = 1;
				//ポインタ更新
				buffer += next_size;
				//再生データ残りサイズを減らす
				data_size -= next_size;
				//バッファ残りサイズリセット
				next_size = VOYAGER_SOUND_SIZE;
				//データが中断している場合は再度ＤＭＡにキックする
				if(abnml_flg == 1) {
					dma2_queue_buffer(sh7751_dmasound_play_irq,
						VOYAGER_SOUND_BUF2,dma_req[1]);
					dma2_start(sh7751_dmasound_play_irq);
					abnml_flg = 0;
				}
			}
			//初めての再生？
			if(first_flg == 0) {
				//バッファ１が一杯？（初めての転送は必ず１側）
				if(buff_flg1 == 1) {
					//ＤＭＡにリクエスト
					dma2_queue_buffer(sh7751_dmasound_play_irq,
       							VOYAGER_SOUND_BUF1,
       							VOYAGER_SOUND_SIZE/4);
					//８０５１側再生開始
					data = 0x01;
					command_8051(0x07,&data,&data2);
					//ＤＭＡ転送開始
					dma2_start(sh7751_dmasound_play_irq);
					//初回フラグセット
					first_flg = 1;
				}
			}
			//バッファが２本とも一杯の場合空きが出来るまで待つ
			if((buff_flg1 == 1)&&(buff_flg2 == 1)) {
				wari_flg = 1;
				i = 0;
				while(wari_flg) {
					//既に再生が終わっている場合
					if(abnml_flg) {
						break;
					}
					//バッファが空くまで休眠する
					ret = interruptible_sleep_on_timeout(&int_wq,WAIT_TIMEOUT);
					if(ret == 0) {
						break;
					}
					//コントロールＣチェック
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
	//再生データサイズがバッファ残りサイズよりも小さい場合
	if((data_size != 0)&&(data_size < next_size)) {
		//バッファ１側が空いている
		if((next_write == 1)&&(buff_flg1 == 0)) {
			//ＶＲＡＭにデータをコピー
			copy_from_user((long *)(VOYAGER_SOUND_BUF1+VOYAGER_SOUND_SIZE-next_size),
				(void *)buffer,data_size);
			//バッファ残りサイズリセット
			next_size -= data_size;
			//バッファが一杯？
			if(next_size == 0) {
				//バッファ１使用中
				buff_flg1 = 1;
				next_write = 2;
				//バッファ残りサイズリセット
				next_size = VOYAGER_SOUND_SIZE;
				//データが中断している場合は再度ＤＭＡにキックする
				if(abnml_flg == 1) {
					dma2_queue_buffer(sh7751_dmasound_play_irq,
						VOYAGER_SOUND_BUF1,dma_req[0]);
					dma2_start(sh7751_dmasound_play_irq);
					abnml_flg = 0;
				}
			}
		}
		//バッファ２側が空いている
		else if((next_write == 2)&&(buff_flg2 == 0)) {
			//ＶＲＡＭにデータをコピー
			copy_from_user((long *)(VOYAGER_SOUND_BUF2+VOYAGER_SOUND_SIZE-next_size),
				(void *)buffer,data_size);
			//バッファ残りサイズリセット
			next_size -= data_size;
			//バッファが一杯？
			if(next_size == 0) {
				//バッファ２使用中
				buff_flg2 = 1;
				next_write = 1;
				//バッファ残りサイズリセット
				next_size = VOYAGER_SOUND_SIZE;
				//データが中断している場合は再度ＤＭＡにキックする
				if(abnml_flg == 1) {
					dma2_queue_buffer(sh7751_dmasound_play_irq,
						VOYAGER_SOUND_BUF2,dma_req[1]);
					dma2_start(sh7751_dmasound_play_irq);
					abnml_flg = 0;
				}
			}
		}
	}
	//初めての再生？
	if(first_flg == 0) {
		//バッファ１が一杯？（初めての転送は必ず１側）
		if(buff_flg1 == 1) {
			//ＤＭＡにリクエスト
			dma2_queue_buffer(sh7751_dmasound_play_irq,
       					VOYAGER_SOUND_BUF1,
       					VOYAGER_SOUND_SIZE/4);
			//８０５１側再生開始
			data = 0x01;
			command_8051(0x07,&data,&data2);
			dma2_start(sh7751_dmasound_play_irq);
			//初回フラグセット
			first_flg = 1;
		}
	}
	return(count);
}

//ポーリング−未使用
static unsigned int voyagergx_poll(struct file *file,
				struct poll_table_struct *wait)
{
	return 0;
}

//ｍｍａｐ−未使用
static int voyagergx_mmap(struct file *file, struct vm_area_struct *vma)
{
	return 0;
}

//ＩＯコントロールルーチン
//データ種類の変更やボリュームの操作などを行う
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

	case SNDCTL_DSP_SPEED:		//サンプリングレート設定
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

	case SNDCTL_DSP_STEREO:		//再生チャンネル数設定（実際はステレオかモノラルかの選択）
		setting.ch = *(int *)arg;
		//８０５１側にチャンネル数を知らせる
		data = setting.ch;
		command_8051(0x05,&data,&data2);
        	dma2_set_device(sh7751_dmasound_play_irq);
		return(setting.ch);

	case SNDCTL_DSP_CHANNELS:
		return 0;

	case SNDCTL_DSP_GETFMTS:	//再生データフォーマットゲット（８ビットデータ or １６ビットデータ）
		if(setting.fmt == 16) {
			(int *)arg = AFMT_S16_LE;
		}
		else {
			(int *)arg = AFMT_U8;
		}
		return 0;

	case SNDCTL_DSP_SETFMT:		//再生データフォーマットセット（８ビットデータ or １６ビットデータ）
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

//オープン処理
//各種フラグ初期化
//８０５１は再生中止（起動はライトの初回で掛ける）
static int  voyagergx_open(struct inode *inode, struct file *file)
{
struct voyagergx_state *s = &voyagergx_state;
int	data,data2;

	file->private_data = s;

	s->open_mode |= file->f_mode & (FMODE_READ | FMODE_WRITE);
	//初回転送フラグクリア
	first_flg = 0;
	//最終転送フラグクリア
	last_flg = 0;
	//バッファフラグクリア
	buff_flg1 = buff_flg2 = 0;
	//初回再生はバッファ１側から
	proc_flg = 1;
	//バッファ残りサイズリセット
	next_size = VOYAGER_SOUND_SIZE;
	//ＤＭＡ転送サイズ初期化
	dma_req[0] = VOYAGER_SOUND_SIZE/4;
	dma_req[1] = VOYAGER_SOUND_SIZE/4;
	//ブレイクフラグクリア
	break_flg = 0;
	//再生中断フラグクリア
	abnml_flg = 0;
	//書き込みバッファ番号初期化
	next_write = 1;
	//８０５１側再生開始
	data = 0x00;
	command_8051(0x07,&data,&data2);

	return 0;
}

//リリース（クローズ）処理
//バッファに再生データが存在する場合は再生
//ＤＭＡ停止
//８０５１再生中止
static int voyagergx_release(struct inode *inode, struct file *file)
{
int	data,data2,i;
int	ret;

	//ブレイクフラグが立っているなら即終了
	if(break_flg) {
		return 0;
	}
	//バッファが２本とも一杯の場合空きが出来るまで待つ
	if((buff_flg1 == 1)&&(buff_flg2 == 1)) {
		wari_flg = 1;
		i = 0;
		while(wari_flg) {
			//既に再生が終わっている場合
			if(abnml_flg) {
				break;
			}
			//バッファが空くまで休眠する
			ret = interruptible_sleep_on_timeout(&int_wq,WAIT_TIMEOUT);
			if(ret == 0) {
				break;
			}
			//コントロールＣチェック
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
	//再生サイズが４バイトに満たない場合は再生せずに終了
	if((VOYAGER_SOUND_SIZE - next_size) < 4) {
		voy_break();
		return 0;
	}
	//バッファに空きがある？
	if(next_size != 0) {
		
		//初めての再生？
		if(first_flg == 0) {
			//再生データなし？
			if(next_size == VOYAGER_SOUND_SIZE) {
				//終了する
				voy_break();
				return 0;
			}
			//再生はバッファ１側？
			if(proc_flg == 1) {
      				dma2_queue_buffer(sh7751_dmasound_play_irq,
       						VOYAGER_SOUND_BUF1,
       						(VOYAGER_SOUND_SIZE-next_size)/4);
				//最終判定フラグを立てる
				last_flg = 1;
			}
			//８０５１側再生開始
			data = 0x01;
			command_8051(0x07,&data,&data2);
			dma2_start(sh7751_dmasound_play_irq);
		}
		//バッファ１本に満たないデータの再生
		else {
			if(proc_flg == 2) {
				dma_req[0] = (VOYAGER_SOUND_SIZE-next_size)/4;
				buff_flg1 = 1;
				//最終判定フラグを立てる
				last_flg = 1;
				//データが中断している場合は再度ＤＭＡにキックする
				if(abnml_flg == 1) {
					dma2_queue_buffer(sh7751_dmasound_play_irq,
						VOYAGER_SOUND_BUF1,dma_req[0]);
					dma2_start(sh7751_dmasound_play_irq);
					abnml_flg = 0;
				}
			}
			else {
				//ＤＭＡにリクエスト
				dma_req[1] = (VOYAGER_SOUND_SIZE-next_size)/4;
				buff_flg2 = 1;
				//最終判定フラグを立てる
				last_flg = 1;
				//データが中断している場合は再度ＤＭＡにキックする
				if(abnml_flg == 1) {
					dma2_queue_buffer(sh7751_dmasound_play_irq,
						VOYAGER_SOUND_BUF2,dma_req[1]);
					dma2_start(sh7751_dmasound_play_irq);
					abnml_flg = 0;
				}
			}
		}
	}
	//全再生（転送）が終わるまで待つ
	while(1) {
		if(abnml_flg) {
			break;
		}
		//最終判定フラグが０なら全再生（転送）終了
		if(last_flg == 0) {
			break;
		}
		wari_flg = 1;
		i = 0;
		while(wari_flg) {
			//既に再生が終わっている場合
			if(abnml_flg) {
				break;
			}
			//最終判定フラグが０なら全再生（転送）終了
			if(last_flg == 0) {
				break;
			}
			//バッファが空くまで休眠する
			ret = interruptible_sleep_on_timeout(&int_wq,WAIT_TIMEOUT);
			if(ret == 0) {
				voy_break();
				return -EFAULT;
			}
			//コントロールＣチェック
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
	//終了
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

//プローブ処理
//ドライバ初期化
//コーデック初期化
//ＡＣ９７のチップを表示
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

	//サンプリングレート初期化（４８ｋ）
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

//リムーブ処理
static void __devinit voyagergx_remove(void)
{
	struct voyagergx_state *s = &voyagergx_state;

	if (!s)
		return;
	unregister_sound_dsp(s->dev_audio);
	unregister_sound_mixer(s->codec.dev_mixer);
}

//初期化処理
static	int __init init_voyagergx(void)
{
unsigned long	value;
int	err;

	info("sakuma@ace-jp.com, built " __TIME__ " on " __DATE__);

	//ＧＰＩＯをＡＣ９７と８０５１を使う設定にする
	value = *(volatile unsigned long *)(GPIO_MUX_LOW);
	value |= GPIO_MUX_LOW_AC97 | GPIO_MUX_LOW_8051;
	*(volatile unsigned long *)(GPIO_MUX_LOW) = value;

	//ＤＭＡ停止
	dma2_stop(sh7751_dmasound_play_irq);

	//DMA interrupt request
	//ＤＭＡの割り込みを登録
        err = request_dma2(sh7751_dmasound_play_irq, "voyager DMA",do_irq);
        if (err) {
                return 0;
        }

	//ＡＣ９７の割り込みを有効にする
	value = *(volatile unsigned long *)(VOYAGER_INT_MASK);
	value |= VOYAGER_INT_MASK_AC;
	*(volatile unsigned long *)(VOYAGER_INT_MASK) = value;

	//ＡＣ９７に通電
	value = *(volatile unsigned long *)(POWER_MODE0_GATE);
	value |= POWER_MODE0_GATE_AC;
	*(volatile unsigned long *)(POWER_MODE0_GATE) = value;

	//ＡＣ９７に通電
	value = *(volatile unsigned long *)(POWER_MODE1_GATE);
	value |= POWER_MODE1_GATE_AC;
	*(volatile unsigned long *)(POWER_MODE1_GATE) = value;

	//ＡＣ９７を有効にする
	value = *(volatile unsigned long *)(AC97_CONTROL_STATUS);
	value |= 0x0000000F;
	*(volatile unsigned long *)(AC97_CONTROL_STATUS) = value;
	//一定時間待つ
	mdelay(2);
	//リセット解除
	value &= 0xFFFFFFF9;
	*(volatile unsigned long *)(AC97_CONTROL_STATUS) = value;

	//ＴＡＧの初期化（ＳＬＯＴ１〜４を有効にする）
	value = *(volatile unsigned long *)(AC97_TX_SLOT0);
	value |= 0x0000F800;
	*(volatile unsigned long *)(AC97_TX_SLOT0) = value;

	//初期状態はモノラル、１６ビット、４８ｋ
	setting.ch = 0;
	setting.fmt = 16;
	setting.rate = 48000;

	//ＤＭＡ初期化
        dma2_set_device(sh7751_dmasound_play_irq);

	//８０５１初期化
	init_8051();

	return voyagergx_probe();
}

//アンロード処理
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


//ライトコーデック
static	void wrcodec(struct ac97_codec *codec, u8 addr, u16 data)
{
	u32             cmd,ret;
	int	i;

	//リセットだった場合
	if(addr == 0) {
		//リセットがうまくいかなかった場合はリトライする
		for(i=0;i<10;i++) {
			//書き込むデータをセット
			cmd = (u32) data << AC97C_WD_BIT;	// OR in the data word
			command_8051(3,&cmd,&ret);
			//書き込むアドレスをセット
			cmd = (u32) (addr & AC97C_INDEX_MASK) << 12;
			cmd &= ~AC97C_READ;	// write command
			command_8051(1,&cmd,&ret);
			//リセットがうまくいったか確認
			ret = rdcodec(codec,0);
			if((ret & 0x8000) == 0) {
				break;
			}
		}
	}
	//それ以外だった場合
	else {
		//書き込むデータをセット
		cmd = (u32) data << AC97C_WD_BIT;	// OR in the data word
		command_8051(3,&cmd,&ret);
		//書き込むアドレスをセット
		cmd = (u32) (addr & AC97C_INDEX_MASK) << 12;
		cmd &= ~AC97C_READ;	// write command
		command_8051(1,&cmd,&ret);
		mdelay(1);
		//ボリューム関係の値が消えないように読み出す
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


