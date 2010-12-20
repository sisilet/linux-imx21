#include <linux/config.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/fb.h>

#include <asm/io.h>

#include <video/voyagergx_reg.h>

#define SM501_8051_DEVICENAME "sm501_8051"
#define PFX SM501_8051_DEVICENAME

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




static	unsigned char	code_8051[] = {
	0x02,0x01,0x00,0x02,0x07,0x73,0x90,0x08,0xae,0xe0,0xfa,0xa3,0xe0,0x8a,0x82,0xf5,
	0x83,0xe0,0x60,0x13,0x90,0x08,0xa2,0xe4,0xf0,0xa3,0x74,0x30,0xf0,0x78,0x19,0x76,
	0x01,0x78,0x1e,0x76,0x01,0x80,0x04,0x78,0x19,0x76,0x00,0x90,0x3f,0xf2,0x74,0x01,
	0xf0,0xa3,0xe4,0xf0,0x22,0xff,0xff,0xff,0xff,0xff,0xff,0x02,0x06,0x83,0xff,0xff,
	0xff,0xff,0xff,0x02,0x04,0xe3,0x90,0x3f,0xf2,0xe4,0xf0,0xa3,0xf0,0x90,0x3f,0xf0,
	0xe0,0x70,0x02,0xa3,0xe0,0x70,0x06,0x12,0x08,0x8d,0x02,0x00,0xd4,0x90,0x3f,0xf0,
	0xc3,0xe0,0x94,0x01,0x70,0x04,0xa3,0xe0,0x94,0x00,0x70,0x06,0x12,0x00,0xd9,0x02,
	0x00,0xd4,0x90,0x3f,0xf0,0xc3,0xe0,0x94,0x02,0x70,0x04,0xa3,0xe0,0x94,0x00,0x70,
	0x05,0x12,0x07,0xf3,0x80,0x4e,0x90,0x3f,0xf0,0xc3,0xe0,0x94,0x03,0x70,0x04,0xa3,
	0xe0,0x94,0x00,0x70,0x05,0x12,0x08,0x1a,0x80,0x3a,0x90,0x3f,0xf0,0xc3,0xe0,0x94,
	0x04,0x70,0x04,0xa3,0xe0,0x94,0x00,0x70,0x05,0x12,0x08,0x41,0x80,0x26,0x90,0x3f,
	0xf0,0xc3,0xe0,0x94,0x05,0x70,0x04,0xa3,0xe0,0x94,0x00,0x70,0x05,0x12,0x08,0x7c,
	0x80,0x12,0x90,0x3f,0xf0,0xc3,0xe0,0x94,0x07,0x70,0x04,0xa3,0xe0,0x94,0x00,0x70,
	0x03,0x12,0x00,0x06,0x78,0x1b,0x76,0x00,0x22,0x90,0x3f,0xf4,0x7a,0x04,0x78,0x08,
	0xe0,0xf6,0xa3,0x08,0xda,0xfa,0x90,0x91,0x04,0x7a,0x04,0x78,0x08,0xe6,0xf0,0xa3,
	0x08,0xda,0xfa,0x12,0x06,0xd6,0x90,0x3f,0xf2,0x74,0x01,0xf0,0xa3,0xe4,0xf0,0x22,
	0x75,0x81,0x2b,0x75,0x0c,0xc0,0x75,0x0d,0x08,0x75,0x0e,0xff,0x75,0x0f,0x2f,0xc2,
	0xd3,0xc2,0xd4,0x12,0x03,0x4a,0x12,0x03,0xb6,0x12,0x03,0x2c,0x12,0x03,0x0e,0x12,
	0x02,0xac,0x12,0x07,0xcb,0x12,0x01,0x2a,0x01,0x00,0x90,0x08,0xc1,0x85,0x82,0x0a,
	0x85,0x83,0x0b,0x74,0x98,0xf5,0x82,0xf5,0x08,0x74,0x02,0xf5,0x83,0xf5,0x09,0x7a,
	0x14,0xe4,0x93,0xa3,0x85,0x82,0x08,0x85,0x83,0x09,0x85,0x0a,0x82,0x85,0x0b,0x83,
	0xf0,0xa3,0x85,0x83,0x0b,0x85,0x82,0x0a,0x85,0x09,0x83,0x85,0x08,0x82,0xda,0xe1,
	0x90,0x08,0xd5,0xe4,0xf0,0xa3,0xf0,0x90,0x08,0xd7,0xe4,0xf0,0xa3,0xf0,0x90,0x08,
	0xd9,0xe4,0xf0,0xa3,0xf0,0x90,0x08,0xdb,0xe4,0xf0,0xa3,0xf0,0x78,0x18,0x76,0x00,
	0x78,0x1f,0x76,0x00,0x90,0x91,0x80,0x74,0x03,0xf0,0x90,0x08,0xc1,0x7a,0x04,0x78,
	0x08,0xe0,0xf6,0xa3,0x08,0xda,0xfa,0x90,0x91,0x00,0x7a,0x04,0x78,0x08,0xe6,0xf0,
	0xa3,0x08,0xda,0xfa,0x90,0x08,0xc5,0x7a,0x04,0x78,0x08,0xe0,0xf6,0xa3,0x08,0xda,
	0xfa,0x90,0x91,0x04,0x7a,0x04,0x78,0x08,0xe6,0xf0,0xa3,0x08,0xda,0xfa,0x90,0x08,
	0xc9,0x7a,0x04,0x78,0x08,0xe0,0xf6,0xa3,0x08,0xda,0xfa,0x90,0x91,0x08,0x7a,0x04,
	0x78,0x08,0xe6,0xf0,0xa3,0x08,0xda,0xfa,0x90,0x08,0xcd,0x7a,0x04,0x78,0x08,0xe0,
	0xf6,0xa3,0x08,0xda,0xfa,0x90,0x91,0x0c,0x7a,0x04,0x78,0x08,0xe6,0xf0,0xa3,0x08,
	0xda,0xfa,0x90,0x08,0xd1,0x7a,0x04,0x78,0x08,0xe0,0xf6,0xa3,0x08,0xda,0xfa,0x90,
	0x91,0x10,0x7a,0x04,0x78,0x08,0xe6,0xf0,0xa3,0x08,0xda,0xfa,0x90,0x91,0x40,0xe4,
	0xf0,0xa3,0xf0,0xa3,0xf0,0xa3,0xf0,0x90,0x91,0x44,0xe4,0xf0,0xa3,0xf0,0xa3,0xf0,
	0xa3,0xf0,0x90,0x91,0x48,0xe4,0xf0,0xa3,0xf0,0xa3,0xf0,0xa3,0xf0,0x43,0xa0,0x01,
	0x90,0x08,0xdd,0xe4,0xf0,0xa3,0xf0,0x12,0x06,0x2f,0x90,0x08,0xdd,0x74,0xc8,0xf0,
	0xa3,0xe4,0xf0,0x12,0x05,0xd0,0x90,0x91,0x80,0x74,0x01,0xf0,0x90,0x08,0xdd,0x74,
	0xff,0xf0,0xa3,0xe4,0xf0,0x12,0x06,0x2f,0x90,0x08,0xdd,0x74,0x64,0xf0,0xa3,0xe4,
	0xf0,0x12,0x05,0xd0,0x12,0x07,0xa1,0x90,0x08,0xdd,0x74,0xff,0xf0,0xa3,0xe4,0xf0,
	0x12,0x06,0x2f,0x12,0x07,0xa1,0x12,0x07,0x28,0x12,0x07,0x28,0x78,0x1b,0xe6,0x60,
	0x03,0x12,0x00,0x46,0x78,0x1e,0xe6,0x60,0x07,0x78,0x1e,0x76,0x00,0x12,0x08,0x9b,
	0x80,0xea,0x90,0x91,0x80,0xe4,0xf0,0x22,0x00,0xf8,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x74,0x14,0x70,0x04,
	0x74,0x00,0x60,0x1e,0x90,0x08,0x68,0xa8,0x83,0xa9,0x82,0x90,0x08,0xa2,0xaa,0x83,
	0xab,0x82,0x90,0x08,0xb6,0xac,0x83,0xad,0x82,0xc3,0xed,0x9b,0x70,0x28,0xec,0x9a,
	0x70,0x24,0x74,0x0a,0x70,0x04,0x74,0x00,0x60,0x1b,0x90,0x08,0xc0,0xac,0x83,0xad,
	0x82,0x90,0x08,0xb6,0xc3,0xed,0x95,0x82,0x70,0x05,0xec,0x95,0x83,0x60,0x06,0xe4,
	0xf0,0xa3,0x02,0x02,0xe4,0x22,0x88,0x83,0x89,0x82,0xe4,0x93,0xa3,0xa8,0x83,0xa9,
	0x82,0x8a,0x83,0x8b,0x82,0xf0,0xa3,0xaa,0x83,0xab,0x82,0x02,0x02,0xc9,0x74,0x00,
	0x60,0x0d,0x90,0x00,0x00,0x79,0x00,0xfa,0xe4,0x93,0xf7,0xa3,0x09,0xda,0xf9,0x74,
	0x13,0x60,0x08,0x78,0x18,0xfa,0xe4,0xf6,0x08,0xda,0xfc,0x22,0x74,0x00,0x60,0x0d,
	0x90,0x00,0x00,0x79,0x00,0xfa,0xe4,0x93,0xf7,0xa3,0x09,0xda,0xf9,0x74,0x00,0x60,
	0x08,0x78,0x00,0xfa,0xe4,0xf6,0x08,0xda,0xfc,0x22,0x74,0x00,0x60,0x35,0xfb,0x90,
	0x00,0x00,0x74,0x00,0x75,0xf0,0x08,0x84,0x24,0x20,0xf8,0x7a,0x08,0xe5,0xf0,0x60,
	0x07,0xe6,0x03,0x1a,0xd5,0xf0,0xfb,0xf6,0xeb,0x60,0x0d,0x1b,0xe4,0x93,0xa3,0xa2,
	0xe0,0xe6,0x13,0xf6,0xda,0xf2,0x80,0x05,0xe6,0x03,0xda,0xfd,0xf6,0x08,0x7a,0x08,
	0xeb,0x70,0xe5,0x74,0x00,0x60,0x2e,0xfb,0x74,0x00,0x75,0xf0,0x08,0x84,0x24,0x20,
	0xf8,0x7a,0x08,0xe5,0xf0,0x60,0x07,0xe6,0x03,0x1a,0xd5,0xf0,0xfb,0xf6,0xeb,0x60,
	0x09,0x1b,0xc3,0xe6,0x13,0xf6,0xda,0xf6,0x80,0x05,0xe6,0x03,0xda,0xfd,0xf6,0x08,
	0x7a,0x08,0xeb,0x70,0xe9,0x22,0x74,0x00,0x60,0x0d,0x90,0x00,0x00,0x79,0x00,0xfa,
	0xe4,0x93,0xf7,0xa3,0x09,0xda,0xf9,0x74,0x00,0x60,0x08,0x78,0x00,0xfa,0xe4,0xf6,
	0x08,0xda,0xfc,0x22,0x90,0x08,0xdd,0xe4,0xf0,0xa3,0xf0,0x90,0x08,0xdd,0xc3,0xe0,
	0x94,0x40,0xa3,0xe0,0x94,0x00,0x30,0xd2,0x04,0xb2,0xe7,0xd2,0xe0,0x20,0xe7,0x03,
	0x02,0x04,0xdf,0x90,0x08,0xdd,0xe0,0xf5,0x08,0xa3,0xe0,0xf5,0x09,0x7a,0x0c,0xe5,
	0x08,0xc3,0x33,0xf5,0x08,0xe5,0x09,0x33,0xf5,0x09,0xda,0xf3,0xe5,0x09,0x33,0xe4,
	0x50,0x01,0x14,0xf5,0x0a,0xf5,0x0b,0xe5,0x08,0x44,0x00,0xf5,0x08,0xe5,0x09,0x44,
	0x00,0xf5,0x09,0xe5,0x0a,0x44,0x08,0xf5,0x0a,0xe5,0x0b,0x44,0x00,0xf5,0x0b,0x90,
	0x08,0xe1,0xe5,0x08,0xf0,0xa3,0xe5,0x09,0xf0,0x90,0x08,0xe1,0xe0,0xf5,0x08,0xa3,
	0xe0,0xf5,0x09,0x33,0xe4,0x50,0x01,0x14,0xf5,0x0a,0xf5,0x0b,0x90,0x91,0x04,0x7a,
	0x04,0x78,0x08,0xe6,0xf0,0xa3,0x08,0xda,0xfa,0x90,0x08,0xdf,0xe4,0xf0,0xa3,0xf0,
	0x90,0x08,0xdf,0xc3,0xe0,0x94,0x80,0xa3,0xe0,0x94,0x38,0x30,0xd2,0x04,0xb2,0xe7,
	0xd2,0xe0,0x30,0xe7,0x5b,0x90,0x91,0x40,0xe0,0x54,0x00,0xf5,0x08,0xa3,0xe0,0x54,
	0xa0,0xf5,0x09,0xa3,0xe0,0x54,0x00,0xf5,0x0a,0xa3,0xe0,0x54,0x00,0xf5,0x0b,0x78,
	0x08,0xc3,0xe6,0x94,0x00,0x70,0x10,0x08,0xe6,0x94,0xa0,0x70,0x0a,0x08,0xe6,0x94,
	0x00,0x70,0x04,0x08,0xe6,0x94,0x00,0x70,0x19,0x90,0x91,0x48,0x7a,0x04,0x78,0x08,
	0xe0,0xf6,0xa3,0x08,0xda,0xfa,0x90,0x08,0xe1,0xe5,0x08,0xf0,0xa3,0xe5,0x09,0xf0,
	0x80,0x0e,0x90,0x08,0xdf,0xe0,0x24,0x01,0xf0,0xa3,0xe0,0x34,0x00,0xf0,0x80,0x90,
	0x90,0x08,0xdd,0xe0,0x24,0x01,0xf0,0xa3,0xe0,0x34,0x00,0xf0,0x02,0x03,0xdb,0x12,
	0x06,0xd6,0x22,0xc0,0xe0,0xc0,0x82,0xc0,0x83,0xc0,0xd0,0xc0,0x08,0xc0,0x09,0xc0,
	0x0a,0xc0,0x0b,0xc2,0xd3,0xd2,0xd4,0xc2,0xaf,0x53,0xa8,0xfe,0x53,0xe8,0xf9,0x78,
	0x19,0xe6,0x70,0x03,0x02,0x05,0xac,0x90,0x08,0xbe,0xc3,0xe0,0x94,0x01,0x70,0x04,
	0xa3,0xe0,0x94,0x00,0x60,0x03,0x02,0x05,0xac,0x78,0x1f,0xe6,0x70,0x37,0x90,0x08,
	0xa4,0xe0,0xfa,0xa3,0xe0,0x8a,0x82,0xf5,0x83,0xe0,0xf5,0x08,0xa3,0xe0,0xf5,0x09,
	0x33,0xe4,0x50,0x01,0x14,0xf5,0x0a,0xf5,0x0b,0x90,0x91,0x10,0x7a,0x04,0x78,0x08,
	0xe6,0xf0,0xa3,0x08,0xda,0xfa,0x90,0x91,0x0c,0x7a,0x04,0x78,0x08,0xe6,0xf0,0xa3,
	0x08,0xda,0xfa,0x80,0x54,0x90,0x08,0xa4,0xe0,0xfa,0xa3,0xe0,0x8a,0x82,0xf5,0x83,
	0xe0,0xf5,0x08,0xa3,0xe0,0xf5,0x09,0x33,0xe4,0x50,0x01,0x14,0xf5,0x0a,0xf5,0x0b,
	0x90,0x91,0x0c,0x7a,0x04,0x78,0x08,0xe6,0xf0,0xa3,0x08,0xda,0xfa,0x90,0x08,0xa4,
	0xe0,0x24,0x02,0xfa,0xa3,0xe0,0x34,0x00,0xf5,0x83,0x8a,0x82,0xe0,0xf5,0x08,0xa3,
	0xe0,0xf5,0x09,0x33,0xe4,0x50,0x01,0x14,0xf5,0x0a,0xf5,0x0b,0x90,0x91,0x10,0x7a,
	0x04,0x78,0x08,0xe6,0xf0,0xa3,0x08,0xda,0xfa,0x12,0x08,0x9b,0x90,0x91,0x81,0xe0,
	0x90,0x00,0x39,0xf0,0x43,0xe8,0x06,0x43,0xf8,0x04,0x43,0xa8,0x01,0xd2,0xaf,0xd0,
	0x0b,0xd0,0x0a,0xd0,0x09,0xd0,0x08,0xd0,0xd0,0xd0,0x83,0xd0,0x82,0xd0,0xe0,0x32,
	0x90,0x08,0xdf,0xe4,0xf0,0xa3,0xf0,0x90,0x08,0xdf,0xc3,0xe0,0x94,0xe8,0xa3,0xe0,
	0x94,0x03,0x30,0xd2,0x04,0xb2,0xe7,0xd2,0xe0,0x30,0xe7,0x42,0x90,0x08,0xe1,0xe4,
	0xf0,0xa3,0xf0,0x90,0x08,0xdd,0xe0,0xf5,0x08,0xa3,0xe0,0xf5,0x09,0x90,0x08,0xe1,
	0xc3,0xe0,0x95,0x08,0xa3,0xe0,0x95,0x09,0x30,0xd2,0x04,0xb2,0xe7,0xd2,0xe0,0x30,
	0xe7,0x0e,0x90,0x08,0xe1,0xe0,0x24,0x01,0xf0,0xa3,0xe0,0x34,0x00,0xf0,0x80,0xd3,
	0x90,0x08,0xdf,0xe0,0x24,0x01,0xf0,0xa3,0xe0,0x34,0x00,0xf0,0x80,0xa9,0x22,0x90,
	0x08,0xe1,0xe4,0xf0,0xa3,0x74,0x30,0xf0,0x90,0x08,0xdf,0xe4,0xf0,0xa3,0xf0,0x90,
	0x08,0xdf,0xc3,0xe0,0x94,0x00,0xa3,0xe0,0x94,0x06,0x30,0xd2,0x04,0xb2,0xe7,0xd2,
	0xe0,0x30,0xe7,0x2e,0x90,0x08,0xdd,0xe0,0x90,0x08,0xe1,0xc0,0xe0,0xe0,0xfa,0xa3,
	0xe0,0x8a,0x82,0xf5,0x83,0xd0,0xe0,0xf0,0x90,0x08,0xe1,0xe0,0x24,0x01,0xf0,0xa3,
	0xe0,0x34,0x00,0xf0,0x90,0x08,0xdf,0xe0,0x24,0x01,0xf0,0xa3,0xe0,0x34,0x00,0xf0,
	0x80,0xbd,0x22,0xc0,0xe0,0xc0,0x82,0xc0,0x83,0xc0,0xd0,0xc0,0x08,0xc0,0x09,0xc0,
	0x0a,0xc0,0x0b,0xc2,0xd3,0xd2,0xd4,0xc2,0xaf,0x53,0xa8,0xfe,0x53,0xe8,0xf9,0x78,
	0x1b,0x76,0x01,0x90,0x90,0x0c,0x7a,0x04,0x78,0x08,0xe0,0xf6,0xa3,0x08,0xda,0xfa,
	0x90,0x00,0x35,0x7a,0x04,0x78,0x08,0xe6,0xf0,0xa3,0x08,0xda,0xfa,0x43,0xe8,0x06,
	0x43,0xa8,0x01,0xd2,0xaf,0xd0,0x0b,0xd0,0x0a,0xd0,0x09,0xd0,0x08,0xd0,0xd0,0xd0,
	0x83,0xd0,0x82,0xd0,0xe0,0x32,0x90,0x00,0x3e,0xe4,0xf0,0xa3,0xf0,0x90,0x00,0x3e,
	0xc3,0xe0,0x94,0x80,0xa3,0xe0,0x94,0x38,0x30,0xd2,0x04,0xb2,0xe7,0xd2,0xe0,0x30,
	0xe7,0x35,0x90,0x91,0x40,0xe0,0x54,0x00,0xf5,0x08,0xa3,0xe0,0x54,0x60,0xf5,0x09,
	0xa3,0xe0,0x54,0x00,0xf5,0x0a,0xa3,0xe0,0x54,0x00,0xf5,0x0b,0x78,0x08,0x7a,0x04,
	0xe6,0x70,0x03,0x08,0xda,0xfa,0x70,0x01,0x22,0x90,0x00,0x3e,0xe0,0x24,0x01,0xf0,
	0xa3,0xe0,0x34,0x00,0xf0,0x80,0xb6,0x22,0x90,0x3f,0xf0,0x74,0x0f,0xf0,0xa3,0xe4,
	0xf0,0x90,0x3f,0xf4,0xe4,0xf0,0xa3,0x74,0x60,0xf0,0xa3,0x74,0x0a,0xf0,0xa3,0xe4,
	0xf0,0x90,0x3f,0xf8,0xe4,0xf0,0xa3,0xf0,0xa3,0xf0,0xa3,0xf0,0x12,0x00,0x46,0x90,
	0x3f,0xf8,0xe0,0x54,0xf0,0xf5,0x08,0x7a,0x03,0x78,0x08,0xa3,0x08,0xe0,0x54,0x00,
	0xf6,0xda,0xf8,0x78,0x08,0x7a,0x04,0xe6,0x70,0x03,0x08,0xda,0xfa,0x60,0x01,0x22,
	0x80,0xb6,0x22,0xc0,0xe0,0xc0,0x82,0xc0,0x83,0xc0,0xd0,0xc2,0xd3,0xd2,0xd4,0xc2,
	0xaf,0x53,0xa8,0xfe,0x53,0xe8,0xf9,0x90,0x08,0xbe,0x74,0x01,0xf0,0xa3,0xe4,0xf0,
	0x43,0xe8,0x06,0x43,0xa8,0x01,0xd2,0xaf,0xd0,0xd0,0xd0,0x83,0xd0,0x82,0xd0,0xe0,
	0x32,0x90,0x91,0x40,0xe0,0x54,0x00,0xf5,0x08,0xa3,0xe0,0x54,0x80,0xf5,0x09,0xa3,
	0xe0,0x54,0x00,0xf5,0x0a,0xa3,0xe0,0x54,0x00,0xf5,0x0b,0x78,0x08,0x7a,0x04,0xe6,
	0x70,0x03,0x08,0xda,0xfa,0x60,0x01,0x22,0x80,0xd7,0x22,0x90,0x08,0xbe,0xe4,0xf0,
	0xa3,0xf0,0x90,0x08,0xb8,0xe4,0xf0,0xa3,0xf0,0x90,0x08,0xb6,0xe4,0xf0,0xa3,0xf0,
	0x78,0x1e,0x76,0x00,0x75,0xa8,0x81,0x75,0xb8,0x01,0x75,0xe8,0x06,0x75,0xf8,0x04,
	0xd2,0xaf,0x22,0x90,0x91,0x44,0x7a,0x04,0x78,0x08,0xe0,0xf6,0xa3,0x08,0xda,0xfa,
	0x90,0x3f,0xf4,0x7a,0x04,0x78,0x08,0xe6,0xf0,0xa3,0x08,0xda,0xfa,0x12,0x06,0xd6,
	0x90,0x3f,0xf2,0x74,0x01,0xf0,0xa3,0xe4,0xf0,0x22,0x90,0x3f,0xf4,0x7a,0x04,0x78,
	0x08,0xe0,0xf6,0xa3,0x08,0xda,0xfa,0x90,0x91,0x08,0x7a,0x04,0x78,0x08,0xe6,0xf0,
	0xa3,0x08,0xda,0xfa,0x12,0x06,0xd6,0x90,0x3f,0xf2,0x74,0x01,0xf0,0xa3,0xe4,0xf0,
	0x22,0x90,0x91,0x48,0x7a,0x04,0x78,0x08,0xe0,0xf6,0xa3,0x08,0xda,0xfa,0x90,0x3f,
	0xf4,0x7a,0x04,0x78,0x08,0xe6,0xf0,0xa3,0x08,0xda,0xfa,0x12,0x06,0xd6,0x90,0x3f,
	0xf2,0x74,0x01,0xf0,0xa3,0xe4,0xf0,0x22,0x00,0x30,0x00,0x36,0x41,0x91,0x45,0x91,
	0xf4,0x3f,0xf6,0x3f,0xf4,0x3f,0xf5,0x3f,0xf6,0x3f,0xf7,0x3f,0x90,0x3f,0xf4,0x78,
	0x1f,0xe0,0xf6,0x90,0x3f,0xf2,0x74,0x01,0xf0,0xa3,0xe4,0xf0,0x22,0x78,0x1e,0x76,
	0x01,0x90,0x3f,0xf2,0x74,0x01,0xf0,0xa3,0xe4,0xf0,0x22,0x53,0xa0,0xfe,0x43,0xa0,
	0x01,0x22,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};


#define SM501_8051_DEVICENAME "sm501_8051"


struct sm501_8051_regmap {
  volatile unsigned long reset;
  volatile unsigned long select;
  volatile unsigned long proto_8051_irq;
  volatile unsigned long proto_cpu_irq;
  volatile const unsigned char _reserved[0x10000 - 4 * sizeof(long)];
  volatile unsigned char sram[0x3000];
  volatile unsigned long dpram[0x1000];
} __attribute__((packed));

struct sm501_8051_ac97progiomap {
  volatile unsigned short cmd;
  volatile unsigned short status;
  volatile unsigned long data1;
  volatile unsigned long data2;
} __attribute__((packed));

static int command_8051_start (struct sm501_8051_regmap *base,
                               int cmd, int *data1, int *data2)
{
  int ret;

  struct sm501_8051_ac97progiomap *p8051_ac97
    = (struct sm501_8051_ac97progiomap *)(&base->sram);

  p8051_ac97->cmd = cmd;
  p8051_ac97->status = 0;
  p8051_ac97->data1 = *data1;
  p8051_ac97->data2 = *data2;

  err("data1: %d %d\n", *data1, p8051_ac97->data1);
  err("data2: %d %d\n", *data2, p8051_ac97->data2);

  base->proto_cpu_irq = 0xff;
  return 0;
}


static int command_8051_finished (struct sm501_8051_regmap *base,
                                  int cmd, int *data1, int *data2)
{
  struct sm501_8051_ac97progiomap *p8051_ac97
    = (struct sm501_8051_ac97progiomap *)(&base->sram);
  int ret = p8051_ac97->status;

  if (ret)
    {
      *data1 = p8051_ac97->data1;
      *data2 = p8051_ac97->data2;
    }

  return ret;
}

static void sm501_8051_command (struct sm501_8051_regmap *base,
                                int cmd, int *data1, int *data2)
{
  command_8051_start(base, cmd, data1, data2);
  do {
    mdelay(100);
  } while (!command_8051_finished(base, cmd, data1, data2));
}


static void sm501_8051_init (struct sm501_8051_regmap *base)
{
  err("init");

  base->reset = 0;
  base->select = 0x11;

  {
    int i;
    volatile unsigned char *pdsram
      = (unsigned char*)(&base->sram);

    for (i = 0; i < sizeof(code_8051); i++)
      pdsram[i] = code_8051[i];
  }

  base->reset = 1;
  mdelay(100);

  int d = 1;

#warning TODO: I do not think this is working...
  sm501_8051_command(base, 5, &d, &d);
}


static int __devinit
sm501_8051_probe (struct device *dev)
{
  int ret = 0;
  struct platform_device *pdev = to_platform_device(dev);

  info("SM501_8051_VERSION_INFO_HERE");

  if (!request_mem_region(pdev->resource[0].start,
                          pdev->resource[0].end - pdev->resource[0].start + 1,
                          SM501_8051_DEVICENAME)) {
    ret = -EBUSY;
    goto bail_on_err;
  }
  dev->driver_data = ioremap_nocache(pdev->resource[0].start,
                                     pdev->resource[0].end - pdev->resource[0].start + 1);
  sm501_8051_init((struct sm501_8051_regmap*)(dev->driver_data));

  return 0;

 bail_on_err:
  err("bailing on error");
  return ret;
}

static int __devexit sm501_8051_remove (struct device *dev)
{
  return 0;
}

static struct device_driver sm501_8051_driver = {
  .name = SM501_8051_DEVICENAME,
  .bus = &platform_bus_type,
  .probe = sm501_8051_probe,
  .remove = sm501_8051_remove,
};


static	int __init init_sm501_8051 (void)
{
  int ret = driver_register(&sm501_8051_driver);
  return ret;
}

static void __exit cleanup_sm501_8051(void)
{
  driver_unregister(&sm501_8051_driver);
}


module_init(init_sm501_8051);
module_exit(cleanup_sm501_8051);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver for SM501 8051");
MODULE_AUTHOR("Bill Gatliff <bgat@billgatliff.com>");

