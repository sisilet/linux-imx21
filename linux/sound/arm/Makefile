snd-pxa2xx-pcm-objs := pxa2xx-pcm.o
snd-pxa2xx-ac97-objs := pxa2xx-ac97.o

obj-$(CONFIG_SND_PXA2XX_PCM) += snd-pxa2xx-pcm.o
obj-$(CONFIG_SND_PXA2XX_AC97) += snd-pxa2xx-ac97.o
snd-csb536-wm8731-objs := csb536-wm8731.o
snd-csb536fs-objs := csb536fs.o

# Toplevel Module Dependency
snd-sa11x0-pcm-objs		:= sa11x0-pcm.o devdma.o

obj-$(CONFIG_SND_S3C2410)	+= s3c24xx-iis.o
obj-$(CONFIG_SND_CSB536FS) += snd-csb536fs.o
obj-$(CONFIG_SND_CSB536PC) += snd-csb536-wm8731.o
