#
# Makefile for the linux kernel.
#

obj-y			:= clock.o irq.o time.o gpio.o common.o devices.o
obj-m			:=
obj-n			:=
obj-			:=

# Board-specific support
obj-$(CONFIG_ARCH_AT91RM9200DK)	+= board-dk.o
obj-$(CONFIG_MACH_AT91RM9200EK)	+= board-ek.o
obj-$(CONFIG_MACH_CSB337)	+= board-csb337.o
obj-$(CONFIG_MACH_CSB437TL)	+= board-csb437tl.o
obj-$(CONFIG_MACH_CSB637)	+= board-csb637.o
obj-$(CONFIG_MACH_CARMEVA)	+= board-carmeva.o


# LEDs support
leds-$(CONFIG_ARCH_AT91RM9200DK) += dk-leds.o
leds-$(CONFIG_MACH_AT91RM9200EK) += dk-leds.o
leds-$(CONFIG_MACH_CSB337) += dk-leds.o
leds-$(CONFIG_MACH_CSB437TL) += dk-leds.o
leds-$(CONFIG_MACH_CSB637) += dk-leds.o
obj-$(CONFIG_LEDS) += $(leds-y)

# VGA support
obj-$(CONFIG_FB_S1D13XXX)	+= ics1523.o
