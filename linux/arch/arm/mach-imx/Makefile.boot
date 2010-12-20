ifeq ($(CONFIG_ARCH_MX1ADS),y)
    zreladdr-$(CONFIG_ARCH_MX1ADS)	:= 0x08008000
else
ifeq ($(CONFIG_MACH_CSB536),y)
#    zreladdr-$(CONFIG_MACH_CSB536)	:= 0x08200000
    zreladdr-$(CONFIG_MACH_CSB536)	:= 0x08008000
#    params_phys-$(CONFIG_MACH_CSB536)	:= 0x08000100
#    initrd_phys-$(CONFIG_MACH_CSB536)	:= 0x08410000

endif
endif
