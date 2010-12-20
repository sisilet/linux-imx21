#
# Makefile for the VoyagerGX framebuffer driver
#

obj-$(CONFIG_FB_VOYAGER_GX) += voyagerfb.o

# Link order matters
voyagerfb-objs	:= voyager_gxfb.o	\
		   voyager_videofb.o	\
		   voyager_valphafb.o	\
		   voyager_alphafb.o	\
		   voyager_panelcsrfb.o	\
		   voyager_crtfb.o	\
		   voyager_crtcsrfb.o
