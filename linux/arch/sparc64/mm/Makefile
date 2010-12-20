# $Id: Makefile,v 1.1.1.1 2005/03/24 06:16:30 arch-linux Exp $
# Makefile for the linux Sparc64-specific parts of the memory manager.
#

EXTRA_AFLAGS := -ansi
EXTRA_CFLAGS := -Werror

obj-y    := ultra.o tlb.o fault.o init.o generic.o extable.o

obj-$(CONFIG_HUGETLB_PAGE) += hugetlbpage.o
