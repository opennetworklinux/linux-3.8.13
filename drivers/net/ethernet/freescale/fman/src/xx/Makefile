#
# Makefile for the Freescale Ethernet controllers
#
EXTRA_CFLAGS           += -DVERSION=\"\"
#
#Include netcomm SW specific definitions
include $(srctree)/drivers/net/ethernet/freescale/fman/ncsw_config.mk

obj-y		+= fsl-ncsw-xx.o

fsl-ncsw-xx-objs	:=   xx_linux.o udivdi3.o stdlib.o \
				module_strings.o
