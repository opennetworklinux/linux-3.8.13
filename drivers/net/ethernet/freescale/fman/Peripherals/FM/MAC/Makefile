#
# Makefile for the Freescale Ethernet controllers
#
EXTRA_CFLAGS           += -DVERSION=\"\"
#
#Include netcomm SW specific definitions
include $(srctree)/drivers/net/ethernet/freescale/fman/ncsw_config.mk

NCSW_FM_INC = $(srctree)/drivers/net/ethernet/freescale/fman/Peripherals/FM/inc

EXTRA_CFLAGS += -I$(NCSW_FM_INC)

obj-y		+= fsl-ncsw-MAC.o

fsl-ncsw-MAC-objs	:=  dtsec.o dtsec_mii_acc.o fm_mac.o tgec.o tgec_mii_acc.o \
			    fman_dtsec.o fman_dtsec_mii_acc.o fman_memac.o \
			    fman_tgec.o fman_crc32.o

ifeq ($(CONFIG_FMAN_T4240),y)
fsl-ncsw-MAC-objs	+=  memac.o memac_mii_acc.o
endif
