#
# Makefile for the Freescale Ethernet controllers
#
EXTRA_CFLAGS           += -DVERSION=\"\"
#
#Include netcomm SW specific definitions
include $(srctree)/drivers/net/ethernet/freescale/fman/ncsw_config.mk
NCSW_FM_INC = $(srctree)/drivers/net/ethernet/freescale/fman/Peripherals/FM/inc

EXTRA_CFLAGS += -I$(NCSW_FM_INC)


obj-y		+= fsl-ncsw-PFM1.o

fsl-ncsw-PFM1-objs	:=   fm.o fm_muram.o

obj-y		+= MAC/
obj-y		+= Pcd/
obj-y		+= SP/
obj-y		+= Port/
obj-y		+= HC/
obj-y		+= Rtc/
