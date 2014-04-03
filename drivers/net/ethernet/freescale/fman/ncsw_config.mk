#
# Makefile config for the Freescale NetcommSW
#
NET_DPA     = $(srctree)/drivers/net
DRV_DPA     = $(srctree)/drivers/net/ethernet/freescale/dpa
FMAN        = $(srctree)/drivers/net/ethernet/freescale/fman

ifeq ("$(CONFIG_FMAN_P3040_P4080_P5020)", "y")
EXTRA_CFLAGS +=-include $(FMAN)/p3040_4080_5020_dflags.h
endif
ifeq ("$(CONFIG_FMAN_P1023)", "y")
EXTRA_CFLAGS +=-include $(FMAN)/p1023_dflags.h
endif
ifdef CONFIG_FMAN_T4240
EXTRA_CFLAGS +=-include $(FMAN)/t4240_dflags.h
endif

EXTRA_CFLAGS += -I$(DRV_DPA)/
EXTRA_CFLAGS += -I$(FMAN)/inc
EXTRA_CFLAGS += -I$(FMAN)/inc/cores
EXTRA_CFLAGS += -I$(FMAN)/inc/etc
EXTRA_CFLAGS += -I$(FMAN)/inc/Peripherals
EXTRA_CFLAGS += -I$(FMAN)/inc/flib

ifeq ("$(CONFIG_FMAN_P3040_P4080_P5020)", "y")
EXTRA_CFLAGS += -I$(FMAN)/inc/integrations/P3040_P4080_P5020
endif
ifeq ("$(CONFIG_FMAN_P1023)", "y")
EXTRA_CFLAGS += -I$(FMAN)/inc/integrations/P1023
endif
ifdef CONFIG_FMAN_T4240
EXTRA_CFLAGS += -I$(FMAN)/inc/integrations/T4240
endif

EXTRA_CFLAGS += -I$(FMAN)/src/inc
EXTRA_CFLAGS += -I$(FMAN)/src/inc/system
EXTRA_CFLAGS += -I$(FMAN)/src/inc/wrapper
EXTRA_CFLAGS += -I$(FMAN)/src/inc/xx
EXTRA_CFLAGS += -I$(srctree)/include/uapi/linux/fmd
EXTRA_CFLAGS += -I$(srctree)/include/uapi/linux/fmd/Peripherals
EXTRA_CFLAGS += -I$(srctree)/include/uapi/linux/fmd/integrations
