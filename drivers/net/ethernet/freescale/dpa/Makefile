#
# Makefile for the Freescale Ethernet controllers
#
EXTRA_CFLAGS           += -DVERSION=\"\"
#
#Include netcomm SW specific definitions
include $(srctree)/drivers/net/ethernet/freescale/fman/ncsw_config.mk

EXTRA_CFLAGS += -I$(NET_DPA)

obj-$(CONFIG_FSL_DPAA_1588) += dpaa_1588.o
obj-$(CONFIG_FSL_DPAA_ETH_SG_SUPPORT) += fsl-dpa-sg.o
# dpaa_debugfs needs to be initialized before dpaa_eth
obj-$(CONFIG_FSL_DPAA_ETH_DEBUGFS) += dpaa_debugfs.o
obj-$(CONFIG_FSL_DPAA_ETH) += fsl-mac.o fsl-dpa.o
obj-$(CONFIG_FSL_DPAA_OFFLINE_PORTS) += fsl-oh.o

fsl-dpa-objs	:= dpa-ethtool.o dpaa_eth.o dpaa_eth_sysfs.o
fsl-dpa-sg-objs	:= dpaa_eth_sg.o
fsl-mac-objs	:= mac.o mac-api.o
fsl-oh-objs	:= offline_port.o

#Needed by the tracing framework
CFLAGS_dpaa_eth.o := -I$(src)
