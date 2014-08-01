#
# Makefile for the Freescale Ethernet controllers
#
ccflags-y += -DVERSION=\"\"
#
#Include netcomm SW specific definitions
include $(srctree)/drivers/net/ethernet/freescale/fman/ncsw_config.mk

ccflags-y += -I$(NET_DPA)

obj-$(CONFIG_FSL_DPAA_1588) += dpaa_1588.o
# dpaa_debugfs needs to be initialized before dpaa_eth
obj-$(CONFIG_FSL_DPAA_ETH_DEBUGFS) += dpaa_debugfs.o
obj-$(CONFIG_FSL_DPAA_ETH) += fsl-mac.o fsl-dpa.o fsl-dpa-common.o \
 fsl-dpa-base.o fsl-dpa-shared.o fsl-dpa-macless.o fsl-dpa-proxy.o
obj-$(CONFIG_FSL_DPAA_OFFLINE_PORTS) += fsl-oh.o

fsl-dpa-objs	:= dpa-ethtool.o dpaa_eth_sysfs.o \
  dpaa_eth.o dpaa_eth_sg.o
fsl-dpa-common-objs	:= dpaa_eth_common.o
fsl-dpa-base-objs	:= dpaa_eth_base.o
fsl-dpa-sg-objs	:= dpaa_eth_sg.o
fsl-dpa-shared-objs	:= dpaa_eth_shared.o
fsl-dpa-macless-objs	:= dpaa_eth_macless.o
fsl-dpa-proxy-objs	:= dpaa_eth_proxy.o
fsl-mac-objs	:= mac.o mac-api.o
fsl-oh-objs	:= offline_port.o

#Needed by the tracing framework
CFLAGS_dpaa_eth.o := -I$(src)
