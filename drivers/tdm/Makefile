#
# Makefile for the TDM core.
#

obj-$(CONFIG_TDM)		+= tdm-core.o device/ line_ctrl/ test/
ifeq ($(CONFIG_TDM_DEBUG_CORE),y)
EXTRA_CFLAGS += -DDEBUG
endif
