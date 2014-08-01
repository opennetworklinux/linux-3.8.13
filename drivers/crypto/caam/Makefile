#
# Makefile for the CAAM backend and dependent components
#
ccflags-$(CONFIG_CRYPTO_DEV_FSL_CAAM_DEBUG) := -DDEBUG
obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM) += caam.o
obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM_CRYPTO_API) += caamalg.o
obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM_AHASH_API) += caamhash.o
obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM_RNG_API) += caamrng.o

caam-objs := ctrl.o jr.o error.o key_gen.o

ifeq ($(CONFIG_FSL_QMAN), y)
	caam-objs += qi.o
endif

obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM_CRYPTO_API_QI) += caamalg_qi.o
obj-$(CONFIG_FSL_CAAM_PKC_SUPPORT) += caam_pkc.o

caam_pkc-objs := caampkc.o pkc_desc.o
