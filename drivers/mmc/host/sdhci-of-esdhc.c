/*
 * Freescale eSDHC controller driver.
 *
 * Copyright (c) 2007, 2010, 2012 Freescale Semiconductor, Inc.
 * Copyright (c) 2009 MontaVista Software, Inc.
 *
 * Authors: Xiaobo Xie <X.Xie@freescale.com>
 *	    Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mmc/host.h>
#include "sdhci-pltfm.h"
#include "sdhci-esdhc.h"
#include <asm/mpc85xx.h>

#define VENDOR_V_22	0x12
#define VENDOR_V_23	0x13
static u32 esdhc_readl(struct sdhci_host *host, int reg)
{
	u32 ret;

	ret = in_be32(host->ioaddr + reg);
	/*
	 * The bit of ADMA flag in eSDHC is not compatible with standard
	 * SDHC register, so set fake flag SDHCI_CAN_DO_ADMA2 when ADMA is
	 * supported by eSDHC.
	 * And for many FSL eSDHC controller, the reset value of field
	 * SDHCI_CAN_DO_ADMA1 is one, but some of them can't support ADMA,
	 * only these vendor version is greater than 2.2/0x12 support ADMA.
	 * For FSL eSDHC, must aligned 4-byte, so use 0xFC to read the
	 * the verdor version number, oxFE is SDHCI_HOST_VERSION.
	 */
	if ((reg == SDHCI_CAPABILITIES) && (ret & SDHCI_CAN_DO_ADMA1)) {
		u32 tmp = in_be32(host->ioaddr + SDHCI_SLOT_INT_STATUS);
		tmp = (tmp & SDHCI_VENDOR_VER_MASK) >> SDHCI_VENDOR_VER_SHIFT;
		if (tmp > VENDOR_V_22)
			ret |= SDHCI_CAN_DO_ADMA2;
	}

	return ret;
}

static u16 esdhc_readw(struct sdhci_host *host, int reg)
{
	u16 ret;
	int base = reg & ~0x3;
	int shift = (reg & 0x2) * 8;

	if (unlikely(reg == SDHCI_HOST_VERSION))
		ret = in_be32(host->ioaddr + base) & 0xffff;
	else
		ret = (in_be32(host->ioaddr + base) >> shift) & 0xffff;
	return ret;
}

static u8 esdhc_readb(struct sdhci_host *host, int reg)
{
	int base = reg & ~0x3;
	int shift = (reg & 0x3) * 8;
	u8 ret = (in_be32(host->ioaddr + base) >> shift) & 0xff;

	/*
	 * "DMA select" locates at offset 0x28 in SD specification, but on
	 * P5020 or P3041, it locates at 0x29.
	 */
	if (reg == SDHCI_HOST_CONTROL) {
		u32 dma_bits;

		dma_bits = in_be32(host->ioaddr + reg);
		/* DMA select is 22,23 bits in Protocol Control Register */
		dma_bits = (dma_bits >> 5) & SDHCI_CTRL_DMA_MASK;

		/* fixup the result */
		ret &= ~SDHCI_CTRL_DMA_MASK;
		ret |= dma_bits;
	}

	return ret;
}

static void esdhc_writel(struct sdhci_host *host, u32 val, int reg)
{
	/*
	 * Enable IRQSTATEN[BGESEN] is just to set IRQSTAT[BGE]
	 * when SYSCTL[RSTD]) is set for some special operations.
	 * No any impact other operation.
	 */
	if (reg == SDHCI_INT_ENABLE)
		val |= SDHCI_INT_BLK_GAP;
	sdhci_be32bs_writel(host, val, reg);
}

static void esdhc_writew(struct sdhci_host *host, u16 val, int reg)
{
	if (reg == SDHCI_BLOCK_SIZE) {
		/*
		 * Two last DMA bits are reserved, and first one is used for
		 * non-standard blksz of 4096 bytes that we don't support
		 * yet. So clear the DMA boundary bits.
		 */
		val &= ~SDHCI_MAKE_BLKSZ(0x7, 0);
	}
	sdhci_be32bs_writew(host, val, reg);
}

static void esdhc_writeb(struct sdhci_host *host, u8 val, int reg)
{
	/*
	 * "DMA select" location is offset 0x28 in SD specification, but on
	 * P5020 or P3041, it's located at 0x29.
	 */
	if (reg == SDHCI_HOST_CONTROL) {
		u32 dma_bits;

		/* DMA select is 22,23 bits in Protocol Control Register */
		dma_bits = (val & SDHCI_CTRL_DMA_MASK) << 5;
		clrsetbits_be32(host->ioaddr + reg , SDHCI_CTRL_DMA_MASK << 5,
			dma_bits);
		val &= ~SDHCI_CTRL_DMA_MASK;
		val |= in_be32(host->ioaddr + reg) & SDHCI_CTRL_DMA_MASK;
	}

	/* Prevent SDHCI core from writing reserved bits (e.g. HISPD). */
	if (reg == SDHCI_HOST_CONTROL)
		val &= ~ESDHC_HOST_CONTROL_RES;

	/* If we have this quirk just use reset cmd and reset data to
	 * instead of reset all.
	 */
	if ((reg == SDHCI_SOFTWARE_RESET) &&
			(host->quirks2 & SDHCI_QUIRK2_BROKEN_RESET_ALL) &&
			(val & SDHCI_RESET_ALL))
		val = SDHCI_RESET_CMD | SDHCI_RESET_DATA;

	if (reg == SDHCI_POWER_CONTROL) {
		/* eSDHC don't support gate off power */
		if (!host->pwr || !val)
			return;

		if (fsl_svr_is(SVR_T4240)) {
			u8 vol;

			vol = sdhci_be32bs_readb(host, reg);
			if (host->pwr == SDHCI_POWER_180)
				vol &= ~ESDHC_VOL_SEL;
			else
				vol |= ESDHC_VOL_SEL;
		} else
			return;
	}

	sdhci_be32bs_writeb(host, val, reg);
}

/*
 * For Abort or Suspend after Stop at Block Gap, ignore the ADMA
 * error(IRQSTAT[ADMAE]) if both Transfer Complete(IRQSTAT[TC])
 * and Block Gap Event(IRQSTAT[BGE]) are also set.
 * For Continue, apply soft reset for data(SYSCTL[RSTD]);
 * and re-issue the entire read transaction from beginning.
 */
static void esdhci_of_adma_workaround(struct sdhci_host *host, u32 intmask)
{
	u32 tmp;
	bool applicable;
	dma_addr_t dmastart;
	dma_addr_t dmanow;

	tmp = in_be32(host->ioaddr + SDHCI_SLOT_INT_STATUS);
	tmp = (tmp & SDHCI_VENDOR_VER_MASK) >> SDHCI_VENDOR_VER_SHIFT;

	applicable = (intmask & SDHCI_INT_DATA_END) &&
		(intmask & SDHCI_INT_BLK_GAP) &&
		(tmp == VENDOR_V_23);
	if (applicable) {

		sdhci_reset(host, SDHCI_RESET_DATA);
		host->data->error = 0;
		dmastart = sg_dma_address(host->data->sg);
		dmanow = dmastart + host->data->bytes_xfered;
		/*
		 * Force update to the next DMA block boundary.
		 */
		dmanow = (dmanow & ~(SDHCI_DEFAULT_BOUNDARY_SIZE - 1)) +
			SDHCI_DEFAULT_BOUNDARY_SIZE;
		host->data->bytes_xfered = dmanow - dmastart;
		sdhci_writel(host, dmanow, SDHCI_DMA_ADDRESS);

		return;
	}

	/*
	 * Check for A-004388: eSDHC DMA might not stop if error
	 * occurs on system transaction
	 * Impact list:
	 * T4240-R1.0 B4860-R1.0 P1010-R1.0
	 */
	if (!((fsl_svr_is(SVR_T4240) && fsl_svr_rev_is(1, 0)) ||
		(fsl_svr_is(SVR_B4860) && fsl_svr_rev_is(1, 0)) ||
		(fsl_svr_is(SVR_P1010) && fsl_svr_rev_is(1, 0))))
		return;

	if (host->flags & SDHCI_USE_ADMA) {
		u32 mod, i, offset;
		u8 *desc;
		dma_addr_t addr;
		struct scatterlist *sg;

		mod = esdhc_readl(host, SDHCI_TRANSFER_MODE);
		if (mod & SDHCI_TRNS_BLK_CNT_EN) {
			/* In case read transfer there is no data
			 * was corrupted
			 */
			if (host->data->flags & MMC_DATA_READ)
				return;
			host->data->error = 0;
			sdhci_reset(host, SDHCI_RESET_DATA);
		}

		sdhci_reset(host, SDHCI_RESET_DATA);

		BUG_ON(!host->data);
		desc = host->adma_desc;
		for_each_sg(host->data->sg, sg, host->sg_count, i) {
			addr = sg_dma_address(sg);
			offset = (4 - (addr & 0x3)) & 0x3;
			if (offset)
				desc += 8;
			desc += 8;
		}

		/*
		 * Add an extra zero descriptor next to the
		 * terminating descriptor.
		 */
		desc += 8;
		WARN_ON((desc - host->adma_desc) > (128 * 2 + 1) * 4);

		if (host->quirks & SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC) {
			desc -= 8;
			desc[0] |= 0x2; /* end */
		} else {
			__le32 *dataddr = (__le32 __force *)(desc + 4);
			__le16 *cmdlen = (__le16 __force *)desc;

			cmdlen[0] = cpu_to_le16(0x3);
			cmdlen[1] = cpu_to_le16(0);

			dataddr[0] = cpu_to_le32(0);
		}

		return;
	}

	if ((host->flags & SDHCI_USE_SDMA)) {
		if (host->data->flags & MMC_DATA_READ)
			return;

		host->data->error = 0;
		sdhci_reset(host, SDHCI_RESET_DATA);
		return;
	}
}

static int esdhc_of_enable_dma(struct sdhci_host *host)
{
	setbits32(host->ioaddr + ESDHC_DMA_SYSCTL, ESDHC_DMA_SNOOP);
	return 0;
}

static unsigned int esdhc_of_get_max_clock(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	return pltfm_host->clock;
}

static unsigned int esdhc_of_get_min_clock(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	return pltfm_host->clock / 256 / 16;
}

static void esdhc_of_set_clock(struct sdhci_host *host, unsigned int clock)
{
	/* Workaround to reduce the clock frequency for p1010 esdhc */
	if (of_find_compatible_node(NULL, NULL, "fsl,p1010-esdhc")) {
		if (clock > 20000000)
			clock -= 5000000;
		if (clock > 40000000)
			clock -= 5000000;
	}

	/* Set the clock */
	esdhc_set_clock(host, clock);
}

#ifdef CONFIG_PM
static u32 esdhc_proctl;
static void esdhc_of_suspend(struct sdhci_host *host)
{
	esdhc_proctl = sdhci_be32bs_readl(host, SDHCI_HOST_CONTROL);
}

static void esdhc_of_resume(struct sdhci_host *host)
{
	sdhci_be32bs_writel(host, esdhc_proctl, SDHCI_HOST_CONTROL);
}
#endif

static u32 clock;
static void esdhc_of_platform_reset_enter(struct sdhci_host *host, u8 mask)
{
	if (host->quirks2 & SDHCI_QUIRK2_BROKEN_RESET_ALL)
		clock = host->clock;
}

static void esdhc_of_platform_reset_exit(struct sdhci_host *host, u8 mask)
{
	if (host->quirks2 & SDHCI_QUIRK2_BROKEN_RESET_ALL)
		host->clock = clock;
}

static void esdhc_of_platform_init(struct sdhci_host *host)
{
	u32 vvn;

	vvn = in_be32(host->ioaddr + SDHCI_SLOT_INT_STATUS);
	vvn = (vvn & SDHCI_VENDOR_VER_MASK) >> SDHCI_VENDOR_VER_SHIFT;
	if (vvn == VENDOR_V_22)
		host->quirks2 |= SDHCI_QUIRK2_HOST_NO_CMD23;

	if (vvn > VENDOR_V_22)
		host->quirks &= ~SDHCI_QUIRK_NO_BUSY_IRQ;

	/*
	 * Check for A-005055: A glitch is generated on the card clock
	 * due to software reset or a clock change
	 * Impact list:
	 * T4240-R1.0 B4860-R1.0 P3041-R1.0 P3041-R2.0 P2041-R1.0
	 * P2041-R1.1 P2041-R2.0 P1010-R1.0
	 */
	if ((fsl_svr_is(SVR_T4240) && fsl_svr_rev_is(1, 0)) ||
		(fsl_svr_is(SVR_B4860) && fsl_svr_rev_is(1, 0)) ||
		(fsl_svr_is(SVR_P3041) && fsl_svr_rev_is(1, 0)) ||
		(fsl_svr_is(SVR_P3041) && fsl_svr_rev_is(2, 0)) ||
		(fsl_svr_is(SVR_P2041) && fsl_svr_rev_is(2, 0)) ||
		(fsl_svr_is(SVR_P2041) && fsl_svr_rev_is(1, 1)) ||
		(fsl_svr_is(SVR_P2041) && fsl_svr_rev_is(1, 0)) ||
		(fsl_svr_is(SVR_P1010) && fsl_svr_rev_is(1, 0)))
		host->quirks2 |= SDHCI_QUIRK2_BROKEN_RESET_ALL;
}

/* Return: 1 - the card is present; 0 - card is absent */
static int esdhc_of_get_cd(struct sdhci_host *host)
{
	u32 present;
	u32 sysctl;

	if (host->flags & SDHCI_DEVICE_DEAD)
		return 0;
	if (host->quirks2 & SDHCI_QUIRK2_FORCE_CMD13_DETECT_CARD)
		return -ENOSYS;

	sysctl = sdhci_be32bs_readl(host, SDHCI_CLOCK_CONTROL);

	/* Enable the controller clock to update the present state */
	sdhci_be32bs_writel(host, sysctl | SDHCI_CLOCK_INT_EN,
			SDHCI_CLOCK_CONTROL);

	/* Detect the card present or absent */
	present = sdhci_be32bs_readl(host, SDHCI_PRESENT_STATE);
	present &= (SDHCI_CARD_PRESENT | SDHCI_CARD_CDPL);

	/* Resave the previous to System control register */
	sdhci_be32bs_writel(host, sysctl, SDHCI_CLOCK_CONTROL);

	return !!present;
}

static struct sdhci_ops sdhci_esdhc_ops = {
	.read_l = esdhc_readl,
	.read_w = esdhc_readw,
	.read_b = esdhc_readb,
	.write_l = esdhc_writel,
	.write_w = esdhc_writew,
	.write_b = esdhc_writeb,
	.set_clock = esdhc_of_set_clock,
	.enable_dma = esdhc_of_enable_dma,
	.get_max_clock = esdhc_of_get_max_clock,
	.get_min_clock = esdhc_of_get_min_clock,
	.platform_reset_enter = esdhc_of_platform_reset_enter,
	.platform_reset_exit = esdhc_of_platform_reset_exit,
	.platform_init = esdhc_of_platform_init,
	.get_cd = esdhc_of_get_cd,
#ifdef CONFIG_PM
	.platform_suspend = esdhc_of_suspend,
	.platform_resume = esdhc_of_resume,
#endif
	.adma_workaround = esdhci_of_adma_workaround,
};

static struct sdhci_pltfm_data sdhci_esdhc_pdata = {
	/*
	 * card detection could be handled via GPIO
	 * eSDHC cannot support End Attribute in NOP ADMA descriptor
	 */
	.quirks = ESDHC_DEFAULT_QUIRKS | SDHCI_QUIRK_BROKEN_CARD_DETECTION
		| SDHCI_QUIRK_NO_CARD_NO_RESET
		| SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC,
	.ops = &sdhci_esdhc_ops,
};

static int sdhci_esdhc_probe(struct platform_device *pdev)
{
	return sdhci_pltfm_register(pdev, &sdhci_esdhc_pdata);
}

static int sdhci_esdhc_remove(struct platform_device *pdev)
{
	return sdhci_pltfm_unregister(pdev);
}

static const struct of_device_id sdhci_esdhc_of_match[] = {
	{ .compatible = "fsl,mpc8379-esdhc" },
	{ .compatible = "fsl,mpc8536-esdhc" },
	{ .compatible = "fsl,esdhc" },
	{ }
};
MODULE_DEVICE_TABLE(of, sdhci_esdhc_of_match);

static struct platform_driver sdhci_esdhc_driver = {
	.driver = {
		.name = "sdhci-esdhc",
		.owner = THIS_MODULE,
		.of_match_table = sdhci_esdhc_of_match,
		.pm = SDHCI_PLTFM_PMOPS,
	},
	.probe = sdhci_esdhc_probe,
	.remove = sdhci_esdhc_remove,
};

module_platform_driver(sdhci_esdhc_driver);

MODULE_DESCRIPTION("SDHCI OF driver for Freescale MPC eSDHC");
MODULE_AUTHOR("Xiaobo Xie <X.Xie@freescale.com>, "
	      "Anton Vorontsov <avorontsov@ru.mvista.com>");
MODULE_LICENSE("GPL v2");
