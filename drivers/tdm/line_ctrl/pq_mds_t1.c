/*
 * drivers/tdm/line/pq_mds_t1.c
 *
 * Copyright 2011-2012 Freescale Semiconductor, Inc.
 *
 * T1/E1 PHY(DS26528) Control Module for Freescale PQ-MDS-T1 card.
 *
 * Author: Kai Jiang <Kai.Jiang@freescale.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/param.h>
#include <linux/delay.h>
#include <linux/of.h>

#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/io.h>

#include "pq_mds_t1.h"

/* judge if pq-mds-t1 board is connected already */
static int pq_mds_t1_connected(struct pq_mds_t1 *t1_info)
{
	struct pld_mem *pld_base = t1_info->pld_base;
	u8 brdrev, pldrev;

	brdrev = in_8(&pld_base->brdrev);
	pldrev = in_8(&pld_base->pldrev);

	if (brdrev != PQ_MDS_8E1T1_BRD_REV ||
	    pldrev != PQ_MDS_8E1T1_PLD_REV) {
		return -ENODEV;
	}

	return 0;
}

/* setup pq-mds-t1 board basic clock */
static void pq_mds_t1_clock_set(struct pq_mds_t1 *pq_mds_t1_info)
{
	struct ds26528_mem *ds26528 = pq_mds_t1_info->ds26528_base;
	struct pld_mem *pld = pq_mds_t1_info->pld_base;

	if (pq_mds_t1_info->card_support == LM_CARD) {
		/* General clock configuration
		 * Altera register setting: Framer DIGIOEN and TXEN active */
		out_8(&pld->pinset,
			PLD_PINSET_DIGIOEN | PLD_PINSET_TXEN);
		/* Drive MCLK & REFCLKIO with 2.048MHz */
		out_8(&pld->csr,
			PLD_CSR_RCSRC_2048KHZ | PLD_CSR_MSRC_2048KHZ);
		/* Drive TSYSCLK & RSYSCLK with 2.048MHz */
		out_8(&pld->sysclk_tr,
			PLD_SYSCLK_RS_2048KHZ | PLD_SYSCLK_TS_2048KHZ);
		/* Not using transmit-side elastic store,
		 * tri-state - PLD pin connected to TSSYNCIO */
		out_8(&pld->synctss, PLD_SYNCTSS_TRI_STATE);
		/* Drive TCLK1..4 with 2.048MHz*/
		out_8(&pld->tcsr1, PLD_TCSR1_2048KHZ);
		/* Drive TCLK5..8 with 2.048MHz*/
		out_8(&pld->tcsr2, PLD_TCSR2_2048KHZ);

		/* Connect tdm port A to LM card */
		out_8(&pld->gcr, PLD_GCR_LCC_TDMA | PLD_GCR_LCE_LM);
		/* Framer setting
		 * Select MCLK - 2.048MHz, REFCLKIO - 1.544MHz (GTCCR) */
		out_8(&ds26528->link[0].gbl.gtccr,
			DS26528_GTCCR_BPREFSEL_REFCLKIN |
			DS26528_GTCCR_BFREQSEL_1544KHZ |
			DS26528_GTCCR_FREQSEL_1544KHZ);
		udelay(100);

		/* Set TSSYNCIO -> OUTPUT (GTCR2) */
		out_8(&ds26528->link[0].gbl.gtcr2, DS26528_GTCR2_TSSYNCOUT);
		/* Select BPCLK=2.048MHz (GFCR) */
		out_8(&ds26528->link[0].gbl.gfcr, DS26528_GFCR_BPCLK_2048KHZ);
	} else {
		if (pq_mds_t1_info->line_rate == LINE_RATE_T1) {
			/* General clock configuration
			 * Altera register setting: Framer DIGIOEN
			 * and TXEN active */
			out_8(&pld->pinset,
				PLD_PINSET_DIGIOEN | PLD_PINSET_TXEN);
			/* Drive MCLK & REFCLKIO with 1.544MHz */
			out_8(&pld->csr,
				PLD_CSR_RCSRC_1544KHZ | PLD_CSR_MSRC_1544KHZ);
			/* Drive TSYSCLK & RSYSCLK with 1.544MHz */
			out_8(&pld->sysclk_tr,
				PLD_SYSCLK_RS_1544KHZ | PLD_SYSCLK_TS_1544KHZ);
			/* Framer setting
			 * Select MCLK - 1.544MHz,
			 * REFCLKIO - 1.544MHz (GTCCR) */
			out_8(&ds26528->link[0].gbl.gtccr,
				DS26528_GTCCR_BPREFSEL_REFCLKIN |
				DS26528_GTCCR_BFREQSEL_1544KHZ |
				DS26528_GTCCR_FREQSEL_1544KHZ);
			udelay(100);
			/* Set TSSYNCIO -> OUTPUT (GTCR2) */
			out_8(&ds26528->link[0].gbl.gtcr2,
				DS26528_GTCR2_TSSYNCOUT);
			/* Select BPCLK=2.048MHz (GFCR) */
			out_8(&ds26528->link[0].gbl.gfcr,
				DS26528_GFCR_BPCLK_2048KHZ);
		} else {
			/* General clock configuration
			 * Altera register setting: Framer DIGIOEN
			 * and TXEN active */
			out_8(&pld->pinset,
				PLD_PINSET_DIGIOEN | PLD_PINSET_TXEN);
			/* Drive MCLK & REFCLKIO with 2.048MHz */
			out_8(&pld->csr,
				PLD_CSR_RCSRC_2048KHZ | PLD_CSR_MSRC_2048KHZ);
			/* Drive TSYSCLK & RSYSCLK with 2.048MHz */
			out_8(&pld->sysclk_tr,
				PLD_SYSCLK_RS_2048KHZ | PLD_SYSCLK_TS_2048KHZ);

			/* Framer setting
			 * Select MCLK - 2.048MHz,
			 * REFCLKIO - 2.048MHz (GTCCR) */
			out_8(&ds26528->link[0].gbl.gtccr,
				DS26528_GTCCR_BPREFSEL_REFCLKIN |
				DS26528_GTCCR_BFREQSEL_2048KHZ |
				DS26528_GTCCR_FREQSEL_2048KHZ);
			udelay(100);
			/* Set TSSYNCIO -> OUTPUT (GTCR2) */
			out_8(&ds26528->link[0].gbl.gtcr2,
				DS26528_GTCR2_TSSYNCOUT);
			/* Select BPCLK=2.048MHz (GFCR) */
			out_8(&ds26528->link[0].gbl.gfcr,
				DS26528_GFCR_BPCLK_2048KHZ);
		}
	}
}

/* setup the TCLK to T1 rate for DS26528 */
static void card_pld_t1_clk_set(struct pq_mds_t1 *pq_mds_t1_info)
{
	struct pld_mem *pld = pq_mds_t1_info->pld_base;
	/* Drive TCLK1..4 with 1.544MHz*/
	out_8(&pld->tcsr1, PLD_TCSR1_1544KHZ);
	/* Drive TCLK5..8 with 1.544MHz*/
	out_8(&pld->tcsr2, PLD_TCSR2_1544KHZ);

	udelay(100);
}

/* setup the TCLK to E1 rate for DS26528 */
static void card_pld_e1_clk_set(struct pq_mds_t1 *pq_mds_t1_info)
{
	struct pld_mem *pld = pq_mds_t1_info->pld_base;
	/* Drive TCLK1..4 with 2.048MHz*/
	out_8(&pld->tcsr1, PLD_TCSR1_2048KHZ);
	/* Drive TCLK5..8 with 2.048MHz*/
	out_8(&pld->tcsr2, PLD_TCSR2_2048KHZ);

	udelay(100);
}


static void ds26528_bulk_write_set(struct pq_mds_t1 *pq_mds_t1_info)
{
	/* Select Bulk write for Framer Register Init (GTCR1) */
	out_8(&(pq_mds_t1_info->ds26528_base->link[0].gbl.gtcr1),
		DS26528_GTCR1_BWE);
}

static void ds26528_bulk_write_unset(struct pq_mds_t1 *pq_mds_t1_info)
{
	/* Unselect Bulk write for Framer Register Init (GTCR1) */
	out_8(&(pq_mds_t1_info->ds26528_base->link[0].gbl.gtcr1), 0x00);
}

/* reset LIU of all channels on ds26528 */
static void ds26528_gbl_sreset(struct pq_mds_t1 *pq_mds_t1_info)
{
	struct ds26528_mem *ds26528 = pq_mds_t1_info->ds26528_base;

	/* Global LIU Software Reset Register (GLSRR) */
	out_8(&ds26528->link[0].gbl.glsrr, DS26528_GLSRR_Reset);
	/* Global Framer and BERT Software Reset Register (GFSRR) */
	out_8(&ds26528->link[0].gbl.gfsrr, DS26528_GFSRR_Reset);

	udelay(100);

	out_8(&ds26528->link[0].gbl.glsrr, DS26528_GLSRR_NORMAL);
	out_8(&ds26528->link[0].gbl.gfsrr, DS26528_GFSRR_NORMAL);
}

/* reset recieve and transimt framer for a channel on ds26528 */
static int ds26528_rx_tx_sreset(struct pq_mds_t1 *pq_mds_t1_info, u8 phy_id)
{
	struct ds26528_mem *ds26528 = pq_mds_t1_info->ds26528_base;

	if (phy_id > MAX_NUM_OF_CHANNELS)
		return -ENODEV;

	/* Perform RX/TX SRESET,Reset receiver (RMMR) */
	out_8(&ds26528->link[phy_id].rx.rmmr, DS26528_RMMR_SFTRST);
	/* Reset tranceiver (TMMR) */
	out_8(&ds26528->link[phy_id].tx.tmmr, DS26528_TMMR_SFTRST);

	udelay(100);

	return 0;
}

/* clear framer LIU BERT of a channel on ds26528 */
static void ds26528_frame_regs_clear(struct pq_mds_t1 *pq_mds_t1_info,
					u8 phy_id)
{
	struct ds26528_mem *ds26528 = pq_mds_t1_info->ds26528_base;
	/* Zero all Framer Registers */
	memset(&ds26528->link[phy_id].rx, 0, sizeof(struct rx_frame));
	memset(&ds26528->link[phy_id].tx, 0, sizeof(struct tx_frame));
	memset(&ds26528->liu[phy_id], 0, sizeof(struct liu_reg));
	memset(&ds26528->bert[phy_id], 0, sizeof(struct bert_reg));
}

/* setup the ds26825 mode, loopback or normal */
static void ds26528_trans_mode_set(struct pq_mds_t1 *pq_mds_t1_info, u8 phy_id)
{
	struct ds26528_mem *ds26528 = pq_mds_t1_info->ds26528_base;

	switch (pq_mds_t1_info->trans_mode) {
	case FRAMER_LB:
		out_8(&ds26528->link[phy_id].rx.rcr3, DS26528_RCR3_FLB);
		break;
	case NORMAL:
		break;
	default:
		dev_err(pq_mds_t1_info->dev, "No such transfer mode\n");
	}
}

/* setup ds26528 for T1 specification */
static void ds26528_t1_spec_config(struct pq_mds_t1 *pq_mds_t1_info, u8 phy_id)
{
	struct ds26528_mem *ds26528 = pq_mds_t1_info->ds26528_base;
	/* Receive T1 Mode (Receive Master Mode Register - RMMR)
	   Framer Disabled */
	out_8(&ds26528->link[phy_id].rx.rmmr, DS26528_RMMR_T1);
	/* Transmit T1 Mode (Transmit Master Mode Register - TMMR)
	   Framer Disabled */
	out_8(&ds26528->link[phy_id].tx.tmmr, DS26528_TMMR_T1);
	/* Receive T1 Mode Framer Enable (RMMR - Framer Enabled/T1) */
	out_8(&ds26528->link[phy_id].rx.rmmr,
		DS26528_RMMR_FRM_EN | DS26528_RMMR_T1);
	/* Transmit T1 Mode Framer Enable (TMMR - Framer Enabled/T1) */
	out_8(&ds26528->link[phy_id].tx.tmmr,
		DS26528_TMMR_FRM_EN | DS26528_TMMR_T1);
	/* RCR1, receive T1 B8zs & ESF (Receive Control Register 1 - T1 MODE) */
	out_8(&ds26528->link[phy_id].rx.rcr1,
		DS26528_RCR1_T1_SYNCT |
		DS26528_RCR1_T1_RB8ZS |
		DS26528_RCR1_T1_SYNCC);
	/* RIOCR  (RSYSCLK=1.544MHz, RSYNC-Output) */
	out_8(&ds26528->link[phy_id].rx.riocr,
		DS26528_RIOCR_1544KHZ | DS26528_RIOCR_RSIO_OUT);
	/* TCR1 Transmit T1 b8zs*/
	out_8(&ds26528->link[phy_id].tx.tcr[0], DS26528_TCR1_TB8ZS);
	/* TIOCR (TSYSCLK=1.544MHz, TSYNC-Output) */
	out_8(&ds26528->link[phy_id].tx.tiocr,
		DS26528_TIOCR_1544KHZ | DS26528_TIOCR_TSIO_OUT);
	/* Receive T1 Mode Framer Enable & init Done */
	out_8(&ds26528->link[phy_id].rx.rmmr,
		DS26528_RMMR_FRM_EN | DS26528_RMMR_INIT_DONE);
	/* Transmit T1 Mode Framer Enable & init Done */
	out_8(&ds26528->link[phy_id].tx.tmmr,
		DS26528_TMMR_FRM_EN | DS26528_TMMR_INIT_DONE);
	/* Configure LIU (LIU Transmit Receive Control Register
	   - LTRCR. T1 mode) */
	out_8(&ds26528->liu[phy_id].ltrcr, DS26528_LTRCR_T1);
	/* T1 Mode default 100 ohm 0db CSU (LIU Transmit Impedance and
	   Pulse Shape Selection Register - LTITSR)*/
	out_8(&ds26528->liu[phy_id].ltitsr,
		DS26528_LTITSR_TLIS_100OHM | DS26528_LTITSR_TLIS_0DB_CSU);
	/* T1 Mode default 100 ohm Long haul (LIU Receive Impedance and
	   Sensitivity Monitor Register - LRISMR)*/
	out_8(&ds26528->liu[phy_id].lrismr,
		DS26528_LRISMR_100OHM | DS26528_LRISMR_MAX);
	/* Enable Transmit output (LIU Maintenance Control Register - LMCR) */
	out_8(&ds26528->liu[phy_id].lmcr, DS26528_LMCR_TE);
}

/* setup ds26528 for E1 specification */
static void ds26528_e1_spec_config(struct pq_mds_t1 *pq_mds_t1_info, u8 phy_id)
{
	struct ds26528_mem *ds26528 = pq_mds_t1_info->ds26528_base;

	/* Receive E1 Mode (Receive Master Mode Register - RMMR)
	   Framer Disabled */
	out_8(&ds26528->link[phy_id].rx.rmmr, DS26528_RMMR_E1);
	/* Transmit E1 Mode (Transmit Master Mode Register - TMMR)
	   Framer Disabled*/
	out_8(&ds26528->link[phy_id].tx.tmmr, DS26528_TMMR_E1);
	/* Receive E1 Mode Framer Enable (RMMR - Framer Enabled/E1)*/
	out_8(&ds26528->link[phy_id].rx.rmmr,
		DS26528_RMMR_FRM_EN | DS26528_RMMR_E1);
	/* Transmit E1 Mode Framer Enable (TMMR - Framer Enabled/E1)*/
	out_8(&ds26528->link[phy_id].tx.tmmr,
		DS26528_TMMR_FRM_EN | DS26528_TMMR_E1);
	/* RCR1, receive E1 B8zs & ESF (Receive Control Register 1 - E1 MODE)*/
	out_8(&ds26528->link[phy_id].rx.rcr1,
		DS26528_RCR1_E1_HDB3 | DS26528_RCR1_E1_CCS);
	/* RIOCR (RSYSCLK=2.048MHz, RSYNC-Output) */
	out_8(&ds26528->link[phy_id].rx.riocr,
		DS26528_RIOCR_2048KHZ | DS26528_RIOCR_RSIO_OUT);
	/* TCR1 Transmit E1 b8zs */
	out_8(&ds26528->link[phy_id].tx.tcr[0], DS26528_TCR1_TB8ZS);
	/* TIOCR (TSYSCLK=2.048MHz, TSYNC-Output) */
	out_8(&ds26528->link[phy_id].tx.tiocr,
		DS26528_TIOCR_2048KHZ | DS26528_TIOCR_TSIO_OUT);
	/* Set E1TAF (Transmit Align Frame Register regsiter) */
	out_8(&ds26528->link[phy_id].tx.e1taf, DS26528_E1TAF_DEFAULT);
	/* Set E1TNAF register (Transmit Non-Align Frame Register) */
	out_8(&ds26528->link[phy_id].tx.e1tnaf, DS26528_E1TNAF_DEFAULT);
	/* Receive E1 Mode Framer Enable & init Done (RMMR) */
	out_8(&ds26528->link[phy_id].rx.rmmr,
		DS26528_RMMR_FRM_EN |
		DS26528_RMMR_INIT_DONE |
		DS26528_RMMR_E1);
	/* Transmit E1 Mode Framer Enable & init Done (TMMR) */
	out_8(&ds26528->link[phy_id].tx.tmmr,
		DS26528_TMMR_FRM_EN |
		DS26528_TMMR_INIT_DONE |
		DS26528_TMMR_E1);
	/* Configure LIU (LIU Transmit Receive Control Register
	   - LTRCR. E1 mode) */
	out_8(&ds26528->liu[phy_id].ltrcr, DS26528_LTRCR_E1);
	/* E1 Mode default 75 ohm w/Transmit Impedance Matlinking
	 (LIU Transmit Impedance and Pulse Shape Selection Register - LTITSR)*/
	out_8(&ds26528->liu[phy_id].ltitsr,
		DS26528_LTITSR_TLIS_75OHM | DS26528_LTITSR_LBOS_75OHM);
	/* E1 Mode default 75 ohm Long Haul w/Receive Impedance Matlinking
	  (LIU Receive Impedance and Sensitivity Monitor Register - LRISMR)*/
	out_8(&ds26528->liu[phy_id].lrismr,
		DS26528_LRISMR_75OHM | DS26528_LRISMR_MAX);
	/* Enable Transmit output (LIU Maintenance Control Register - LMCR) */
	out_8(&ds26528->liu[phy_id].lmcr, DS26528_LMCR_TE);
}

int ds26528_t1_e1_config(struct pq_mds_t1 *pq_mds_t1_info, u8 phy_id)
{
	struct pq_mds_t1 *card_info = pq_mds_t1_info;

	ds26528_bulk_write_set(card_info);
	ds26528_gbl_sreset(card_info);
	switch (card_info->line_rate) {
	case LINE_RATE_T1:
		card_pld_t1_clk_set(card_info);
		break;
	case LINE_RATE_E1:
		card_pld_e1_clk_set(card_info);
		break;
	default:
		dev_err(pq_mds_t1_info->dev,
			"Support E1/T1. Frame mode not support.\n");
		return -ENODEV;
		break;
	}
	ds26528_rx_tx_sreset(card_info, phy_id);
	ds26528_frame_regs_clear(card_info, phy_id);
	ds26528_trans_mode_set(card_info, phy_id);

	switch (card_info->line_rate) {
	case LINE_RATE_T1:
		ds26528_t1_spec_config(card_info, phy_id);
		break;
	case LINE_RATE_E1:
		ds26528_e1_spec_config(card_info, phy_id);
		break;
	default:
		dev_err(pq_mds_t1_info->dev,
			"Support E1/T1. Frame mode not support\n");
		return -ENODEV;
		break;
	}
	ds26528_bulk_write_unset(card_info);

	return 0;
}

static enum line_rate_t set_phy_line_rate(const char *line_rate)
{
	if (strcasecmp(line_rate, "e1") == 0)
		return LINE_RATE_E1;
	else
		return LINE_RATE_T1;
}

static enum card_support_type set_card_support_type(const phandle *card_type)
{
	struct device_node *np;
	enum card_support_type type;

	np = of_find_node_by_phandle(*card_type);
	if (!np)
		return NO_CARD;

	if (of_device_is_compatible(np, "dallas,ds26528"))
		type =  DS26528_CARD;
	else
		type =  LM_CARD;

	of_node_put(np);

	return type;
}

static enum tdm_trans_mode_t set_phy_trans_mode(const char *trans_mode)
{
	if (strcasecmp(trans_mode, "framer-loopback") == 0)
		return FRAMER_LB;
	else
		return NORMAL;
}

static int pq_mds_t1_probe(struct platform_device *pdev)
{
	int err = 0;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *child;
	struct resource res;
	struct pq_mds_t1 *t1_info;
	const unsigned char *prop;
	const phandle *ph;

	t1_info = kzalloc(sizeof(struct pq_mds_t1), GFP_KERNEL);
	if (!t1_info) {
		dev_err(&pdev->dev, "No memory to alloc pq_mds_t1\n");
		return -ENOMEM;
	}
	dev_set_drvdata(&pdev->dev, t1_info);
	t1_info->dev = &pdev->dev;

	for_each_child_of_node(np, child)
		if (of_device_is_compatible(child, "fsl,pq-mds-t1-pld"))
			break;
	if (!child) {
		dev_err(&pdev->dev, "Invalid fsl,pq-mds-t1-pld property\n");
		err =  -ENODEV;
		goto err_miss_pld_property;
	}

	err = of_address_to_resource(child, 0, &res);
	if (err) {
		err = -ENODEV;
		goto err_miss_pld_property;
	}
	t1_info->pld_base = ioremap(res.start, res.end - res.start + 1);

	ph = of_get_property(child, "fsl,card-support", NULL);
	if (!ph) {
		err = -ENODEV;
		dev_err(&pdev->dev, "Invalid card-support property\n");
		goto err_miss_pld_property;
	}

	t1_info->card_support = set_card_support_type(ph);
	if (t1_info->card_support == NO_CARD) {
		dev_err(&pdev->dev, "Couldn't get tdm phy phandle\n");
		err = -ENODEV;
		goto err_miss_pld_property;
	}

	for_each_child_of_node(np, child)
		if (of_device_is_compatible(child, "dallas,ds26528"))
			break;

	if (!child) {
		dev_err(&pdev->dev, "Invalid dallas,ds26528 property\n");
		err = -ENODEV;
		goto err_miss_phy_property;

	}

	err = of_address_to_resource(child, 0, &res);
	if (err) {
		err = -ENODEV;
		goto err_miss_phy_property;
	}

	t1_info->ds26528_base = ioremap(res.start, res.end - res.start + 1);

	err = pq_mds_t1_connected(t1_info);
	if (err) {
		err = -ENODEV;
		dev_err(&pdev->dev, "No PQ_MDS_T1 CARD\n");
		goto err_card_unplugin;
	}

	prop = of_get_property(child, "line-rate", NULL);
	if (!prop) {
		err = -ENODEV;
		dev_err(&pdev->dev, "Invalid line-rate property\n");
		goto err_card_unplugin;
	}

	t1_info->line_rate = set_phy_line_rate(prop);

	prop = of_get_property(child, "trans-mode", NULL);
	if (!prop) {
		err = -ENODEV;
		dev_err(&pdev->dev, "Invalid trans-mode property\n");
		goto err_card_unplugin;
	}
	t1_info->trans_mode = set_phy_trans_mode(prop);

	pq_mds_t1_clock_set(t1_info);
	if (t1_info->card_support == DS26528_CARD)
		ds26528_t1_e1_config(t1_info, 0);

	dev_info(&pdev->dev, "registed pq-mds-t1 card\n");

	return 0;

err_card_unplugin:
	iounmap(t1_info->ds26528_base);
err_miss_phy_property:
	iounmap(t1_info->pld_base);
err_miss_pld_property:
	kfree(t1_info);

	return err;
}

static int pq_mds_t1_remove(struct platform_device *pdev)
{
	struct pq_mds_t1 *t1_info = dev_get_drvdata(&pdev->dev);

	iounmap(t1_info->pld_base);
	iounmap(t1_info->ds26528_base);
	kfree(t1_info);

	return 0;
}

static struct of_device_id pq_mds_t1_of_match[] = {
	{ .compatible = "fsl,pq-mds-t1" },
	{},
};

static struct platform_driver pq_mds_t1_driver = {
	.probe	= pq_mds_t1_probe,
	.remove	= pq_mds_t1_remove,
	.driver	= {
		.name		= "pq_mds_t1",
		.owner		= THIS_MODULE,
		.of_match_table	= pq_mds_t1_of_match,
	},
};

static int __init pq_mds_t1_init(void)
{
	return platform_driver_register(&pq_mds_t1_driver);
}

static void __exit pq_mds_t1_exit(void)
{
	platform_driver_unregister(&pq_mds_t1_driver);
}

MODULE_AUTHOR("Jiang Kai");
MODULE_DESCRIPTION("Freescale Developed PQ_MDS_T1CARD Driver");
MODULE_LICENSE("GPL");

module_init(pq_mds_t1_init);
module_exit(pq_mds_t1_exit);
