/*
 * Freescale QUICC Engine TDM Device Driver
 *
 * Copyright 2011-2012 Freescale Semiconductor, Inc.
 *
 * Author: Haiying Wang <Haiying.Wang@freescale.com>
 *		    Kai Jiang	   <Kai.Jiang@freescale.com>
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
 *
 * This driver adds support for TDM devices via Freescale's QUICC Engine.
 */


#ifndef CONFIG_UCC_TDM_H
#define CONFIG_UCC_TDM_H

#include <linux/kernel.h>
#include <linux/list.h>

#include <asm/immap_qe.h>
#include <asm/qe.h>

#include <asm/ucc.h>
#include <asm/ucc_fast.h>

/* SI RAM entries */
#define SIR_LAST	0x0001
#define SIR_BYTE	0x0002
#define SIR_CNT(x)	((x) << 2)
#define SIR_CSEL(x)	((x) << 5)
#define SIR_SGS		0x0200
#define SIR_SWTR	0x4000
#define SIR_MCC		0x8000
#define SIR_IDLE	0

/* SIxMR fields */
#define SIMR_SAD(x) ((x) << 12)
#define SIMR_SDM_NORMAL	0x0000
#define SIMR_SDM_INTERNAL_LOOPBACK	0x0800
#define SIMR_SDM_MASK	0x0c00
#define SIMR_CRT	0x0040
#define SIMR_SL		0x0020
#define SIMR_CE		0x0010
#define SIMR_FE		0x0008
#define SIMR_GM		0x0004
#define SIMR_TFSD(n)	(n)
#define SIMR_RFSD(n)	((n) << 8)

enum tdm_ts_t {
	TDM_TX_TS,
	TDM_RX_TS
};


enum tdm_framer_t {
	TDM_FRAMER_T1,
	TDM_FRAMER_E1
};

enum tdm_mode_t {
	TDM_INTERNAL_LOOPBACK,
	TDM_NORMAL
};

struct ucc_transparent_param {
	__be16 riptr;
	__be16 tiptr;
	__be16 res0;
	__be16 mrblr;
	__be32 rstate;
	__be32 rbase;
	__be16 rbdstat;
	__be16 rbdlen;
	__be32 rdptr;
	__be32 tstate;
	__be32 tbase;
	__be16 tbdstat;
	__be16 tbdlen;
	__be32 tdptr;
	__be32 rbptr;
	__be32 tbptr;
	__be32 rcrc;
	__be32 res1;
	__be32 tcrc;
	__be32 res2;
	__be32 res3;
	__be32 c_mask;
	__be32 c_pres;
	__be16 disfc;
	__be16 crcec;
	__be32 res4[4];
	__be16 ts_tmp;
	__be16 tmp_mb;
} __attribute__ ((__packed__));

struct si_mode_info {
	u8 simr_rfsd;
	u8 simr_tfsd;
	u8 simr_crt;
	u8 simr_sl;
	u8 simr_ce;
	u8 simr_fe;
	u8 simr_gm;
};

struct ucc_tdm_info {
	struct ucc_fast_info uf_info;
	struct si_mode_info si_info;
};

struct ucc_tdm_private {
	struct ucc_tdm_info *ut_info;
	struct ucc_fast_private *uccf;
	struct device *dev;
	struct ucc_fast __iomem *uf_regs;	/* UCC Fast registers */
	struct si1 __iomem *si_regs;
	struct ucc_transparent_param __iomem *ucc_pram;
	u16 tdm_port;		/* port for this tdm:TDMA,TDMB,TDMC,TDMD */
	u32 siram_entry_id;
	u16 __iomem *siram;
	enum tdm_mode_t tdm_mode;
	enum tdm_framer_t tdm_framer_type;
	bool tdm_busy;
	u8 num_of_ts;		/* the number of timeslots in this tdm frame */
	u32 tx_ts_mask;		/* tx time slot mask */
	u32 rx_ts_mask;		/*rx time slot mask */
	u8 *rx_buffer;		/* buffer used for Rx by the tdm */
	u8 *tx_buffer;		/* buffer used for Tx by the tdm */
	dma_addr_t dma_rx_addr;	/* dma mapped buffer for TDM Rx */
	dma_addr_t dma_tx_addr;	/* dma mapped buffer for TDM Tx */
	struct qe_bd *tx_bd;
	struct qe_bd *rx_bd;
	u8 phase_rx;
	u8 phase_tx;
	u32 ucc_pram_offset;
	u32 tx_bd_offset;
	u32 rx_bd_offset;
	spinlock_t tdmlock;
	wait_queue_head_t tdm_queue;
	bool tdm_queue_flag;
	struct tdm_adapter adap;
};

#define NUM_OF_BUF	4
#define MAX_RX_BUF_LENGTH	(72*0x20)
#define ALIGNMENT_OF_UCC_TRANS_PRAM	64
#define SI_BANK_SIZE	128
#define MAX_TDM_NUM	8
#define BD_LEN_MASK	0xffff

#endif
