/*
 * drivers/device/fsl_tdm.h
 *
 * Copyright (C) 2007-2012 Freescale Semiconductor, Inc, All rights reserved.
 *
 * Created by P. V. Suresh <pala@freescale.com>
 *
 * Modified by Rajesh Gumasta <rajesh.gumasta@freescale.com>
 * 1. Modified to support MPC85xx based devices
 * 2. Modified the priv structure to support Adapter Registeration
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
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

#ifndef FSL_TDM_H
#define FSL_TDM_H

/* enable clock to TDM */
#ifdef CONFIG_MPC831x_RDB

#define SCCR_OFFSET 0x0A08
#define SCCR_TDM_MASK 0x000000C0
#define TDM_CM_01 (0x01<<6)

/* enable tdm in SICR */
#define SICRL_OFFSET 0x0114
#define SICRL_TDM_MASK 0xF00F0000

#endif

/* TDM data register offset */
#define TDM_TDR_OFFSET 0x108
#define TDM_RDR_OFFSET 0x100
#define TDM_DATAREG_OFFSET 0x100
#define TDM_CLKREG_OFFSET 0x180

/* max number of TDM-DMA channels */
#define DMA_MAX_CHANNELS 4

/* TCD params */
#define SOFF_VAL	0x08
#define DOFF_VAL	0x08
#define NBYTES		0x08 /*Minor Bytes transfer count*/
#define SLAST		0x00 /* last source addr adjustment*/
#define SLAST_SGA	0x00
#define DLAST_SGA	0x00

/* RIR Params*/
#define RIR_RFSD_VAL	0x01
#define RIR_RFWM_VAL	0x00

/* TIR Params*/
#define TIR_RFSD_VAL	0x01
#define TIR_RFWM_VAL	0x00

/* TDMTCEN */
#define NUM_TDMTCEN_REG		0x04
#define TDMTCEN_REG_LEN		32

/* each DMA-ch contains 8 Transfer Control Discriptors  */
#define DMA_MAX_TCD 8

#define DMAC_TX_INT 1
#define DMAC_RX_INT 2

#define TDM_CHANNEL_8BIT_LIN        0x00000000      /* 8 bit linear */
#define TDM_CHANNEL_8BIT_ULAW       0x00000001      /* 8 bit Mu-law */
#define TDM_CHANNEL_8BIT_ALAW       0x00000002      /* 8 bit A-law */
#define TDM_CHANNEL_16BIT_LIN       0x00000003      /* 16 bit Linear */

/* DMAC TCD structure */
struct tcd {
	u32 tcd[DMA_MAX_TCD];
};

/* DMA Controllor */
struct dmac_regs {
	u32 dmacr;		/* DMA Control Register			*/
	u32 dmaes;		/* DMA Error Status Register		*/
	u32 dmaerqh;		/* DMA Enable Request			*/
	u32 dmaerql;		/* DMA Enable Request			*/
	u32 dmaeeih;		/* DMA Enable Error Interrupt		*/
	u32 dmaeeil;		/* DMA Enable Error Interrupt		*/

	u8 dmaserq;		/* DMA Set Enable Request		*/
	u8 dmacerq;		/* DMA Clear Enable Request		*/
	u8 dmaseei;		/* DMA Set Enable Error Interrupt	*/
	u8 dmaceei;		/* DMA Clear Enable Error Interrupt	*/

	u8 dmacint;		/* DMA Clear Interrupt Request		*/
	u8 dmacerr;		/* DMA Clear Error			*/
	u8 dmassrt;		/* DMA Set Start Bit			*/
	u8 dmacdne;		/* DMA Clear Done Bit			*/

	u32 dmainth;		/* DMA Interrupt Request High		*/
	u32 dmaintl;		/* DMA Interrupt Request		*/
	u32 dmaerrh;		/* DMA Error				*/
	u32 dmaerrl;		/* DMA Error				*/
	u32 dmahrsh;		/* DMA Hardware Request status		*/
	u32 dmahrsl;		/* DMA HardWired Request status		*/
	u32 dmagpor;		/* DMA General Purpose Register		*/
	u8 reserved0[0xC4];
	u8 dchpri[DMA_MAX_CHANNELS];	/* DMA Port Priority		*/
	u8 reserved1[0xEFC];
	struct tcd tcd[DMA_MAX_CHANNELS];	/*Transfer Control Descriptor */
};

/* DMA GPOR */
#define DMAGPOR_SNOOP	0x00000040	/* Enable Snooping */

/* DMA Control Register (DMACR) */
#define DMACR_EMLM	0x00000080	/* Enable Minor loop Mapping */
#define DMACR_CLM	0x00000040	/* Continuous link mode */
#define DMACR_HALT	0x00000020	/* Halt DMA */
#define DMACR_HOE	0x00000010	/* Halt on Error */
#define DMACR_ERGA	0x00000008	/* Round robin among the groups */
#define DMACR_ERCA	0x00000004	/* Round robin Port Arbitration */
#define DMACR_EDBG	0x00000002	/* Debug */
#define DMACR_EBW	0x00000001	/* Enable Buffer */

/* DMA Error Status DMAES */
#define DMAES_VLD	0x80000000	/* Logical OR of all DMA errors. */
#define DMAES_ECX	0x00010000	/* Transfer cancelled */
#define DMAES_GPE	0x00008000	/* Group priority error */
#define DMAES_CPE	0x00004000	/* Channel priority error */
/* errored/cancelled channel */
#define DMAES_ERRCHN(ERRCH)	(((ERRCH) & 0x1F00) >> 8)
#define DMAES_SAE	0x00000080	/* Source address error */
#define DMAES_SOE	0x00000040	/* Source offset error */
#define DMAES_DAE	0x00000020	/* Destination address error */
#define DMAES_DOE	0x00000010	/* Destination offset error */
#define DMAES_NCE	0x00000008	/* Nbytes citer error */
#define DMAES_SGE	0x00000004	/* Scatter gather error */
#define DMAES_SBE	0x00000002	/* Source bus error */
#define DMAES_DBE	0x00000001	/* Destination bus error */

/* DMA Enable Request (DMAERQH, DMAERQL) Enable/disable device
	request for the channel */
#define DMA_SET_ENABLE_REQUEST(REGS, CH)	out_8(((REGS)->dmasreq), CH)
#define DMA_CLEAR_ENABLE_REQUEST(REGS, CH)	out_8(((REGS)->dmacerq), CH)

/* DMA Enable Error Interrupt (DMAEEIH, DMAEEIL) Enable/disable
	error interrupt for the channel */
#define DMA_SET_ENABLE_ERROR_INT(REGS, CH)	out_8(((REGS)->dmaseei), CH)
#define DMA_CLEAR_ENABLE_ERROR_INT(REGS, CH)	out_8(((REGS)->dmaceei), CH)

/*   Clear interrupt/error for the channel */
#define DMA_CLEAR_INTT_REQUEST(REGS, CH)	out_8(((REGS)->dmacint), CH)
#define DMA_CLEAR_ERROR(REGS, CH)	out_8(((REGS)->dmacerr), CH)

/* Clear done bit for the channel */
#define DMA_CLEAR_DONE_BIT(REGS, CH)	out_8(((REGS)->dmacdne), CH)
/* Set start bit for the channel */
#define DMA_SET_START_BIT(REGS, CH)	out_8(((REGS)->dmassrt), CH)

#define TDMTX_DMA_CH	0	/* TDM Tx uses DMA 0 HardWired */
#define TDMRX_DMA_CH	1	/* TDM Rx uses DMA 1 Hardwired */
#define TCD_SIZE 32		/* 32 byte TCD for channel */
#define TCD_BUFFER_SIZE 64	/* 64 byte buffer for TCD */

/* Source address modulo */
#define DMA_TCD1_SMOD(SMOD)   (((SMOD) & 0x1F) << 27)
/* Source data transfer size */
#define DMA_TCD1_SSIZE(SSIZE) (((SSIZE) & 0x7) << 24)

/* Destination address modulo */
#define DMA_TCD1_DMOD(DMOD)   (((DMOD) & 0x1F) << 19)
/* data transfer size  */
#define DMA_TCD1_DSIZE(DSIZE) (((DSIZE) & 0x7) << 16)

/* Source address signed offset */
#define DMA_TCD1_SOFF(SOFF)   ((SOFF) & 0xFFFF)

/* Enable link to another channel on minor iteration completion. */
#define DMA_TCD5_E_MINOR_LINK 0x80000000
/* Link to this channel. */
#define DMA_TCD5_LINK_CH(CH) (((CH) & 0x3F) << 25)
/* Current iteration count when linking disnabled */
#define DMA_TCD5_CITER_DISABLE_LINK(CITER) (((CITER) & 0x7FFF) << 16)
/* Current iteration count when linking enabled */
#define DMA_TCD5_CITER_ENABLE_LINK(CITER) (((CITER) & 0x00FF) << 16)
/*  Destination address signed offset */
#define DMA_TCD5_DOFF(DOFF) ((DOFF) & 0xFFFF)

/* Beginning iteration count when linking disnabled */
#define DMA_TCD7_BITER_DISABLE_LINK(CITER) (((CITER) & 0x7FFF) << 16)
/* Beginning iteration count when linking enabled */
#define DMA_TCD7_BITER_ENABLE_LINK(CITER) (((CITER) & 0x00FF) << 16)
#define DMA_TCD7_BWC(BW) (((BW)&0x3)<<14)	/* BandWidth Control. */
/* Link channel number */
#define DMA_TCD7_LINKCH(CH)   (((CH) & 0x1F) << 8)
#define DMA_TCD7_DONE		0x00000080	/* Channel done  */
#define DMA_TCD7_ACTIVE		0x00000040	/* Channel active */
#define DMA_TCD7_E_MAJOR_LINK	0x00000020	/* channel to channel linking */
#define DMA_TCD7_E_SG		0x00000010	/* Enable scatter gather */
#define DMA_TCD7_D_REQ		0x00000008	/* Disable request */
/* interrupt on half major counter */
#define DMA_TCD7_INT_HALF	0x00000004
#define DMA_TCD7_INT_MAJ	0x00000002	/* interrupt on major counter */
#define DMA_TCD7_START		0x00000001	/* Channel start */

#define SSIZE_08BITS		0x00	/* port 1 byte */
#define SSIZE_16BITS		0x01
#define SSIZE_32BITS		0x02
#define SSIZE_64BITS		0x03

/* TDM  Control Registers. */
struct tdm_regs {
	u32 gir;		/*  General Interface Register  */
	u32 rir;		/*  Receive Interface Register  */
	u32 tir;		/*  Transmit Interface Register */
	u32 rfp;		/*  Receive Frame Parameters    */
	u32 tfp;		/*  Transmit Frame Parameters   */
	u8 reserved0[0xC];
	u32 rcen[4];		/*  Recieve Channel Enabled     */
	u8 reserved1[0x10];
	u32 tcen[4];		/*  Transmit Channel Enabled    */
	u8 reservedd2[0x10];
	u32 tcma[4];		/*  Transmit Channel Mask       */
	u8 reservederved3[0x10];
	u32 rcr;		/*  Receiver Control Register           */
	u32 tcr;		/*  Transmitter Control Register        */
	u32 rier;		/*  Receive Interrupt Enable Register   */
	u32 tier;		/*  Transmit Interrupt Enable Register  */
	u8 reserved4[0x10];
	u32 rer;		/*  Receive Event Register              */
	u32 ter;		/*  Transmit Event Register             */
	u32 rsr;		/*  Receive Status Register             */
	u32 tsr;		/*  Transmit Status Register            */
};

struct tdm_data {
	u64 rdr;		/* Receive Data Register */
	u64 tdr;		/* Transmit Dataa Register */
};

struct tdm_clock {
	u32 rx;			/* Transmit Dataa Register */
	u32 tx;			/* Receive Data Register */
};

/* TDMGIR  General Interface Register */
#define GIR_LPBK	0x00000004	/* loopback mode */
#define GIR_CTS		0x00000002	/* Common TDM signals */
#define GIR_RTS		0x00000001	/* Rx & Tx sharing */

/* TDMRIR Recieve Interface Rgister */
#define RIR_RFWM_MASK	0x00000003	/* Recieve FIFO Watermark */
#define RIR_RFWM_SHIFT	16
#define RIR_RFWM(x)     ((x & RIR_RFWM_MASK) << RIR_RFWM_SHIFT)
#define RIR_RFEN	0x00008000	/* Recieve FIFO Enable */
#define RIR_RWEN	0x00004000	/* Recieve Wide FIFO Enable */
#define RIR_RDMA	0x00000040	/* Recieve DMA Enable */
#define RIR_RFSD_SHIFT	0x00000004	/* Recieve Frame Sync Delay */
#define RIR_RFSD_MASK	0x00000003
#define RIR_RFSD(x)	((x & RIR_RFSD_MASK) << RIR_RFSD_SHIFT)
#define RIR_RSO		0x00002000	/* Recieve sync Out */
#define RIR_RSL		0x00000800	/* Recieve sync Out Length */
#define RIR_RSOE	0x00000400	/* Recieve sync Out Edge */
#define RIR_RCOE	0x00000200	/* Recieve Clock Output Enable */
#define RIR_RSA		0x00000008	/* Recieve Sync Active */
#define RIR_RDE		0x00000004	/* Recieve Data Edge */
#define RIR_RFSE	0x00000002	/* Recieve Frame Sync Edge */
#define RIR_RRDO	0x00000001	/* Revieve Reversed Data Order */

/* TDMTIR  Transmit Interface Rgister */
#define TIR_TFWM_MASK	0x00000003	/* Transmit FIFO Watermark */
#define TIR_TFWM_SHIFT	16
#define TIR_TFWM(x)	((x & TIR_TFWM_MASK) << TIR_TFWM_SHIFT)
#define TIR_TFEN	0x00008000	/* Transmit FIFO Enable */
#define TIR_TWEN	0x00004000	/* Transmit Wide FIFO Enable */
#define TIR_TDMA	0x00000040	/* Transmit DMA Enable */
#define TIR_TFSD_SHIFT	0x00000004	/* Transmit Frame Sync Delay */
#define TIR_TFSD_MASK	0x00000003
#define TIR_TFSD(x)	((x & TIR_TFSD_MASK) << TIR_TFSD_SHIFT)
#define TIR_TSO		0x00002000	/* Transmit Sync Output */
#define TIR_TSL		0x00000800	/* Transmit sync Out Length */
#define TIR_TSOE	0x00000400	/* Transmit sync Out Edge */
#define TIR_TCOE	0x00000200	/* Transmit Clock Output Enable */
#define TIR_TSA		0x00000008	/* Transmit Sync Active */
#define TIR_TDE		0x00000004	/* Transmit Data Edge */
#define TIR_TFSE	0x00000002	/* Transmit Frame Sync Edge */
#define TIR_TRDO	0x00000001	/* Transmit Reversed Data Order */

/*TDMRFP  Revieve Frame Parameters */
#define RFP_RNCF_SHIFT	0x00000010	/* Number of Channels in TDM Frame */
#define RFP_RNCF_MASK	0x000000FF
#define RFP_RNCF(x)	(((x - 1) & RFP_RNCF_MASK) << RFP_RNCF_SHIFT)
#define RFP_RCS_SHIFT	0x00000004	/* Recieve Channel Size */
#define RFP_RCS_MASK	0x00000003
#define RFP_RCS(x)	((x & RFP_RCS_MASK) << RFP_RCS_SHIFT)
#define RFP_RT1		0x00000002	/* Recieve T1 Frame */

/*TDMTFP Transmit Frame Parameters */
#define TFP_TNCF_SHIFT	0x00000010	/* Number of Channels in TDM Frame */
#define TFP_TNCF_MASK	0x000000FF
#define TFP_TNCF(x)	(((x - 1) & TFP_TNCF_MASK) << TFP_TNCF_SHIFT)
#define TFP_TCS_SHIFT	0x00000004	/* Transmit Channel Size */
#define TFP_TCS_MASK	0x00000003
#define TFP_TCS(x)	((x & RFP_RCS_MASK) << RFP_RCS_SHIFT)
#define TFP_TT1		0x00000002	/* Transmit T1 Frame */


/* TDMRCR  Recieve Control Register */
#define RCR_REN		0x00000001	/* Recieve Enable */
/* TDMTCR  Transmit Control Register */
#define TCR_TEN		0x00000001	/* Transmit Enable */

/* TDMRIER receive interrupt enable register */
#define RIER_RCEUE	0x00000100	/* Channel Enable Update Enable */
#define RIER_RLCEE	0x00000080	/* Recieve Last Channel Event Enable */
#define RIER_RFSEE	0x00000040	/* Recieve Frame Sync Event Enable */
#define RIER_RFFEE	0x00000020	/* Recieve FIFO Full Event Enable */
#define RIER_RDREE	0x00000010	/* Recieve Data Ready Event Enable */
#define RIER_RSEEE	0x00000008	/* Recieve Sync Error Event Enable */
#define RIER_ROEE	0x00000004	/* Recieve Overrun Event Enable */

/* TDMTIER  transmit interrupt enable register */
#define TIER_TCEUE	0x00000100	/* Channel Enable Update Enable */
#define TIER_TLCEE	0x00000080	/* Transmit Last Channel Event */
#define TIER_TFSEE	0x00000040	/* Transmit Frame Sync Event Enable */
#define TIER_TFFEE	0x00000020	/* Transmit FIFO Full Event Enable */
#define TIER_TDREE	0x00000010	/* Transmit Data Ready Event Enable */
#define TIER_TSEEE	0x00000008	/* Transmit Sync Error Event Enable */
#define TIER_TUEE	0x00000004	/* Transmit Overrun Event Enable */

/* TDMRER  Recieve Event Register */
#define RER_RCEU	0x00000100	/* Recieve Channel Enable Update */
#define RER_RLCE	0x00000080	/* Recieve Last Channel Event */
#define RER_RFSE	0x00000040	/* Recieve Frame Sync Event */
#define RER_RFFE	0x00000020	/* Recieve FIFO Full Event */
#define RER_RDRE	0x00000010	/* Recieve Data Ready Event */
#define RER_RSEE	0x00000008	/* Recieve Sync Error Event */
#define RER_ROE		0x00000004	/* Recieve Overrun Event */

/* TDMTER  Transmit Event Register */
#define TER_TCEU	0x00000100	/* Transmit Channel Enable Update */
#define TER_TLCE	0x00000080	/* Transmit Last Channel Event */
#define TER_TFSE	0x00000040	/* Transmit Frame Sync Event */
#define TER_TFEE	0x00000020	/* Transmit FIFO Full Event */
#define TER_TDRE	0x00000010	/* Transmit Data Ready Event */
#define TER_TSEE	0x00000008	/* Transmit Sync Error Event */
#define TER_TUE		0x00000004	/* Transmit Overrun Event */

/* TDMRSR  Recieve Status Register */
#define RSR_RFCNT	0x00000038	/* Recieve FIFO counter */
#define RSSS_MASK	0x00000003	/* Recieve SYNC Status */
#define RSR_RSSS_SHIFT  1
#define RSR_RSSS(SSS)	(((SSS) >> (RSR_RSSS_SHIFT)) & (RSR_RSSS_MASK))
#define RSR_RENS	0x00000001	/* Recieve Enable Status */

/* TDMTSR  Transmit Status Register */
#define TSR_TFCNT	0x00000038	/* Transmit FIFO counter */
#define TSR_TSSS_MASK	0x00000003	/* Transmit SYNC Status */
#define TSR_TSSS_SHIFT	1
#define TSR_TSSS(SSS)	(((SSS) >> (TSR_TSSS_SHIFT)) & TSR_TSSS_MASK)
#define TSR_TENS	0x00000001	/* Transmit Enable Status */

/* channel width */
#define CHANNEL_SIZE_1BYTE	1	/* 1 byte channel 8-bit linear */
#define CHANNEL_SIZE_2BYTE	2	/* 2 bytes      */

/* channel parameters   */
#define TDM_ENABLE_TIMEOUT	1000	/* time out for TDM rx, tx enable */
#define NUM_OF_TDM_BUF		3	/* Number of tdm buffers for startlite
					   DMA */
#define ALIGNED_2_BYTES		0x02	/* 2-bytes alignment */
#define ALIGNED_4_BYTES		0x04	/* 4-bytes alignment */
#define ALIGNED_8_BYTES		0x08	/* 8-bytes alignment */
#define ALIGNED_16_BYTES	0x10	/* 16-bytes alignment */
#define ALIGNED_32_BYTES	0x20	/* 32-bytes alignment */
#define ALIGNED_64_BYTES	0x40	/* 64-bytes alignment */

/* Extend a given size to make it alignable */
static inline int ALIGNABLE_SIZE(u32 size, u32 alignment)
{
	return size + alignment - 1;
}

/* Align a given address */
static inline void *ALIGN_ADDRESS(void *address, u32 alignment)
{
	return (void *)(((u32) address + alignment - 1) & (~(alignment - 1)));
}

/* Size of the buffer */
static inline int TDM_1BUF_SIZE(u32 num_ch, u32 channel_size, u32 frame_size)
{
	return frame_size *
		ALIGN_SIZE(channel_size * num_ch, ALIGNED_8_BYTES);
}

/* Alignable size of the 3 buffers */
static inline int TDM_BUF_SIZE(u32 num_ch, u32 channel_size, u32 frame_size)
{
	return
	    ALIGNABLE_SIZE((TDM_1BUF_SIZE(num_ch, channel_size, frame_size) *
			    NUM_OF_TDM_BUF), ALIGNED_8_BYTES);
}


struct tdm_priv {
	struct tdm_regs __iomem *tdm_regs;
	struct tdm_data __iomem *data_regs;
	struct dmac_regs __iomem *dmac_regs;
	struct tdm_clock __iomem *clk_regs;
	u32 ptdm_base;
	u8 *tdm_input_data;
	u8 *tdm_output_data;
	dma_addr_t dma_input_paddr;	/* dma mapped buffer for TDM Rx */
	dma_addr_t dma_output_paddr;	/* dma mapped buffer for TDM Tx */
	void *dma_input_vaddr;
	void *dma_output_vaddr;
	u32 phase_rx;
	u32 phase_tx;
	struct tcd *dma_rx_tcd[NUM_OF_TDM_BUF];
	struct tcd *dma_tx_tcd[NUM_OF_TDM_BUF];
	dma_addr_t dma_rx_tcd_paddr;
	dma_addr_t dma_tx_tcd_paddr;
	void *dma_rx_tcd_vaddr;
	void *dma_tx_tcd_vaddr;
	u32 tdm_buffer_size;
	u32 tdm_err_intr;
	u32 dmac_err_intr;
	u32 dmac_done_intr;
	int tdm_active;
	struct device *device;
	spinlock_t tdmlock;
	struct tdm_adapter *adap;
};

#endif
