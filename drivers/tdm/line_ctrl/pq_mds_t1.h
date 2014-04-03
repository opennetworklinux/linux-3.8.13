/*
 * drivers/tdm/line/pq_mds_t1.h
 *
 * Copyright 2011-2012 Freescale Semiconductor, Inc.
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


#define MAX_NUM_OF_CHANNELS	8
#define PQ_MDS_8E1T1_BRD_REV	0x00
#define PQ_MDS_8E1T1_PLD_REV	0x00

#define PLD_PINSET_DIGIOEN	0x01	/* Normal operation */
#define PLD_PINSET_TXEN		0x02	/* drive TTIP and TRING with data */

#define PLD_CSR_RCSRC_1544KHZ	0x00	/* Drive REFCLKIO with the
					   1.544MHz clock */
#define PLD_CSR_MSRC_1544KHZ	0x00	/* Drive MCLK with the 1.544MHz clock */
#define PLD_CSR_MSRC_2048KHZ	0x01	/* Drive MCLK with the 2.048MHz clock */
#define PLD_CSR_RCSRC_2048KHZ	0x40	/* DDrive REFCLKIO with the
					   2.048MHz clock */

#define PLD_SYSCLK_RS_1544KHZ	0x00	/* Drive RSYSCLK with the
					   1.544MHz clock */
#define PLD_SYSCLK_TS_1544KHZ	0x00	/* Drive TSYSCLK with the
					   1.544MHz clock */
#define PLD_SYSCLK_TS_2048KHZ	0x01	/* Drive TSYSCLK with the
					   2.048MHz clock */
#define PLD_SYSCLK_RS_2048KHZ	0x40	/* Drive RSYSCLK with the
					   2.048MHz clock */

#define PLD_SYNCTSS_TRI_STATE	0x00	/* Not using transmit-side
					   elastic store */

#define PLD_TCSR1_1544KHZ	0x00	/* Drive TCLK1 - TCLK4 with
					   the 1.544MHz clock */
#define PLD_TCSR2_1544KHZ	0x00	/* Drive TCLK5 - TCLK8 with
					   the 1.544MHz clock */
#define PLD_TCSR1_2048KHZ	0x55	/* Drive TCLK1 - TCLK4 with
					   the 2.048MHz clock */
#define PLD_TCSR2_2048KHZ	0x55	/* Drive TCLK5 - TCLK8 with
					   the 2.048MHz clock */

#define PLD_GCR_LCC_TDMA	0x40	/* LM card connected to Host TDM
					   channel A */
#define PLD_GCR_LCE_LM		0x02	/* Drive Line Module Le71HR0826 with
					   the T1/E1 framer BPCLK signal */

#define DS26528_GTCCR_BPREFSEL_REFCLKIN	0xa0	/* REFCLKIO is an input */
#define DS26528_GTCCR_BFREQSEL_1544KHZ	0x08	/* Backplane reference clock
						   is 1.544MHz */
#define DS26528_GTCCR_FREQSEL_1544KHZ	0x04	/* The external master clock
						   is 1.544MHz or multiple
						   thereof */
#define DS26528_GTCCR_BFREQSEL_2048KHZ	0x00	/* Backplane reference clock
						   is 2.048MHz */
#define DS26528_GTCCR_FREQSEL_2048KHZ	0x00	/* The external master clock is
						   2.048MHz or multiple thereof
						*/

#define DS26528_GFCR_BPCLK_2048KHZ	0x00	/* Backplane Clock Select
						   2.048MHz */

#define DS26528_GTCR2_TSSYNCOUT	0x02	/* the TSSYNCIO is an 8kHz output
					   synchronous to the BPCLK */
#define DS26528_GTCR1_BWE	0x04	/* a port write to one of the octal
					   ports is mapped into all eight ports
					*/

#define DS26528_GFSRR_Reset	0xff	/* Reset framer and BERT */
#define DS26528_GFSRR_NORMAL	0x00	/* Normal operation */

#define DS26528_GLSRR_Reset	0xff	/* Reset LIU */
#define DS26528_GLSRR_NORMAL	0x00	/* Normal operation */

#define DS26528_RMMR_SFTRST	0x02	/* Level sensitive soft reset */
#define DS26528_RMMR_FRM_EN	0x80	/* Framer enabled—all features
					   active */
#define DS26528_RMMR_INIT_DONE	0x40	/* Initialization Done */
#define DS26528_RMMR_T1		0x00	/* Receiver T1 Mode Select */
#define DS26528_RMMR_E1		0x01	/* Receiver E1 Mode Select */

#define DS26528_E1TAF_DEFAULT	0x1b	/* Transmit Align Frame Register */
#define DS26528_E1TNAF_DEFAULT	0x40	/* Transmit Non-Align Frame Register */

#define DS26528_TMMR_SFTRST	0x02	/* Level sensitive soft reset */
#define DS26528_TMMR_FRM_EN	0x80	/* Framer enabled—all features
					   active */
#define DS26528_TMMR_INIT_DONE	0x40	/* Initialization Done */
#define DS26528_TMMR_T1		0x00	/* Transmit T1 Mode Select */
#define DS26528_TMMR_E1		0x01	/* Transmit E1 Mode Select */

#define DS26528_RCR1_T1_SYNCT	0x80	/* qualify 24 bits */
#define DS26528_RCR1_T1_RB8ZS	0x40	/* B8ZS enabled */
#define DS26528_RCR1_T1_SYNCC	0x08	/* cross couple Ft and Fs pattern */

#define DS26528_RCR1_E1_HDB3	0x40	/* Receive HDB3 enabled */
#define DS26528_RCR1_E1_CCS	0x20	/* Receive CCS signaling mode */

#define DS26528_RIOCR_1544KHZ	0x00	/* RSYSCLK Mode Select is 1.544MHz */
#define DS26528_RIOCR_2048KHZ	0x10	/* RSYSCLK Mode Select is 2.048MHz or
					   IBO enabled */
#define DS26528_RIOCR_RSIO_OUT	0x00	/* RSYNC is an output */

#define DS26528_RCR3_FLB	0x01	/* Framer Loopback enabled */

#define DS26528_TIOCR_1544KHZ	0x00	/* TSYSCLK is 1.544MHz */
#define DS26528_TIOCR_2048KHZ	0x10	/* TSYSCLK is 2.048/4.096/8.192MHz or
					   IBO enabled */
#define DS26528_TIOCR_TSIO_OUT	0x04	/* TSYNC is an output */

#define DS26528_TCR1_TB8ZS	0x04	/* Transmit B8ZS Enable */

#define DS26528_LTRCR_T1	0x02	/* configures the LIU for T1 */
#define DS26528_LTRCR_E1	0x00	/* configures the LIU for E1 */

#define DS26528_LTITSR_TLIS_75OHM	0x00	/* Transmit Load
						   Impedance 75? */
#define DS26528_LTITSR_LBOS_75OHM	0x00	/* Transmit Pulse
						   Shape 75? */
#define DS26528_LTITSR_TLIS_100OHM	0x10	/* Transmit Load
						   Impedance 100? */
#define DS26528_LTITSR_TLIS_0DB_CSU	0x00	/* DSX-1/0dB CSU,
						   0ft–133ft ABAM 100? */

#define DS26528_LRISMR_75OHM	0x00	/* Receive Impedance 75? */
#define DS26528_LRISMR_100OHM	0x10	/* Receive Impedance 100? */
#define DS26528_LRISMR_MAX	0x03	/* Receive Impedance 120? */

#define DS26528_LMCR_TE	0x01	/* TTIP/TRING outputs enabled */

/*Global register used to configure the Slic Slac in E1 and T1 mode*/
struct global_reg {
	/*Occupied addresses from 0F0 to 0FF*/
	u8 gtcr1;	/* 0x0f0 - Global Transceiver Control Reg 1 */
	u8 gfcr;	/* 0x0f1 - Global Framer Control Reg */
	u8 gtcr2;	/* 0x0f2 - Global Transceiver Control Reg 2 */
	u8 gtccr;	/* 0x0f3 - Global Transciever Clock Control Reg */
	u8 res1;	/* 0x0f4 - Reserved */
	u8 glsrr;	/* 0x0f5 - Global LIU Software Reset Reg */
	u8 gfsrr;	/* 0x0f6 - Global Framer and BERT Software Reset Reg */
	u8 res2;	/* 0x0f7 - Reserved */
	u8 idr;		/* 0x0f8 - Device Identification Reg */
	u8 gfisr;	/* 0x0f9 - Global Framers Interrupt Status Reg */
	u8 gbisr;	/* 0x0fa - Global BERT Interrupt Status Reg */
	u8 glisr;	/* 0x0fb - Global LIU Interrupt Status Reg */
	u8 gfimr;	/* 0x0fc - Global Framers Interrupt Mask Reg */
	u8 gbimr;	/* 0x0fd - Global BERT Interrupt Mask Reg */
	u8 glimr;	/* 0x0fe - Global LIU Interrupt Mask Reg */
	u8 res3;	/* 0x0ff - Reseved */
};

/*Registers that control the Receive Framer in E1 Mode and T1 Mode*/
struct rx_frame {
	/*Occupied addresses from 000 to 0EF*/
	u8 res1[16];	/* 0x000 - 0x00f Reserved */
	u8 rhc;		/* 0x010 - Receive HDLC Control Reg */
	u8 rhbse;	/* 0x011 - Receive HDLC Bit Suppress Reg */
	u8 rds0sel;	/* 0x012 - Receive Channel Monitor Select Reg */
	u8 rsigc;	/* 0x013 - Receive-Signaling Control Reg */
	u8 rsaimr;	/* 0x014 - Receive Control Register 2 (T1 Mode)
				   Receive Sa-Bit Interrupt Mask Register
				   (E1 Mode) */
	u8 t1rbocc;	/* 0x015 - Receive BOC Ccontrol Reg(T1 Mode Only) */
	u8 res2[10];	/* 0x016 - 0x01f Reserved */
	u8 ridr[32];	/* 0x020 - 0x03f Receive Idle Definition 32 Reg
				   For E1 framming */
	u8 rs[16];	/* 0x040 - 0x04f RxSignaling[16] For E1 framming*/
	u8 lcvcr1;	/* 0x050 - Line Code Violation Count Reg 1 */
	u8 lcvcr2;	/* 0x051 - Line Code Violation Count Reg 2 */
	u8 pcvcr1;	/* 0x052 - Path Code Violation Count Reg 1 */
	u8 pcvcr2;	/* 0x053 - Path Code Violation Count Reg 2 */
	u8 foscr1;	/* 0x054 - Frame Out of Sync Count Reg 1 */
	u8 foscr2;	/* 0x055 - Frame Out of Sync Count Reg 2 */
	u8 e1ebcr1;	/* 0x056 - E1-Bit Counter 1(E1 Mode Only) */
	u8 e1ebcr2;	/* 0x057 - E1-Bit Counter 2(E1 Mode Only) */
	u8 res3[8];	/* 0x058 - 0x05f Reserved */
	u8 rds0m;	/* 0x060 - Receive DS0 Monitor Reg */
	u8 e1rfrid;	/* 0x061 - Receive Firmware Revision ID Register
				   (E1 Mode Only)*/
	u8 e1rrts7;	/* 0x062 - Receive FDL Register (T1 Mode)
				   Receive Real-Time Status Register 7
				   (E1 Mode) */
	u8 t1rboc;	/* 0x063 - Receive BOC Register(T1 Mode) */
	u8 e1raf;	/* 0x064 - Receive SLC-96 Data Link Register 1(T1 Mode)
				   E1 Receive Align Frame Register (E1 Mode) */
	u8 e1rnaf;	/* 0x065 - Receive SLC-96 Data Link Register 2(T1 Mode)
				   E1 Receive Non-Align Frame Register
				   (E1 Mode) */
	u8 e1rsiaf;	/* 0x066 - Receive SLC-96 Data Link Register 3(T1 Mode)
				   E1 Received Si Bits of the Align
				   Frame Register (E1 Mode)*/
	u8 e1rsinaf;	/* 0x067 - Receive Si Bit of Non-Align Frame Reg
				   (E1 Mode) */
	u8 e1rra;	/* 0x068 - Receive Remote Alarm Reg (E1 Mode) */
	u8 e1rsa[6];	/* 0x069 - 0x06e E1 Receive Sa Bits Reg
				   (E1 Mode Only) */
	u8 sa6code;	/* 0x06f - Received Sa6 Codeword Reg */
	u8 res5[16];	/* 0x070 - 0x07f Reserved */
	u8 rmmr;	/* 0x080 - Receive Master Mode Reg */
	u8 rcr1;	/* 0x081 - Receive Control Reg 1 */
	u8 rcr2;	/* 0x082 - Receive Control Reg 2 */
	u8 rcr3;	/* 0x083 - Receive Control Reg 3 */
	u8 riocr;	/* 0x084 - Receive I/O Config Reg */
	u8 rescr;	/* 0x085 - Receive Elastic Store Control Reg */
	u8 ercnt;	/* 0x086 - Error-Counter Config Reg */
	u8 rhfc;	/* 0x087 - Receive HDLC FIFO Control Reg */
	u8 riboc;	/* 0x088 - Receive Interleave Bus Operation
				   Control Reg */
	u8 t1rscc;	/* 0x089 - In-Band Receive Spare Control Register
				   (T1 Mode Only)*/
	u8 rxpc;	/* 0x08a - Receive Expansion Port Control Reg */
	u8 rbpbs;	/* 0x08b - Receive BERT Port Bit Suppress Reg */
	u8 res6[4];	/* 0x08c - 0x08f Reserved */
	u8 rls[7];	/* 0x090 - 0x096 Receive Latched Status Reg */
	u8 res8;	/* 0x097 - Reserved */
	u8 rss[4];	/* 0x098 - 0x09b Receive-Signaling Status Reg */
	u8 t1rscd[2];	/* 0x09c - 0x09d Receive Spare Code Definition Reg
				   (T1 Mode Only) */
	u8 res9;		/* 0x09e - Reserved */
	u8 riir;		/* 0x09f - Receive Interrupt Information Reg */
	u8 rim[7];		/* 0x0a0 - 0x0a6 Receive Interrupt Mask[7] */
	u8 res10;		/* 0x0a7 - Reserved */
	u8 rscse[4];	/* 0x0a8 - 0x0ab Receive-Signaling Change of
				   State Enable[4] */
	u8 t1rupcd[2];	/* 0x0ac - 0x0ad Receive Up Code Definition[2]
				   (T1 mode only) */
	u8 t1rdncd[2];	/* 0x0ae - 0x0af Receive Down Code Definition[2]
				   (T1 mode only) */
	u8 rrts1;	/* 0x0b0 - Receive Real-Time Status Reg 1 */
	u8 res11;	/* 0x0b1 - Reserved */
	u8 rrts3;	/* 0x0b2 - Receive Real-Time Status Reg 3 */
	u8 res12;	/* 0x0b3 - Reserved */
	u8 rrts5;	/* 0x0b4 - Receive Real-Time Status Reg 5 */
	u8 rhpba;	/* 0x0b5 - Receive HDLC Packet Bytes Available Reg */
	u8 rhf;		/* 0x0b6 - Receive HDLC FIFO Reg */
	u8 res13[9];	/* 0x0b7 - 0x0bf Reserved */
	u8 rbcs[4];	/* 0x0c0 - 0x0c3 Receive Blank Channel Select[4] */
	u8 rcbr[4];	/* 0x0c4 - 0x0c7 Receive Channel Blocking[4] */
	u8 rsi[4];	/* 0x0c8 - 0x0cb Receive-Signalling Reinsertion[4]
				   Enable Reg */
	u8 rgccs[4];	/* 0x0cc - 0x0cf Receive Gapped Clock Channel
				   Select[4] Reg */
	u8 rcice[4];	/* 0x0d0 - 0x0d3 Receive Channel Idle Code
				   Enable[4] Reg */
	u8 rbpcs[4];	/* 0x0d4 - 0x0d7 Receive BERT Port Channel
				   Select[4] Reg */
	u8 res14[24];	/* 0x0d8 - 0x0ef Reserved */
};

struct tx_frame {
    /*Occupied addresses from 100 to 1FF */
	u8 res1[16];	/* 0x100 - 0x10f */
	u8 thc1;	/* 0x110 - Transmit HDLC Control Reg 1 */
	u8 thbse;	/* 0x111 - Transmit HDLC Bit Suppress Reg */
	u8 res2;	/* 0x112 - Reserved */
	u8 thc2;	/* 0x113 - Transmit HDLC Control Reg 2 */
	u8 e1tsacr;	/* 0x114 - E1 Transmit Sa-Bit Control Reg (E1 Mode) */
	u8 res3[3];	/* 0x115 - 0x117 Reserved */
	u8 ssie[4];	/* 0x118 - 0x11b E1 Transmit Sa-Bit Control Reg
				   (E1 Mode) */
	u8 res4[4];	/* 0x11c - 0x11f Reserved */
	u8 tidr[32];	/* 0x120 - 0x13f Transmit Idle Code Definition */
	u8 ts[16];	/* 0x140 - 0x14f Transmit-Signaling Reg */
	u8 tcice[4];	/* 0x150 - 0x153 Transmit Channel Idle Code
				   Enable Reg */
	u8 res5[13];	/* 0x154 - 0x160 Reserved */
	u8 tfrid;	/* 0x161 - Transmit Firmware Revision ID Register*/
	u8 tfdl;	/* 0x162 - Transmit FDL Register (T1 Mode Only) */
	u8 tboc;	/* 0x163 - Transmit BOC Register (T1 Mode Only) */
	u8 e1taf;	/* 0x164 - Transmit SLC-96 Data Link Register 1
				   (T1 Mode)
				   Transmit Align Frame Register (E1 Mode) */
	u8 e1tnaf;	/* 0x165 - Transmit SLC-96 Data Link Register 2
				   (T1 Mode)
				   Transmit Non-Align Frame Register(E1 Mode)*/
	u8 e1tsiaf;	/* 0x166 - Transmit SLC-96 Data Link Register 3(T1 Mode)
				   Transmit Si Bits of the Align Frame Register
				   (E1 Mode) */
	u8 e1tsinaf;	/* 0x167 - Transmit Si Bits of the Non-Align
				   Frame Register (E1 Mode Only) */
	u8 e1tra;	/* 0x168 - Transmit Remote Alarm Register (E1 Mode) */
	u8 e1tsa[5];	/* 0x169 - 0x16d Transmit Sa Bits Register
				   (E1 Mode Only)*/
	u8 res6[18];	/* 0x16e - 0x17f Reserved */
	u8 tmmr;	/* 0x180 - Transmit Master Mode Register */
	u8 tcr[3];	/* 0x181 - 0x183 Transmit Control Register */
	u8 tiocr;	/* 0x184 - Transmit I/O Configuration Register */
	u8 tescr;	/* 0x185 - Transmit Elastic Store Control Register */
	u8 tcr4;	/* 0x186 - Transmit Control Register 4 (T1 Mode Only)*/
	u8 thfc;	/* 0x187 - Transmit HDLC FIFO Control Register */
	u8 tiboc;	/* 0x188 - Transmit Interleave Bus Operation
				   Control Register */
	u8 tds0sel;	/* 0x189 - Transmit DS0 Channel Monitor Select
				   Register */
	u8 txpc;	/* 0x18a - Transmit Expansion Port Control Register */
	u8 tbpbs;	/* 0x18b - Transmit BERT Port Bit Suppress Register */
	u8 res7[2];	/* 0x18c - 0x18d Reserved */
	u8 tsyncc;	/* 0x18e - Transmit Synchronizer Control Register */
	u8 res8;	/* 0x18f - Reserved */
	u8 tls[3];	/* 0x190 - 0x192 Transmit Latched Status Register*/
	u8 res9[12];	/* 0x193 - 0x19e Reserved*/
	u8 tiir;	/* 0x19f - Transmit Interrupt Information Register*/
	u8 tim[3];	/* 0x1a0 - 0x1a2 Transmit Interrupt Mask Register */
	u8 res10[9];	/* 0x1a3 - 0x1ab Reserved */
	u8 titcd[2];	/* 0x1ac - 0x1ad Transmit Code Definition Register */
	u8 res11[3];	/* 0x1ae - 0x1b0 Reserved */
	u8 trts2;	/* 0x1b1 - Transmit Real-Time Status Register 2(HDLC)*/
	u8 res12;	/* 0x1b2 - Reserved */
	u8 tfba;	/* 0x1b3 - Transmit HDLC FIFO Buffer Available */
	u8 thf;		/* 0x1b4 - Transmit HDLC FIFO Register */
	u8 res13[6];	/* 0x1b5 - 0x1ba Reserved */
	u8 tds0m;	/* 0x1bb - Transmit DS0 Monitor Register */
	u8 res14[4];	/* 0x1bc - 0x1bf Reserved */
	u8 tbcs[4];	/* 0x1c0 - 0x1c3 Transmit Blank Channel
				   Select Register */
	u8 tcbr[4];	/* 0x1c4 - 0x1c7 Transmit Channel Blocking Register */
	u8 thscs[4];	/* 0x1c8 - 0x1cb Transmit Hardware-Signaling Channel
				   Select Register */
	u8 tgccs[4];	/* 0x1cc - 0x1cf Transmit Gapped-Clock Channel
				   Select Register */
	u8 pcl[4];	/* 0x1d0 - 0x1d3 Per-Channel Loopback Enable Register */
	u8 tbpcs[4];	/* 0x1d4 - 0x1d7 Transmit BERT Port Channel
				   Select Register */
	u8 res15[40];	/* 0x1d8 - 0x1ff Reserved */
};

struct liu_reg {
    /*Occupied addresses from 1000 to 101F*/
	u8 ltrcr;	/* 0x1000 - LIU Transmit Receive Control Register */
	u8 ltitsr;	/* 0x1001 - LIU Transmit Impedance and Pulse Shape
				    Selection Register */
	u8 lmcr;	/* 0x1002 - LIU Maintenance Control Register */
	u8 lrsr;	/* 0x1003 - LIU Real Status Register */
	u8 lsimr;	/* 0x1004 - LIU Status Interrupt Mask Register */
	u8 llsr;	/* 0x1005 - LIU Latched Status Register */
	u8 lrsl;	/* 0x1006 - LIU Receive Signal Level Register */
	u8 lrismr;	/* 0x1007 - LIU Receive Impedance and Sensitivity
				    Monitor Register */
	u8  res1[24];	/* 0x1008 - 0x101f Reserved */
};

struct bert_reg {
    /*Occupied addresses from 1100 to 110F*/
	u8 bawc;	/* 0x1100 - BERT Alternating Word Count Rate Register*/
	u8 brp[4];	/* 0x1101 - 0x1104 BERT Repetitive Pattern
				    Set Register */
	u8 bc[2];	/* 0x1105 - 0x1106 BERT Control Register */
	u8 bbc[4];	/* 0x1107 - 0x110a BERT Bit Count Register */
	u8 bec[3];	/* 0x110b - 0x110d BERT Error Count Register */
	u8 blsr;	/* 0x110e - BERT Latched Status Register */
	u8 bsim;	/* 0x110f - BERT Status Interrupt Mask Register */
};

struct link_frame {
	struct rx_frame rx;	/*Receive Framer Settings*/
	struct global_reg gbl;
	struct tx_frame tx;	/*Transmit Framer Registers*/
};

/* Definition of strucute of the framers */
struct ds26528_mem {
	struct link_frame link[MAX_NUM_OF_CHANNELS];
	struct liu_reg liu[MAX_NUM_OF_CHANNELS];
	struct bert_reg bert[MAX_NUM_OF_CHANNELS];
};

struct pld_mem {
	u8 brdid;
	u8 brdrev;
	u8 pldrev;
	u8 pinset;
	u8 csr;
	u8 sysclk_tr;
	u8 synctss;
	u8 tcsr1;
	u8 tcsr2;
	u8 tsyncs1;
	u8 ds3set;
	u8 gcr;
	u8 rsv[4];
};

enum line_rate_t {
	LINE_RATE_T1,	/* T1 line rate (1.544 Mbps)      */
	LINE_RATE_E1	/* E1 line rate (2.048 Mbps)     */
};

enum tdm_trans_mode_t {
	NORMAL = 0,
	FRAMER_LB
};

enum card_support_type {
	LM_CARD = 0,
	DS26528_CARD,
	NO_CARD
};

struct pq_mds_t1 {
	int irq;
	struct device *dev;
	struct ds26528_mem *ds26528_base;
	struct pld_mem *pld_base;
	enum tdm_trans_mode_t trans_mode;
	enum line_rate_t line_rate;
	enum card_support_type card_support;
};
