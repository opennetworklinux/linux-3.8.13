/*
 * drivers/dma/fsl_raid.h
 *
 * Freescale RAID Engine device driver
 *
 * Author:
 *	Harninder Rai <harninder.rai@freescale.com>
 *	Naveen Burmi <naveenburmi@freescale.com>
 *
 * Copyright (c) 2010-2012 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#define RE_DPAA_MODE		(1 << 30)
#define RE_NON_DPAA_MODE	(1 << 31)
#define RE_GFM_POLY		0x1d000000
#define RE_JR_INB_JOB_ADD(x)	((x) << 16)
#define RE_JR_OUB_JOB_REMOVE(x)	((x) << 16)
#define RE_JR_CFG1_CBSI		0x08000000
#define RE_JR_CFG1_CBS0		0x00080000
#define RE_JR_OUB_SLOT_FULL_SHIFT	8
#define RE_JR_OUB_SLOT_FULL(x)	((x) >> RE_JR_OUB_SLOT_FULL_SHIFT)
#define RE_JR_INB_SLOT_AVAIL_SHIFT	8
#define RE_JR_INB_SLOT_AVAIL(x)	((x) >> RE_JR_INB_SLOT_AVAIL_SHIFT)
#define RE_PQ_OPCODE		0x1B
#define RE_XOR_OPCODE		0x1A
#define RE_MOVE_OPCODE		0x8
#define FRAME_DESC_ALIGNMENT	16
#define RE_BLOCK_SIZE		0x3 /* 4096 bytes */
#define CACHEABLE_INPUT_OUTPUT	0x0
#define BUFFERABLE_OUTPUT	0x0
#define INTERRUPT_ON_ERROR	0x1
#define DATA_DEPENDENCY		0x1
#define ENABLE_DPI		0x0
#define RING_SIZE		0x1000
#define RING_SIZE_MASK		(RING_SIZE - 1)
#define RING_SIZE_SHIFT		8
#define RE_JR_ADDRESS_BIT_SHIFT	4
#define RE_JR_ADDRESS_BIT_MASK	((1 << RE_JR_ADDRESS_BIT_SHIFT) - 1)
#define RE_JR_ERROR		0x40000000
#define RE_JR_INTERRUPT		0x80000000
#define RE_JR_CLEAR_INT		0x80000000
#define RE_JR_PAUSE		0x80000000
#define RE_JR_ENABLE		0x80000000

#define RE_JR_REG_LIODN_MASK	0x00000fff
#define RE_CF_CDB_ALIGN		64

/*
 * the largest cf block is 19*sizeof(struct cmpnd_frame), which is 304 bytes.
 * here 19 = 1(cdb)+2(dest)+16(src), align to 64bytes, that is 320 bytes.
 * the largest cdb block: struct pq_cdb which is 180 bytes, adding to cf block
 * 320+180=500, align to 64bytes, that is 512 bytes.
 */
#define RE_CF_DESC_SIZE		320
#define RE_CF_CDB_SIZE		512

struct re_ctrl {
	/* General Configuration Registers */
	__be32 global_config;	/* Global Configuration Register */
	u8     rsvd1[4];
	__be32 galois_field_config; /* Galois Field Configuration Register */
	u8     rsvd2[4];
	__be32 jq_wrr_config;   /* WRR Configuration register */
	u8     rsvd3[4];
	__be32 crc_config;	/* CRC Configuration register */
	u8     rsvd4[228];
	__be32 system_reset;	/* System Reset Register */
	u8     rsvd5[252];
	__be32 global_status;	/* Global Status Register */
	u8     rsvd6[832];
	__be32 re_liodn_base;	/* LIODN Base Register */
	u8     rsvd7[1712];
	__be32 re_version_id;	/* Version ID register of RE */
	__be32 re_version_id_2; /* Version ID 2 register of RE */
	u8     rsvd8[512];
	__be32 host_config;	/* Host I/F Configuration Register */
};

struct jr_config_regs {
	/* Registers for JR interface */
	__be32 jr_config_0;	/* Job Queue Configuration 0 Register */
	__be32 jr_config_1;	/* Job Queue Configuration 1 Register */
	__be32 jr_interrupt_status; /* Job Queue Interrupt Status Register */
	u8     rsvd1[4];
	__be32 jr_command;	/* Job Queue Command Register */
	u8     rsvd2[4];
	__be32 jr_status;	/* Job Queue Status Register */
	u8     rsvd3[228];

	/* Input Ring */
	__be32 inbring_base_h;	/* Inbound Ring Base Address Register - High */
	__be32 inbring_base_l;	/* Inbound Ring Base Address Register - Low */
	__be32 inbring_size;	/* Inbound Ring Size Register */
	u8     rsvd4[4];
	__be32 inbring_slot_avail; /* Inbound Ring Slot Available Register */
	u8     rsvd5[4];
	__be32 inbring_add_job;	/* Inbound Ring Add Job Register */
	u8     rsvd6[4];
	__be32 inbring_cnsmr_indx; /* Inbound Ring Consumer Index Register */
	u8     rsvd7[220];

	/* Output Ring */
	__be32 oubring_base_h;	/* Outbound Ring Base Address Register - High */
	__be32 oubring_base_l;	/* Outbound Ring Base Address Register - Low */
	__be32 oubring_size;	/* Outbound Ring Size Register */
	u8     rsvd8[4];
	__be32 oubring_job_rmvd; /* Outbound Ring Job Removed Register */
	u8     rsvd9[4];
	__be32 oubring_slot_full; /* Outbound Ring Slot Full Register */
	u8     rsvd10[4];
	__be32 oubring_prdcr_indx; /* Outbound Ring Producer Index */
};

/*
 * Command Descriptor Block (CDB) for unicast move command.
 * In RAID Engine terms, memcpy is done through move command
 */
struct move_cdb {
	u32 opcode:5;
	u32 rsvd1:11;
	u32 blk_size:2;
	u32 cache_attrib:2;
	u32 buffer_attrib:1;
	u32 error_attrib:1;
	u32 rsvd2:6;
	u32 data_depend:1;
	u32 dpi:1;
	u32 rsvd3:2;
} __packed;

/* Data protection/integrity related fields */
struct dpi_related {
	u32 apps_mthd:2;
	u32 ref_mthd:2;
	u32 guard_mthd:2;
	u32 dpi_attr:2;
	u32 rsvd1:8;
	u32 meta_tag:16;
	u32 ref_tag:32;
} __packed;

/*
 * CDB for GenQ command. In RAID Engine terminology, XOR is
 * done through this command
 */
struct xor_cdb {
	u32 opcode:5;
	u32 rsvd1:11;
	u32 blk_size:2;
	u32 cache_attrib:2;
	u32 buffer_attrib:1;
	u32 error_attrib:1;
	u32 nrcs:4;
	u32 rsvd2:2;
	u32 data_depend:1;
	u32 dpi:1;
	u32 rsvd3:2;
	u8 gfm[16];
	struct dpi_related dpi_dest_spec;
	struct dpi_related dpi_src_spec[16];
} __packed;

/* CDB for no-op command */
struct noop_cdb {
	u32 opcode:5;
	u32 rsvd1:23;
	u32 dependency:1;
	u32 rsvd2:3;
} __packed;

/*
 * CDB for GenQQ command. In RAID Engine terminology, P/Q is
 * done through this command
 */
struct pq_cdb {
	u32 opcode:5;
	u32 rsvd1:1;
	u32 excl_enable:2;
	u32 excl_q1:4;
	u32 excl_q2:4;
	u32 blk_size:2;
	u32 cache_attrib:2;
	u32 buffer_attrib:1;
	u32 error_attrib:1;
	u32 nrcs:4;
	u32 rsvd2:2;
	u32 data_depend:1;
	u32 dpi:1;
	u32 rsvd3:2;

	u8 gfm_q1[16];
	u8 gfm_q2[16];
	struct dpi_related dpi_dest_spec[2];
	struct dpi_related dpi_src_spec[16];
} __packed;

/* Compound frame */
struct cmpnd_frame {
	u64 rsvd1:24;
	u64 address:40;
	u32 extension:1;
	u32 final:1;
	u32 rsvd3:10;
	u32 length:20;
	u32 rsvd4:8;
	u32 bpid:8;
	u32 rsvd5:3;
	u32 offset:13;

} __packed;

/* Frame descriptor */
struct jr_hw_desc {
	u64 debug:2;
	u64 liodn_off:6;
	u64 bpid:8;
	u64 eliodn_off:4;
	u64 rsvd1:4;
	u64 address:40;
	u64 format:3;
	u64 rsvd2:29;
	u64 status:32;
} __packed;

#define MAX_RE_JRS		4

/* Raid Engine device private data */
struct re_drv_private {
	u8 total_jrs;
	struct dma_device dma_dev;
	struct re_ctrl *re_regs;
	struct re_jr *re_jrs[MAX_RE_JRS];
	struct dma_pool *desc_pool;
	struct dma_pool *hw_desc_pool;
};

/* Per job ring data structure */
struct re_jr {
	dma_cookie_t completed_cookie;
	spinlock_t desc_lock; /* jr data access lock */
	struct list_head ack_q;
	struct device *dev;
	struct re_drv_private *re_dev;
	struct dma_chan chan;
	struct jr_config_regs *jrregs;
	int irq;
	struct tasklet_struct irqtask;

	/* hw descriptor ring for inbound queue*/
	dma_addr_t inb_phys_addr;
	struct jr_hw_desc *inb_ring_virt_addr;
	u32 inb_count;
	u32 pend_count;

	/* hw descriptor ring for outbound queue */
	dma_addr_t oub_phys_addr;
	struct jr_hw_desc *oub_ring_virt_addr;
	u32 oub_count;

	struct fsl_re_dma_async_tx_desc *descs; /* sw descriptor ring */
	void *cfs;				/* dma descriptor ring */
	dma_addr_t phys;          /* phys addr for dma descriptor ring */

	struct timer_list timer;
};

enum desc_state {
	RE_DESC_EMPTY,
	RE_DESC_ALLOC,
};

/* Async transaction descriptor */
struct fsl_re_dma_async_tx_desc {
	struct dma_async_tx_descriptor async_tx;
	struct list_head node;
	struct list_head tx_list;
	struct jr_hw_desc *hwdesc;
	struct re_jr *jr;

	void *cf_addr;
	int dma_len;
	u8 dest_cnt;
	u8 src_cnt;

	u16 cdb_opcode;
	void *cdb_addr;
	dma_addr_t cdb_paddr;
	int cdb_len;

	enum desc_state state;
};
