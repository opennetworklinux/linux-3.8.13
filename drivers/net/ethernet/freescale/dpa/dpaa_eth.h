/*
 * Copyright 2008-2012 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
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
 */

#ifndef __DPA_H
#define __DPA_H

#include <linux/ethtool.h>	/* struct ethtool_ops */
#include <linux/netdevice.h>
#include <linux/list.h>		/* struct list_head */
#include <linux/workqueue.h>	/* struct work_struct */
#include <linux/skbuff.h>
#include <linux/hardirq.h>
#include <linux/if_vlan.h>	/* vlan_eth_hdr */
#include <linux/ip.h>		/* ip_hdr */
#include <linux/ipv6.h>		/* ipv6_hdr */

#include <linux/fsl_qman.h>	/* struct qman_fq */

#include "lnxwrp_fsl_fman.h"
#include "fm_ext.h"
#include "fm_port_ext.h" /* FM_PORT_FRM_ERR_* */

#include "mac.h"		/* struct mac_device */
#ifdef CONFIG_FSL_DPAA_ETH_DEBUGFS
#include "dpaa_debugfs.h"
#endif /* CONFIG_FSL_DPAA_ETH_DEBUGFS */
#include "dpaa_eth_trace.h"

extern int dpa_rx_extra_headroom;
extern int dpa_max_frm;

#define dpa_get_rx_extra_headroom() dpa_rx_extra_headroom
#define dpa_get_max_frm() dpa_max_frm

/* Currently we have the same max_frm on all interfaces, so these macros
 * don't get a net_device argument. This will change in the future.
 */
#define dpa_get_min_mtu()	64
#define dpa_get_max_mtu()	\
	(dpa_get_max_frm() - (VLAN_ETH_HLEN + ETH_FCS_LEN))

#define __hot

/* Simple enum of FQ types - used for array indexing */
enum port_type {RX, TX};

/* TODO: This structure should be renamed & moved to the FMD wrapper */
struct dpa_buffer_layout_s {
	uint16_t	priv_data_size;
	bool		parse_results;
	bool		time_stamp;
	bool		hash_results;
	uint8_t		manip_extra_space;
	uint16_t	data_align;
};

#ifdef CONFIG_FSL_DPAA_ETH_DEBUG
#define DPA_BUG_ON(cond)	BUG_ON(cond)
#else
#define DPA_BUG_ON(cond)
#endif

#define DPA_TX_PRIV_DATA_SIZE	16
#define DPA_PARSE_RESULTS_SIZE sizeof(fm_prs_result_t)
#define DPA_TIME_STAMP_SIZE 8
#define DPA_HASH_RESULTS_SIZE 8
#define DPA_RX_PRIV_DATA_SIZE   (DPA_TX_PRIV_DATA_SIZE + \
					dpa_get_rx_extra_headroom())

#define DPA_SGT_MAX_ENTRIES 16 /* maximum number of entries in SG Table */


#define FM_FD_STAT_RX_ERRORS						\
	(FM_PORT_FRM_ERR_DMA | FM_PORT_FRM_ERR_PHYSICAL	| \
	 FM_PORT_FRM_ERR_SIZE | FM_PORT_FRM_ERR_CLS_DISCARD | \
	 FM_PORT_FRM_ERR_EXTRACTION | FM_PORT_FRM_ERR_NO_SCHEME	| \
	 FM_PORT_FRM_ERR_ILL_PLCR | FM_PORT_FRM_ERR_PRS_TIMEOUT	| \
	 FM_PORT_FRM_ERR_PRS_ILL_INSTRUCT | FM_PORT_FRM_ERR_PRS_HDR_ERR)

#define FM_FD_STAT_TX_ERRORS \
	(FM_PORT_FRM_ERR_UNSUPPORTED_FORMAT | \
	 FM_PORT_FRM_ERR_LENGTH | FM_PORT_FRM_ERR_DMA)

#ifndef CONFIG_FSL_DPAA_ETH_JUMBO_FRAME
/* The raw buffer size must be cacheline aligned.
 * Normally we use 2K buffers.
 */
#define DPA_BP_RAW_SIZE		2048
#else
/* For jumbo frame optimizations, use buffers large enough to accomodate
 * 9.6K frames, FD maximum offset, skb sh_info overhead and some extra
 * space to account for further alignments.
 */
#define DPA_MAX_FRM_SIZE	9600
#define DPA_BP_RAW_SIZE \
	((DPA_MAX_FRM_SIZE + DPA_MAX_FD_OFFSET + \
	  sizeof(struct skb_shared_info) + 128) & ~(SMP_CACHE_BYTES - 1))
#endif

/* This is what FMan is ever allowed to use.
 * FMan-DMA requires 16-byte alignment for Rx buffers, but SKB_DATA_ALIGN is
 * even stronger (SMP_CACHE_BYTES-aligned), so we just get away with that,
 * via SKB_WITH_OVERHEAD(). We can't rely on netdev_alloc_frag() giving us
 * half-page-aligned buffers (can we?), so we reserve some more space
 * for start-of-buffer alignment.
 */
#define dpa_bp_size(buffer_layout)	(SKB_WITH_OVERHEAD(DPA_BP_RAW_SIZE) - \
						SMP_CACHE_BYTES)
/* We must ensure that skb_shinfo is always cacheline-aligned. */
#define DPA_SKB_SIZE(size)	((size) & ~(SMP_CACHE_BYTES - 1))

/* Maximum size of a buffer for which recycling is allowed.
 * We need an upper limit such that forwarded skbs that get reallocated on Tx
 * aren't allowed to grow unboundedly. On the other hand, we need to make sure
 * that skbs allocated by us will not fail to be recycled due to their size.
 *
 * For a requested size, the kernel allocator provides the next power of two
 * sized block, which the stack will use as is, regardless of the actual size
 * it required; since we must acommodate at most 9.6K buffers (L2 maximum
 * supported frame size), set the recycling upper limit to 16K.
 */
#define DPA_RECYCLE_MAX_SIZE	16384

#if defined(CONFIG_FSL_DPAA_FMAN_UNIT_TESTS)
/*TODO: temporary for fman pcd testing */
#define FMAN_PCD_TESTS_MAX_NUM_RANGES	20
#endif

#define DPAA_ETH_PCD_FQ_BASE(device_addr) \
	(((device_addr) & 0x1fffff) >> 6)

/* Largest value that the FQD's OAL field can hold.
 * This is DPAA-1.x specific.
 * TODO: This rather belongs in fsl_qman.h
 */
#define FSL_QMAN_MAX_OAL	127

/* Maximum offset value for a contig or sg FD (represented on 9 bits) */
#define DPA_MAX_FD_OFFSET	((1 << 9) - 1)

/* Default alignment for start of data in an Rx FD */
#define DPA_FD_DATA_ALIGNMENT  16

/* Values for the L3R field of the FM Parse Results
 */
/* L3 Type field: First IP Present IPv4 */
#define FM_L3_PARSE_RESULT_IPV4	0x8000
/* L3 Type field: First IP Present IPv6 */
#define FM_L3_PARSE_RESULT_IPV6	0x4000

/* Values for the L4R field of the FM Parse Results
 * See $8.8.4.7.20 - L4 HXS - L4 Results from DPAA-Rev2 Reference Manual.
 */
/* L4 Type field: UDP */
#define FM_L4_PARSE_RESULT_UDP	0x40
/* L4 Type field: TCP */
#define FM_L4_PARSE_RESULT_TCP	0x20
/* This includes L4 checksum errors, but also other errors that the Hard Parser
 * can detect, such as invalid combinations of TCP control flags, or bad UDP
 * lengths.
 */
#define FM_L4_PARSE_ERROR	0x10
/* Check if the hardware parser has run */
#define FM_L4_HXS_RUN		0xE0

/* FD status field indicating whether the FM Parser has attempted to validate
 * the L4 csum of the frame.
 * Note that having this bit set doesn't necessarily imply that the checksum
 * is valid. One would have to check the parse results to find that out.
 */
#define FM_FD_STAT_L4CV		0x00000004


#define FM_FD_STAT_ERR_PHYSICAL	FM_PORT_FRM_ERR_PHYSICAL

/* Check if the FMan Hardware Parser has run for L4 protocols.
 *
 * @parse_result_ptr must be of type (fm_prs_result_t *).
 */
#define fm_l4_hxs_has_run(parse_result_ptr) \
	((parse_result_ptr)->l4r & FM_L4_HXS_RUN)
/* Iff the FMan Hardware Parser has run for L4 protocols, check error status.
 *
 * @parse_result_ptr must be of type (fm_prs_result_t *).
 */
#define fm_l4_hxs_error(parse_result_ptr) \
	((parse_result_ptr)->l4r & FM_L4_PARSE_ERROR)
/* Check if the parsed frame was found to be a TCP segment.
 *
 * @parse_result_ptr must be of type (fm_prs_result_t *).
 */
#define fm_l4_frame_is_tcp(parse_result_ptr) \
	((parse_result_ptr)->l4r & FM_L4_PARSE_RESULT_TCP)

/* number of Tx queues to FMan */
#define DPAA_ETH_TX_QUEUES	NR_CPUS
#define DPAA_ETH_RX_QUEUES	128

struct pcd_range {
	uint32_t			 base;
	uint32_t			 count;
};

/* More detailed FQ types - used for fine-grained WQ assignments */
enum dpa_fq_type {
	FQ_TYPE_RX_DEFAULT = 1, /* Rx Default FQs */
	FQ_TYPE_RX_ERROR,       /* Rx Error FQs */
	FQ_TYPE_RX_PCD,         /* User-defined PCDs */
	FQ_TYPE_TX,             /* "Real" Tx FQs */
	FQ_TYPE_TX_CONFIRM,     /* Tx default Conf FQ (actually an Rx FQ) */
	FQ_TYPE_TX_CONF_MQ,     /* Tx conf FQs (one for each Tx FQ) */
	FQ_TYPE_TX_ERROR,       /* Tx Error FQs (these are actually Rx FQs) */
#ifdef CONFIG_FSL_DPAA_TX_RECYCLE
	FQ_TYPE_TX_RECYCLE,	/* Tx FQs for recycleable frames only */
#endif
};

struct dpa_fq {
	struct qman_fq		 fq_base;
	struct list_head	 list;
	struct net_device	*net_dev;
	bool			 init;
	uint32_t fqid;
	uint32_t flags;
	uint16_t channel;
	uint8_t wq;
	enum dpa_fq_type fq_type;
};

typedef struct dpa_fq_cbs_t {
	struct qman_fq rx_defq;
	struct qman_fq tx_defq;
	struct qman_fq rx_errq;
	struct qman_fq tx_errq;
	struct qman_fq egress_ern;
} dpa_fq_cbs_t;

struct fqid_cell {
	uint32_t start;
	uint32_t count;
};

struct dpa_bp {
	struct bman_pool		*pool;
	uint8_t				bpid;
	struct device			*dev;
	union {
		/* The buffer pools used for the private ports are initialized
		 * with target_count buffers for each CPU; at runtime the
		 * number of buffers per CPU is constantly brought back to this
		 * level
		 */
		int target_count;
		/* The configured value for the number of buffers in the pool,
		 * used for shared port buffer pools
		 */
		int config_count;
	};
	size_t				size;
	bool				seed_pool;
	/* physical address of the contiguous memory used by the pool to store
	 * the buffers
	 */
	dma_addr_t			paddr;
	/* virtual address of the contiguous memory used by the pool to store
	 * the buffers
	 */
	void				*vaddr;
	/* current number of buffers in the bpool alloted to this CPU */
	int *percpu_count;
	atomic_t refs;
	/* some bpools need to be seeded before use by this cb */
	int (*seed_cb)(struct dpa_bp *);
	/* some bpools need to be emptied before freeing; this cb is used
	 * for freeing of individual buffers taken from the pool
	 */
	void (*free_buf_cb)(void *addr);
};

struct dpa_rx_errors {
	u64 dme;		/* DMA Error */
	u64 fpe;		/* Frame Physical Error */
	u64 fse;		/* Frame Size Error */
	u64 phe;		/* Header Error */
	u64 cse;		/* Checksum Validation Error */
};

/* Counters for QMan ERN frames - one counter per rejection code */
struct dpa_ern_cnt {
	u64 cg_tdrop;		/* Congestion group taildrop */
	u64 wred;		/* WRED congestion */
	u64 err_cond;		/* Error condition */
	u64 early_window;	/* Order restoration, frame too early */
	u64 late_window;	/* Order restoration, frame too late */
	u64 fq_tdrop;		/* FQ taildrop */
	u64 fq_retired;		/* FQ is retired */
	u64 orp_zero;		/* ORP disabled */
};

struct dpa_napi_portal {
	struct napi_struct napi;
	struct qman_portal *p;
};

struct dpa_percpu_priv_s {
	struct net_device *net_dev;
	struct dpa_napi_portal *np;
	u64 in_interrupt;
	u64 tx_returned;
	u64 tx_confirm;
	/* fragmented (non-linear) skbuffs received from the stack */
	u64 tx_frag_skbuffs;
	struct rtnl_link_stats64 stats;
	struct dpa_rx_errors rx_errors;
	struct dpa_ern_cnt ern_cnt;
};

struct dpa_priv_s {
	struct dpa_percpu_priv_s	*percpu_priv;
	struct dpa_bp *dpa_bp;
	/* Store here the needed Tx headroom for convenience and speed
	 * (even though it can be computed based on the fields of buf_layout)
	 */
	uint16_t tx_headroom;
	struct net_device *net_dev;
	struct mac_device	*mac_dev;
	struct qman_fq		*egress_fqs[DPAA_ETH_TX_QUEUES];
	struct qman_fq		*conf_fqs[DPAA_ETH_TX_QUEUES];

	size_t bp_count;

	uint16_t		 channel;	/* "fsl,qman-channel-id" */
	struct list_head	 dpa_fq_list;
#ifdef CONFIG_FSL_DPAA_TX_RECYCLE
	struct qman_fq		*recycle_fqs[DPAA_ETH_TX_QUEUES];
#endif

#ifdef CONFIG_FSL_DPAA_ETH_DEBUGFS
	struct dentry		*debugfs_file;
#endif /* CONFIG_FSL_DPAA_ETH_DEBUGFS */

	uint32_t		 msg_enable;	/* net_device message level */
#ifdef CONFIG_FSL_DPAA_1588
	struct dpa_ptp_tsu	 *tsu;
#endif

#if defined(CONFIG_FSL_DPAA_FMAN_UNIT_TESTS)
/* TODO: this is temporary until pcd support is implemented in dpaa */
	int			priv_pcd_num_ranges;
	struct pcd_range	priv_pcd_ranges[FMAN_PCD_TESTS_MAX_NUM_RANGES];
#endif

	struct {
		/**
		 * All egress queues to a given net device belong to one
		 * (and the same) congestion group.
		 */
		struct qman_cgr cgr;
		/* If congested, when it began. Used for performance stats. */
		u32 congestion_start_jiffies;
		/* Number of jiffies the Tx port was congested. */
		u32 congested_jiffies;
		/**
		 * Counter for the number of times the CGR
		 * entered congestion state
		 */
		u32 cgr_congested_count;
	} cgr_data;
	/* Use a per-port CGR for ingress traffic. */
	bool use_ingress_cgr;
	struct qman_cgr ingress_cgr;

#ifdef CONFIG_FSL_DPAA_TS
	bool ts_tx_en; /* Tx timestamping enabled */
	bool ts_rx_en; /* Rx timestamping enabled */
#endif /* CONFIG_FSL_DPAA_TS */

	struct dpa_buffer_layout_s *buf_layout;
	uint16_t rx_headroom;
	char if_type[30];

	void *peer;
};

struct fm_port_fqs {
	struct dpa_fq *tx_defq;
	struct dpa_fq *tx_errq;
	struct dpa_fq *rx_defq;
	struct dpa_fq *rx_errq;
};

/* functions with different implementation for SG and non-SG: */
int dpa_bp_priv_seed(struct dpa_bp *dpa_bp);
int dpaa_eth_refill_bpools(struct dpa_bp *dpa_bp, int *count_ptr);
void __hot _dpa_rx(struct net_device *net_dev,
		struct qman_portal *portal,
		const struct dpa_priv_s *priv,
		struct dpa_percpu_priv_s *percpu_priv,
		const struct qm_fd *fd,
		u32 fqid,
		int *count_ptr);
int __hot dpa_tx(struct sk_buff *skb, struct net_device *net_dev);
struct sk_buff *_dpa_cleanup_tx_fd(const struct dpa_priv_s *priv,
				   const struct qm_fd *fd);
void __hot _dpa_process_parse_results(const fm_prs_result_t *parse_results,
				      const struct qm_fd *fd,
				      struct sk_buff *skb,
				      int *use_gro);

void dpa_bp_add_8_bufs(const struct dpa_bp *dpa_bp, int cpu_id);
int _dpa_bp_add_8_bufs(const struct dpa_bp *dpa_bp);

/* Turn on HW checksum computation for this outgoing frame.
 * If the current protocol is not something we support in this regard
 * (or if the stack has already computed the SW checksum), we do nothing.
 *
 * Returns 0 if all goes well (or HW csum doesn't apply), and a negative value
 * otherwise.
 *
 * Note that this function may modify the fd->cmd field and the skb data buffer
 * (the Parse Results area).
 */
int dpa_enable_tx_csum(struct dpa_priv_s *priv,
	struct sk_buff *skb, struct qm_fd *fd, char *parse_results);

static inline int dpaa_eth_napi_schedule(struct dpa_percpu_priv_s *percpu_priv,
			struct qman_portal *portal)
{
	/* In case of threaded ISR for RT enable kernel,
	 * in_irq() does not return appropriate value, so use
	 * in_serving_softirq to distinguish softirq or irq context.
	 */
	if (unlikely(in_irq() || !in_serving_softirq())) {
		/* Disable QMan IRQ and invoke NAPI */
		int ret = qman_p_irqsource_remove(portal, QM_PIRQ_DQRI);
		if (likely(!ret)) {
			const struct qman_portal_config *pc =
					qman_p_get_portal_config(portal);
			struct dpa_napi_portal *np =
					&percpu_priv->np[pc->index];

			np->p = portal;
			napi_schedule(&np->napi);
			percpu_priv->in_interrupt++;
			return 1;
		}
	}
	return 0;
}

static inline ssize_t __const __must_check __attribute__((nonnull))
dpa_fd_length(const struct qm_fd *fd)
{
	return fd->length20;
}

static inline ssize_t __const __must_check __attribute__((nonnull))
dpa_fd_offset(const struct qm_fd *fd)
{
	return fd->offset;
}

/* Verifies if the skb length is below the interface MTU */
static inline int dpa_check_rx_mtu(struct sk_buff *skb, int mtu)
{
	if (unlikely(skb->len > mtu))
		if ((skb->protocol != ETH_P_8021Q) || (skb->len > mtu + 4))
			return -1;

	return 0;
}

static inline uint16_t dpa_get_headroom(struct dpa_buffer_layout_s *bl)
{
	uint16_t headroom;
	/* The frame headroom must accomodate:
	 * - the driver private data area
	 * - parse results, hash results, timestamp if selected
	 * - manip extra space
	 * If either hash results or time stamp are selected, both will
	 * be copied to/from the frame headroom, as TS is located between PR and
	 * HR in the IC and IC copy size has a granularity of 16bytes
	 * (see description of FMBM_RICP and FMBM_TICP registers in DPAARM)
	 *
	 * Also make sure the headroom is a multiple of data_align bytes
	 */
	headroom = bl->priv_data_size +
		   (bl->parse_results ? DPA_PARSE_RESULTS_SIZE : 0) +
		   (bl->hash_results || bl->time_stamp ?
		    DPA_TIME_STAMP_SIZE + DPA_HASH_RESULTS_SIZE : 0) +
		   bl->manip_extra_space;

	return bl->data_align ? ALIGN(headroom, bl->data_align) : headroom;
}

static inline uint16_t dpa_get_buffer_size(struct dpa_buffer_layout_s *bl,
					   uint16_t data_size)
{
	return dpa_get_headroom(bl) + data_size;
}

int fm_mac_dump_regs(struct mac_device *h_dev, char *buf, int n);

void dpaa_eth_sysfs_remove(struct device *dev);
void dpaa_eth_sysfs_init(struct device *dev);

void dpa_private_napi_del(struct net_device *net_dev);

/* Equivalent to a memset(0), but works faster */
static inline void clear_fd(struct qm_fd *fd)
{
	fd->opaque_addr = 0;
	fd->opaque = 0;
	fd->cmd = 0;
}

static inline int __hot dpa_xmit(struct dpa_priv_s *priv,
			struct rtnl_link_stats64 *percpu_stats, int queue,
			struct qm_fd *fd)
{
	int err, i;
	struct qman_fq *egress_fq;

#ifdef CONFIG_FSL_DPAA_TX_RECYCLE
	/* Choose egress fq based on whether we want
	 * to recycle the frame or not
	 */
	if (fd->cmd & FM_FD_CMD_FCO)
		egress_fq = priv->recycle_fqs[queue];
	else
		egress_fq = priv->egress_fqs[queue];
#else
	egress_fq = priv->egress_fqs[queue];
#endif

	/* Trace this Tx fd */
	trace_dpa_tx_fd(priv->net_dev, egress_fq, fd);

	for (i = 0; i < 100000; i++) {
		err = qman_enqueue(egress_fq, fd, 0);
		if (err != -EBUSY)
			break;
	}

	if (unlikely(err < 0)) {
		/* TODO differentiate b/w -EBUSY (EQCR full) and other codes? */
		percpu_stats->tx_errors++;
		percpu_stats->tx_fifo_errors++;
		return err;
	}

	percpu_stats->tx_packets++;
	percpu_stats->tx_bytes += dpa_fd_length(fd);

	return 0;
}

#if defined CONFIG_FSL_DPAA_ETH_WQ_LEGACY
#define DPA_NUM_WQS 8
/* Older WQ assignment: statically-defined FQIDs (such as PCDs) are assigned
 * round-robin to all WQs available. Dynamically-allocated FQIDs go to WQ7.
 *
 * Not necessarily the best scheme, but worked fine so far, so we might want
 * to keep it around for a while.
 */
static inline void _dpa_assign_wq(struct dpa_fq *fq)
{
	fq->wq = fq->fqid ? fq->fqid % DPA_NUM_WQS : DPA_NUM_WQS - 1;
}
#elif defined CONFIG_FSL_DPAA_ETH_WQ_MULTI
/* Use multiple WQs for FQ assignment:
 *	- Tx Confirmation queues go to WQ1.
 *	- Rx Default, Tx and PCD queues go to WQ3 (no differentiation between
 *	  Rx and Tx traffic, or between Rx Default and Rx PCD frames).
 *	- Rx Error and Tx Error queues go to WQ2 (giving them a better chance
 *	  to be scheduled, in case there are many more FQs in WQ3).
 * This ensures that Tx-confirmed buffers are timely released. In particular,
 * it avoids congestion on the Tx Confirm FQs, which can pile up PFDRs if they
 * are greatly outnumbered by other FQs in the system (usually PCDs), while
 * dequeue scheduling is round-robin.
 */
static inline void _dpa_assign_wq(struct dpa_fq *fq)
{
	switch (fq->fq_type) {
	case FQ_TYPE_TX_CONFIRM:
	case FQ_TYPE_TX_CONF_MQ:
		fq->wq = 1;
		break;
	case FQ_TYPE_RX_DEFAULT:
	case FQ_TYPE_TX:
#ifdef CONFIG_FSL_DPAA_TX_RECYCLE
	case FQ_TYPE_TX_RECYCLE:
#endif
	case FQ_TYPE_RX_PCD:
		fq->wq = 3;
		break;
	case FQ_TYPE_RX_ERROR:
	case FQ_TYPE_TX_ERROR:
		fq->wq = 2;
		break;
	default:
		WARN(1, "Invalid FQ type %d for FQID %d!\n",
		       fq->fq_type, fq->fqid);
	}
}
#else
/* This shouldn't happen, since we've made a "default" choice in the Kconfig. */
#warning "No WQ assignment scheme chosen; Kconfig out-of-sync?"
#endif /* CONFIG_FSL_DPAA_ETH_WQ_ASSIGN_* */


#ifdef CONFIG_FSL_DPAA_ETH_USE_NDO_SELECT_QUEUE
/* Use in lieu of skb_get_queue_mapping() */
#define dpa_get_queue_mapping(skb) \
	smp_processor_id()
#else
/* Use the queue selected by XPS */
#define dpa_get_queue_mapping(skb) \
	skb_get_queue_mapping(skb)
#endif

static inline void _dpa_bp_free_pf(void *addr)
{
	put_page(virt_to_head_page(addr));
}

#endif	/* __DPA_H */
