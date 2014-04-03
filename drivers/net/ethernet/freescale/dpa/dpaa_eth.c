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

#define pr_fmt(fmt) \
	KBUILD_MODNAME ": %s:%hu:%s() " fmt, \
	KBUILD_BASENAME".c", __LINE__, __func__

#include <linux/init.h>
#include <linux/module.h>
#include <linux/sort.h>
#include <linux/of_mdio.h>
#include <linux/of_platform.h>
#include <linux/of_net.h>
#include <linux/kthread.h>
#include <linux/io.h>
#include <linux/etherdevice.h>
#include <linux/if_arp.h>	/* arp_hdr_len() */
#include <linux/if_vlan.h>	/* VLAN_HLEN */
#include <linux/icmp.h>		/* struct icmphdr */
#include <linux/ip.h>		/* struct iphdr */
#include <linux/ipv6.h>		/* struct ipv6hdr */
#include <linux/udp.h>		/* struct udphdr */
#include <linux/tcp.h>		/* struct tcphdr */
#include <linux/net.h>		/* net_ratelimit() */
#include <linux/net_tstamp.h>	/* struct hwtstamp_config */
#include <linux/if_ether.h>	/* ETH_P_IP and ETH_P_IPV6 */
#include <linux/highmem.h>
#include <linux/percpu.h>
#include <linux/dma-mapping.h>
#include <asm/smp.h>		/* get_hard_smp_processor_id() */
#include <linux/fsl_bman.h>

#include "fsl_fman.h"
#include "fm_ext.h"
#include "fm_port_ext.h"

#include "mac.h"
#include "dpaa_eth.h"
#include "dpaa_1588.h"
#ifdef CONFIG_FSL_DPAA_ETH_DEBUGFS
#include "dpaa_debugfs.h"
#endif /* CONFIG_FSL_DPAA_ETH_DEBUGFS */

/* CREATE_TRACE_POINTS only needs to be defined once. Other dpa files
 * using trace events only need to #include <trace/events/sched.h>
 */
#define CREATE_TRACE_POINTS
#include "dpaa_eth_trace.h"


#define ARRAY2_SIZE(arr)	(ARRAY_SIZE(arr) * ARRAY_SIZE((arr)[0]))

/* DPAA platforms benefit from hardware-assisted queue management */
#ifdef CONFIG_AS_FASTPATH
#define DPA_NETIF_FEATURES	(NETIF_F_HW_QDISC | NETIF_F_HW_ACCEL_MQ)
#else
#define DPA_NETIF_FEATURES	NETIF_F_HW_ACCEL_MQ
#endif

#ifdef CONFIG_FSL_DPAA_ETH_UNIT_TESTS
#undef CONFIG_FSL_DPAA_ETH_UNIT_TESTS
#endif

#define DPA_NAPI_WEIGHT		64

/* Size in bytes of the Congestion State notification threshold on 10G ports */
#define DPA_CS_THRESHOLD_10G	0x10000000
/*
 * Size in bytes of the Congestion State notification threshold on 1G ports.

 * The 1G dTSECs can quite easily be flooded by cores doing Tx in a tight loop
 * (e.g. by sending UDP datagrams at "while(1) speed"),
 * and the larger the frame size, the more acute the problem.
 *
 * So we have to find a balance between these factors:
 *	- avoiding the device staying congested for a prolonged time (risking
 *	  the netdev watchdog to fire - see also the tx_timeout module param);
 *	- affecting performance of protocols such as TCP, which otherwise
 *	  behave well under the congestion notification mechanism;
 *	- preventing the Tx cores from tightly-looping (as if the congestion
 *	  threshold was too low to be effective);
 *	- running out of memory if the CS threshold is set too high.
 */
#define DPA_CS_THRESHOLD_1G	0x06000000

/* Size in bytes of the FQ taildrop threshold */
#define DPA_FQ_TD		0x200000

/* S/G table requires at least 256 bytes */
#define sgt_buffer_size(priv) \
	dpa_get_buffer_size(&priv->buf_layout[TX], 256)

/* Maximum frame size on Tx for which skb copying is preferrable to
 * creating a S/G frame */
#define DPA_SKB_COPY_MAX_SIZE	256

/* Valid checksum indication */
#define DPA_CSUM_VALID		0xFFFF

/* Maximum offset value for a contig or sg FD (represented on 9bits) */
#define DPA_MAX_FD_OFFSET	((1 << 9) - 1)

/*
 * Maximum size of a buffer for which recycling is allowed.
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

/* For MAC-based interfaces, we compute the tx needed headroom from the
 * associated Tx port's buffer layout settings.
 * For MACless interfaces just use a default value.
 */
#define DPA_DEFAULT_TX_HEADROOM	64

#define DPA_DESCRIPTION "FSL DPAA Ethernet driver"

MODULE_LICENSE("Dual BSD/GPL");

MODULE_AUTHOR("Andy Fleming <afleming@freescale.com>");

MODULE_DESCRIPTION(DPA_DESCRIPTION);

static uint8_t debug = -1;
module_param(debug, byte, S_IRUGO);
MODULE_PARM_DESC(debug, "Module/Driver verbosity level");

/* This has to work in tandem with the DPA_CS_THRESHOLD_xxx values. */
static uint16_t tx_timeout = 1000;
module_param(tx_timeout, ushort, S_IRUGO);
MODULE_PARM_DESC(tx_timeout, "The Tx timeout in ms");

/* dpaa_eth mirror for the FMan values */
static int dpa_rx_extra_headroom;
static int dpa_max_frm;

static const char rtx[][3] = {
	[RX] = "RX",
	[TX] = "TX"
};

#if defined(CONFIG_FSL_FMAN_TEST)
/* Defined as weak, to be implemented by fman pcd tester. */
int dpa_alloc_pcd_fqids(struct device *, uint32_t, uint8_t, uint32_t *)
__attribute__((weak));

int dpa_free_pcd_fqids(struct device *, uint32_t) __attribute__((weak));
#endif /* CONFIG_FSL_DPAA_FMAN_UNIT_TESTS */

/* BM */

#define DPAA_ETH_MAX_PAD (L1_CACHE_BYTES * 8)

static struct dpa_bp *dpa_bp_array[64];

static struct dpa_bp *default_pool;
static bool default_pool_seeded;
static uint32_t default_buf_size;

/* A set of callbacks for hooking into the fastpath at different points. */
static struct dpaa_eth_hooks_s dpaa_eth_hooks;
/*
 * This function should only be called on the probe paths, since it makes no
 * effort to guarantee consistency of the destination hooks structure.
 */
void fsl_dpaa_eth_set_hooks(struct dpaa_eth_hooks_s *hooks)
{
	if (hooks)
		dpaa_eth_hooks = *hooks;
	else
		pr_err("NULL pointer to hooks!\n");
}
EXPORT_SYMBOL(fsl_dpaa_eth_set_hooks);


struct dpa_bp *dpa_bpid2pool(int bpid)
{
	return dpa_bp_array[bpid];
}

static void dpa_bp_depletion(struct bman_portal	*portal,
		struct bman_pool *pool, void *cb_ctx, int depleted)
{
	if (net_ratelimit())
		pr_err("Invalid Pool depleted notification!\n");
}

/*
 * Copy from a memory region that requires kmapping to a linear buffer,
 * taking into account page boundaries in the source
 */
static void
copy_from_unmapped_area(void *dest, dma_addr_t phys_start, size_t buf_size)
{
	struct page *page;
	size_t size, offset;
	void *page_vaddr;

	while (buf_size > 0) {
		offset = offset_in_page(phys_start);
		size = (offset + buf_size > PAGE_SIZE) ?
			PAGE_SIZE - offset : buf_size;

		page = pfn_to_page(phys_start >> PAGE_SHIFT);
		page_vaddr = kmap_atomic(page);

		memcpy(dest, page_vaddr + offset, size);

		kunmap_atomic(page_vaddr);

		phys_start += size;
		dest += size;
		buf_size -= size;
	}
}

/*
 * Copy to a memory region that requires kmapping from a linear buffer,
 * taking into account page boundaries in the destination
 */
static void
copy_to_unmapped_area(dma_addr_t phys_start, void *src, size_t buf_size)
{
	struct page *page;
	size_t size, offset;
	void *page_vaddr;

	while (buf_size > 0) {
		offset = offset_in_page(phys_start);
		size = (offset + buf_size > PAGE_SIZE) ?
				PAGE_SIZE - offset : buf_size;

		page = pfn_to_page(phys_start >> PAGE_SHIFT);
		page_vaddr = kmap_atomic(page);

		memcpy(page_vaddr + offset, src, size);

		kunmap_atomic(page_vaddr);

		phys_start += size;
		src += size;
		buf_size -= size;
	}
}

#ifndef CONFIG_FSL_DPAA_ETH_SG_SUPPORT
/* Allocate 8 socket buffers.
 * These buffers are counted for a particular CPU.
 */
static void dpa_bp_add_8(const struct dpa_bp *dpa_bp, unsigned int cpu)
{
	struct bm_buffer bmb[8];
	struct sk_buff **skbh;
	dma_addr_t addr;
	int i;
	struct sk_buff *skb;
	int *count_ptr;

	count_ptr = per_cpu_ptr(dpa_bp->percpu_count, cpu);

	for (i = 0; i < 8; i++) {
		/*
		 * The buffers tend to be aligned all to the same cache
		 * index.  A standard dequeue operation pulls in 15 packets.
		 * This means that when it stashes, it evicts half of the
		 * packets it's stashing. In order to prevent that, we pad
		 * by a variable number of cache lines, to reduce collisions.
		 * We always pad by at least 1 cache line, because we want
		 * a little extra room at the beginning for IPSec and to
		 * accommodate NET_IP_ALIGN.
		 */
		int pad = (i + 1) * L1_CACHE_BYTES;

		skb = dev_alloc_skb(dpa_bp->size + pad);
		if (unlikely(!skb)) {
			printk(KERN_ERR "dev_alloc_skb() failed\n");
			bm_buffer_set64(&bmb[i], 0);
			break;
		}

		skbh = (struct sk_buff **)(skb->head + pad);
		*skbh = skb;

		/*
		 * Here we need to map only for device write (DMA_FROM_DEVICE),
		 * but on Tx recycling we may also get buffers in the pool that
		 * are mapped bidirectionally.
		 * Use DMA_BIDIRECTIONAL here as well to avoid any
		 * inconsistencies when unmapping.
		 */
		addr = dma_map_single(dpa_bp->dev, skb->head + pad,
				dpa_bp->size, DMA_BIDIRECTIONAL);
		if (unlikely(dma_mapping_error(dpa_bp->dev, addr))) {
			dev_err(dpa_bp->dev, "DMA mapping failed");
			break;
		}

		bm_buffer_set64(&bmb[i], addr);
	}

	/* Avoid releasing a completely null buffer; bman_release() requires
	 * at least one buf. */
	if (likely(i)) {
		/*
		 * Release the buffers. In case bman is busy, keep trying
		 * until successful. bman_release() is guaranteed to succeed
		 * in a reasonable amount of time
		 */
		while (bman_release(dpa_bp->pool, bmb, i, 0))
			cpu_relax();

		*count_ptr += i;
	}
}

void dpa_make_private_pool(struct dpa_bp *dpa_bp)
{
	int i;

	dpa_bp->percpu_count = alloc_percpu(*dpa_bp->percpu_count);

	/* Give each cpu an allotment of "count" buffers */
	for_each_online_cpu(i) {
		int j;

		for (j = 0; j < dpa_bp->target_count; j += 8)
			dpa_bp_add_8(dpa_bp, i);
	}
}
#endif /* CONFIG_FSL_DPAA_ETH_SG_SUPPORT */

static void dpaa_eth_seed_pool(struct dpa_bp *bp)
{
	int count = bp->target_count;
	size_t addr = bp->paddr;

	while (count) {
		struct bm_buffer bufs[8];
		int num_bufs = 0;

		do {
			BUG_ON(addr > 0xffffffffffffull);
			bufs[num_bufs].bpid = bp->bpid;
			bm_buffer_set64(&bufs[num_bufs++], addr);
			addr += bp->size;

		} while (--count && (num_bufs < 8));

		while (bman_release(bp->pool, bufs, num_bufs, 0))
			cpu_relax();
	}
}

/*
 * Add buffers/pages/skbuffs for Rx processing whenever bpool count falls below
 * REFILL_THRESHOLD.
 */
static void dpaa_eth_refill_bpools(struct dpa_percpu_priv_s *percpu_priv)
{
	int *countptr = percpu_priv->dpa_bp_count;
	int count = *countptr;
	const struct dpa_bp *dpa_bp = percpu_priv->dpa_bp;
	int new_pages;
#ifndef CONFIG_FSL_DPAA_ETH_SG_SUPPORT
	/* this function is called in softirq context;
	 * no need to protect smp_processor_id() on RT kernel
	 */
	unsigned int cpu = smp_processor_id();

	if (unlikely(count < CONFIG_FSL_DPAA_ETH_REFILL_THRESHOLD)) {
		int i;

		for (i = count; i < CONFIG_FSL_DPAA_ETH_MAX_BUF_COUNT; i += 8)
			dpa_bp_add_8(dpa_bp, cpu);
	}
#else
	/* Add pages to the buffer pool */
	while (count < CONFIG_FSL_DPAA_ETH_MAX_BUF_COUNT) {
		new_pages = _dpa_bp_add_8_pages(dpa_bp);
		if (unlikely(!new_pages)) {
			/* Avoid looping forever if we've temporarily
			 * run out of memory. We'll try again at the next
			 * NAPI cycle.
			 */
			break;
		}
		count += new_pages;
	}
	*countptr = count;
#endif
}

static int dpa_make_shared_port_pool(struct dpa_bp *bp)
{
	/*
	 * In MAC-less and Shared-MAC scenarios the physical
	 * address of the buffer pool in device tree is set
	 * to 0 to specify that another entity (USDPAA) will
	 * allocate and seed the buffers
	 */
	if (!bp->paddr)
		return 0;

	devm_request_mem_region(bp->dev, bp->paddr,
			bp->size * bp->config_count, KBUILD_MODNAME);
	bp->vaddr = devm_ioremap_prot(bp->dev, bp->paddr,
			bp->size * bp->config_count, 0);
	if (bp->vaddr == NULL) {
		pr_err("Could not map memory for pool %d\n", bp->bpid);
		return -EIO;
	}

	if (bp->seed_pool)
		dpaa_eth_seed_pool(bp);

	return 0;
}

static int __must_check __attribute__((nonnull))
dpa_bp_alloc(struct dpa_bp *dpa_bp)
{
	int err;
	struct bman_pool_params	 bp_params;
	struct platform_device *pdev;

	BUG_ON(dpa_bp->size == 0);
	BUG_ON(dpa_bp->config_count == 0);

	bp_params.flags = BMAN_POOL_FLAG_DEPLETION;
	bp_params.cb = dpa_bp_depletion;
	bp_params.cb_ctx = dpa_bp;

	/* We support two options.  Either a global shared pool, or
	 * a specified pool. If the pool is specified, we only
	 * create one per bpid */
	if (dpa_bp->kernel_pool && default_pool) {
		atomic_inc(&default_pool->refs);
		return 0;
	}

	if (dpa_bp_array[dpa_bp->bpid]) {
		atomic_inc(&dpa_bp_array[dpa_bp->bpid]->refs);
		return 0;
	}

	if (dpa_bp->bpid == 0)
		bp_params.flags |= BMAN_POOL_FLAG_DYNAMIC_BPID;
	else
		bp_params.bpid = dpa_bp->bpid;

	dpa_bp->pool = bman_new_pool(&bp_params);
	if (unlikely(dpa_bp->pool == NULL)) {
		pr_err("bman_new_pool() failed\n");
		return -ENODEV;
	}

	dpa_bp->bpid = bman_get_params(dpa_bp->pool)->bpid;

	pdev = platform_device_register_simple("dpaa_eth_bpool",
			dpa_bp->bpid, NULL, 0);
	if (IS_ERR(pdev)) {
		err = PTR_ERR(pdev);
		goto pdev_register_failed;
	}

	err = dma_set_mask(&pdev->dev, DMA_BIT_MASK(40));
	if (err)
		goto pdev_mask_failed;

	dpa_bp->dev = &pdev->dev;

	if (dpa_bp->kernel_pool) {
		if (!default_pool)
			default_pool = dpa_bp;
	} else {
		err = dpa_make_shared_port_pool(dpa_bp);
		if (err)
			goto make_shared_pool_failed;
	}

	dpa_bp_array[dpa_bp->bpid] = dpa_bp;

	atomic_set(&dpa_bp->refs, 1);

	return 0;

make_shared_pool_failed:
pdev_mask_failed:
	platform_device_unregister(pdev);
pdev_register_failed:
	bman_free_pool(dpa_bp->pool);

	return err;
}

#ifndef CONFIG_FSL_DPAA_ETH_SG_SUPPORT
static inline void _dpa_bp_free_buf(void *addr)
{
	struct sk_buff **skbh = addr;
	struct sk_buff *skb;

	skb = *skbh;
	dev_kfree_skb_any(skb);
}
#else
static inline void _dpa_bp_free_buf(void *addr)
{
	free_page((unsigned long)addr);
}
#endif

static void __cold __attribute__((nonnull))
_dpa_bp_free(struct dpa_bp *dpa_bp)
{
	struct dpa_bp *bp = dpa_bpid2pool(dpa_bp->bpid);

	if (!atomic_dec_and_test(&bp->refs))
		return;

	if (bp->kernel_pool) {
		int num;

		do {
			struct bm_buffer bmb[8];
			int i;

			num = bman_acquire(bp->pool, bmb, 8, 0);

			for (i = 0; i < num; i++) {
				dma_addr_t addr = bm_buf_addr(&bmb[i]);

				dma_unmap_single(bp->dev, addr, bp->size,
						DMA_BIDIRECTIONAL);

				_dpa_bp_free_buf(phys_to_virt(addr));
			}
		} while (num == 8);
	}

	dpa_bp_array[bp->bpid] = 0;
	bman_free_pool(bp->pool);
}

static void __cold __attribute__((nonnull))
dpa_bp_free(struct dpa_priv_s *priv, struct dpa_bp *dpa_bp)
{
	int i;

	for (i = 0; i < priv->bp_count; i++)
		_dpa_bp_free(&priv->dpa_bp[i]);
}

/* QM */

static struct qman_fq *_dpa_get_tx_conf_queue(const struct dpa_priv_s *priv,
					       struct qman_fq *tx_fq)
{
	int i;

	for (i = 0; i < DPAA_ETH_TX_QUEUES; i++)
		if (priv->egress_fqs[i] == tx_fq)
			return priv->conf_fqs[i];

	return NULL;
}

static int __must_check __attribute__((nonnull))
_dpa_fq_alloc(struct list_head *list, struct dpa_fq *dpa_fq)
{
	int			 _errno;
	const struct dpa_priv_s	*priv;
	struct device		*dev;
	struct qman_fq		*fq;
	struct qm_mcc_initfq	 initfq;
	struct qman_fq		*confq;

	priv = netdev_priv(dpa_fq->net_dev);
	dev = dpa_fq->net_dev->dev.parent;

	if (dpa_fq->fqid == 0)
		dpa_fq->flags |= QMAN_FQ_FLAG_DYNAMIC_FQID;

	dpa_fq->init = !(dpa_fq->flags & QMAN_FQ_FLAG_NO_MODIFY);

	_errno = qman_create_fq(dpa_fq->fqid, dpa_fq->flags, &dpa_fq->fq_base);
	if (_errno) {
		dev_err(dev, "qman_create_fq() failed\n");
		return _errno;
	}
	fq = &dpa_fq->fq_base;

	if (dpa_fq->init) {
		initfq.we_mask = QM_INITFQ_WE_FQCTRL;
		/* FIXME: why would we want to keep an empty FQ in cache? */
		initfq.fqd.fq_ctrl = QM_FQCTRL_PREFERINCACHE;

#ifdef CONFIG_FSL_DPAA_ETH_SG_SUPPORT
		/* Try to reduce the number of portal interrupts for
		 * Tx Confirmation FQs.
		 */
		if (dpa_fq->fq_type == FQ_TYPE_TX_CONFIRM)
			initfq.fqd.fq_ctrl |= QM_FQCTRL_HOLDACTIVE;
#endif

		/* FQ placement */
		initfq.we_mask |= QM_INITFQ_WE_DESTWQ;

		initfq.fqd.dest.channel	= dpa_fq->channel;
		initfq.fqd.dest.wq = dpa_fq->wq;

		/*
		 * Put all egress queues in a congestion group of their own.
		 * Sensu stricto, the Tx confirmation queues are Rx FQs,
		 * rather than Tx - but they nonetheless account for the
		 * memory footprint on behalf of egress traffic. We therefore
		 * place them in the netdev's CGR, along with the Tx FQs.
		 */
		if (dpa_fq->fq_type == FQ_TYPE_TX ||
				dpa_fq->fq_type == FQ_TYPE_TX_CONFIRM) {
			initfq.we_mask |= QM_INITFQ_WE_CGID;
			initfq.fqd.fq_ctrl |= QM_FQCTRL_CGE;
			initfq.fqd.cgid = priv->cgr_data.cgr.cgrid;
			/*
			 * Set a fixed overhead accounting, in an attempt to
			 * reduce the impact of fixed-size skb shells and the
			 * driver's needed headroom on system memory. This is
			 * especially the case when the egress traffic is
			 * composed of small datagrams.
			 * Unfortunately, QMan's OAL value is capped to an
			 * insufficient value, but even that is better than
			 * no overhead accounting at all.
			 */
			initfq.we_mask |= QM_INITFQ_WE_OAC;
			initfq.fqd.oac_init.oac = QM_OAC_CG;
			initfq.fqd.oac_init.oal = min(sizeof(struct sk_buff) +
				priv->tx_headroom, (size_t)FSL_QMAN_MAX_OAL);
		}

		/*
		 * For MAC-less devices we only get here for RX frame queues
		 * initialization, which are the TX queues of the other
		 * partition.
		 * It is safe to rely on one partition to set the FQ taildrop
		 * threshold for the TX queues of the other partition
		 * because the ERN notifications will be received by the
		 * partition doing qman_enqueue.
		 */
		if (!priv->mac_dev) {
			initfq.we_mask |= QM_INITFQ_WE_TDTHRESH;
			qm_fqd_taildrop_set(&initfq.fqd.td,
					DPA_FQ_TD, 1);
			initfq.fqd.fq_ctrl = QM_FQCTRL_TDE;
		}

		/*
		 * Configure the Tx confirmation queue, now that we know
		 * which Tx queue it pairs with.
		 */
		if (dpa_fq->fq_type == FQ_TYPE_TX) {
			confq = _dpa_get_tx_conf_queue(priv, &dpa_fq->fq_base);
			if (confq) {
				initfq.we_mask |= QM_INITFQ_WE_CONTEXTA |
						  QM_INITFQ_WE_CONTEXTB;
				/* CTXA[OVFQ] = 1 */
				initfq.fqd.context_a.hi = 0x80000000;
				initfq.fqd.context_a.lo = 0x0;
				initfq.fqd.context_b = qman_fq_fqid(confq);
			}
		}

#ifdef CONFIG_FSL_DPAA_TX_RECYCLE
		/*
		 * Configure the Tx queues for recycled frames, such that the
		 * buffers are released by FMan and no confirmation is sent
		 */
		if (dpa_fq->fq_type == FQ_TYPE_TX_RECYCLE) {
			initfq.we_mask |= QM_INITFQ_WE_CONTEXTA |
					  QM_INITFQ_WE_CONTEXTB;
			/*
			 * ContextA: OVFQ=1 (use ContextB FQID for confirmation)
			 *           OVOM=1 (use contextA2 bits instead of ICAD)
			 *           A2V=1 (contextA A2 field is valid)
			 *           B0V=1 (contextB field is valid)
			 * ContextA A2: EBD=1 (deallocate buffers inside FMan)
			 * ContextB: Confirmation FQID = 0
			 */
			initfq.fqd.context_a.hi = 0x96000000;
			initfq.fqd.context_a.lo = 0x80000000;
			initfq.fqd.context_b = 0;
		}
#endif

		/* Initialization common to all ingress queues */
		if (dpa_fq->flags & QMAN_FQ_FLAG_NO_ENQUEUE) {
			initfq.we_mask |= QM_INITFQ_WE_CONTEXTA;
			initfq.fqd.fq_ctrl |=
				QM_FQCTRL_CTXASTASHING | QM_FQCTRL_AVOIDBLOCK;
			initfq.fqd.context_a.stashing.exclusive =
				QM_STASHING_EXCL_DATA | QM_STASHING_EXCL_CTX |
				QM_STASHING_EXCL_ANNOTATION;
			initfq.fqd.context_a.stashing.data_cl = 2;
			initfq.fqd.context_a.stashing.annotation_cl = 1;
			initfq.fqd.context_a.stashing.context_cl =
				DIV_ROUND_UP(sizeof(struct qman_fq), 64);
		};

		_errno = qman_init_fq(fq, QMAN_INITFQ_FLAG_SCHED, &initfq);
		if (_errno < 0) {
			dev_err(dev, "qman_init_fq(%u) = %d\n",
					qman_fq_fqid(fq), _errno);
			qman_destroy_fq(fq, 0);
			return _errno;
		}
	}

	dpa_fq->fqid = qman_fq_fqid(fq);
	list_add_tail(&dpa_fq->list, list);

	return 0;
}

static int __cold __attribute__((nonnull))
_dpa_fq_free(struct device *dev, struct qman_fq *fq)
{
	int			 _errno, __errno;
	struct dpa_fq		*dpa_fq;
	const struct dpa_priv_s	*priv;

	_errno = 0;

	dpa_fq = container_of(fq, struct dpa_fq, fq_base);
	priv = netdev_priv(dpa_fq->net_dev);

	if (dpa_fq->init) {
		_errno = qman_retire_fq(fq, NULL);
		if (unlikely(_errno < 0) && netif_msg_drv(priv))
			dev_err(dev, "qman_retire_fq(%u) = %d\n",
					qman_fq_fqid(fq), _errno);

		__errno = qman_oos_fq(fq);
		if (unlikely(__errno < 0) && netif_msg_drv(priv)) {
			dev_err(dev, "qman_oos_fq(%u) = %d\n",
					qman_fq_fqid(fq), __errno);
			if (_errno >= 0)
				_errno = __errno;
		}
	}

	qman_destroy_fq(fq, 0);
	list_del(&dpa_fq->list);

	return _errno;
}

static int __cold __attribute__((nonnull))
dpa_fq_free(struct device *dev, struct list_head *list)
{
	int		 _errno, __errno;
	struct dpa_fq	*dpa_fq, *tmp;

	_errno = 0;
	list_for_each_entry_safe(dpa_fq, tmp, list, list) {
		__errno = _dpa_fq_free(dev, (struct qman_fq *)dpa_fq);
		if (unlikely(__errno < 0) && _errno >= 0)
			_errno = __errno;
	}

	return _errno;
}

static inline void * __must_check __attribute__((nonnull))
dpa_phys2virt(const struct dpa_bp *dpa_bp, dma_addr_t addr)
{
	return dpa_bp->vaddr + (addr - dpa_bp->paddr);
}

static void
dpa_release_sgt(struct qm_sg_entry *sgt, struct dpa_bp *dpa_bp,
		struct bm_buffer *bmb)
{
	int i = 0, j;

	do {
		dpa_bp = dpa_bpid2pool(sgt[i].bpid);
		BUG_ON(IS_ERR(dpa_bp));

		j = 0;
		do {
			BUG_ON(sgt[i].extension);

			bmb[j].hi       = sgt[i].addr_hi;
			bmb[j].lo       = sgt[i].addr_lo;

			j++; i++;
		} while (j < ARRAY_SIZE(bmb) &&
				!sgt[i-1].final &&
				sgt[i-1].bpid == sgt[i].bpid);

		while (bman_release(dpa_bp->pool, bmb, j, 0))
			cpu_relax();
	} while (!sgt[i-1].final);
}

static void
dpa_fd_release_sg(const struct net_device *net_dev,
			const struct qm_fd *fd)
{
	const struct dpa_priv_s		*priv;
	struct qm_sg_entry		*sgt;
	struct dpa_bp			*_dpa_bp, *dpa_bp;
	struct bm_buffer		 _bmb, bmb[8];

	priv = netdev_priv(net_dev);

	_bmb.hi	= fd->addr_hi;
	_bmb.lo	= fd->addr_lo;

	_dpa_bp = dpa_bpid2pool(fd->bpid);

	if (_dpa_bp->vaddr) {
		sgt = dpa_phys2virt(_dpa_bp, bm_buf_addr(&_bmb)) +
					dpa_fd_offset(fd);
		dpa_release_sgt(sgt, dpa_bp, bmb);
	} else {
		sgt = kmalloc(DPA_SGT_MAX_ENTRIES * sizeof(*sgt), GFP_ATOMIC);
		if (sgt == NULL) {
			if (netif_msg_tx_err(priv) && net_ratelimit())
				netdev_err(net_dev,
					"Memory allocation failed\n");
			return;
		}

		copy_from_unmapped_area(sgt, bm_buf_addr(&_bmb) +
						dpa_fd_offset(fd),
					min(DPA_SGT_MAX_ENTRIES * sizeof(*sgt),
						_dpa_bp->size));
		dpa_release_sgt(sgt, dpa_bp, bmb);
		kfree(sgt);
	}

	while (bman_release(_dpa_bp->pool, &_bmb, 1, 0))
		cpu_relax();
}

void __attribute__((nonnull))
dpa_fd_release(const struct net_device *net_dev, const struct qm_fd *fd)
{
	struct qm_sg_entry		*sgt;
	struct dpa_bp			*_dpa_bp, *dpa_bp;
	struct bm_buffer		 _bmb, bmb[8];

	_bmb.hi	= fd->addr_hi;
	_bmb.lo	= fd->addr_lo;

	_dpa_bp = dpa_bpid2pool(fd->bpid);
	BUG_ON(IS_ERR(_dpa_bp));

	if (fd->format == qm_fd_sg) {
		sgt = (phys_to_virt(bm_buf_addr(&_bmb)) + dpa_fd_offset(fd));
		dpa_release_sgt(sgt, dpa_bp, bmb);
	}

	while (bman_release(_dpa_bp->pool, &_bmb, 1, 0))
		cpu_relax();
}
EXPORT_SYMBOL(dpa_fd_release);

#ifndef CONFIG_FSL_DPAA_ETH_SG_SUPPORT
/*
 * Cleanup function for outgoing frame descriptors that were built on Tx path,
 * either contiguous frames or scatter/gather ones with a single data buffer.
 * Skb freeing is not handled here.
 *
 * This function may be called on error paths in the Tx function, so guard
 * against cases when not all fd relevant fields were filled in.
 *
 * Return the skb backpointer, since for S/G frames the buffer containing it
 * gets freed here.
 */
struct sk_buff *_dpa_cleanup_tx_fd(const struct dpa_priv_s *priv,
			       const struct qm_fd *fd)
{
	dma_addr_t addr = qm_fd_addr(fd);
	dma_addr_t sg_addr;
	void *vaddr;
	struct dpa_bp *bp = priv->dpa_bp;
	struct sk_buff **skbh;
	struct sk_buff *skb = NULL;

	BUG_ON(!fd);

	if (unlikely(!addr))
		return skb;
	vaddr = phys_to_virt(addr);
	skbh = (struct sk_buff **)vaddr;

	if (fd->format == qm_fd_contig) {
		/* For contiguous frames, just unmap data buffer;
		 * mapping direction depends on whether the frame was
		 * meant to be recycled or not */
		if (fd->cmd & FM_FD_CMD_FCO)
			dma_unmap_single(bp->dev, addr, bp->size,
					 DMA_BIDIRECTIONAL);
		else
			dma_unmap_single(bp->dev, addr, bp->size,
					 DMA_TO_DEVICE);
		/* Retrieve the skb backpointer */
		skb = *skbh;
	} else {
		/* For s/g, we need to unmap both the SGT buffer and the
		 * data buffer, and also free the SGT buffer */
		struct qm_sg_entry *sg_entry;

		/* Unmap first buffer (contains S/G table) */
		dma_unmap_single(bp->dev, addr, sgt_buffer_size(priv),
				 DMA_TO_DEVICE);

		/* Unmap data buffer */
		sg_entry = (struct qm_sg_entry *)(vaddr + fd->offset);
		sg_addr = qm_sg_addr(sg_entry);
		if (likely(sg_addr))
			dma_unmap_single(bp->dev, sg_addr, bp->size,
					 DMA_TO_DEVICE);
		/* Retrieve the skb backpointer */
		skb = *skbh;

	}
/* on some error paths this might not be necessary: */
#ifdef CONFIG_FSL_DPAA_1588
	if (priv->tsu && priv->tsu->valid && priv->tsu->hwts_tx_en_ioctl)
		dpa_ptp_store_txstamp(priv, skb, (void *)skbh);
#endif
#ifdef CONFIG_FSL_DPAA_TS
	if (unlikely(priv->ts_tx_en &&
			skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)) {
		struct skb_shared_hwtstamps shhwtstamps;

		if (!dpa_get_ts(priv, TX, &shhwtstamps, (void *)skbh))
			skb_tstamp_tx(skb, &shhwtstamps);
	}
#endif /* CONFIG_FSL_DPAA_TS */

	/* Free first buffer (which was allocated on Tx) containing the
	 * skb backpointer and hardware timestamp information
	 */
	if (fd->format != qm_fd_contig)
		kfree(vaddr);

	return skb;
}
#endif /* CONFIG_FSL_DPAA_ETH_SG_SUPPORT */

/* net_device */

/**
 * @param net_dev the device for which statistics are calculated
 * @param stats the function fills this structure with the device's statistics
 * @return the address of the structure containing the statistics
 *
 * Calculates the statistics for the given device by adding the statistics
 * collected by each CPU.
 */
static struct rtnl_link_stats64 * __cold
dpa_get_stats64(struct net_device *net_dev,
		struct rtnl_link_stats64 *stats)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	u64 *cpustats;
	u64 *netstats = (u64 *)stats;
	int i, j;
	struct dpa_percpu_priv_s	*percpu_priv;
	int numstats = sizeof(struct rtnl_link_stats64) / sizeof(u64);

	for_each_online_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);

		cpustats = (u64 *)&percpu_priv->stats;

		for (j = 0; j < numstats; j++)
			netstats[j] += cpustats[j];
	}

	return stats;
}

static int dpa_change_mtu(struct net_device *net_dev, int new_mtu)
{
	const int max_mtu = dpa_get_max_mtu();
	const int min_mtu = dpa_get_min_mtu();

	/* Make sure we don't exceed the Ethernet controller's MAXFRM */
	if (new_mtu < min_mtu || new_mtu > max_mtu) {
		netdev_err(net_dev, "Invalid L3 mtu %d "
				"(must be between %d and %d).\n",
				new_mtu, min_mtu, max_mtu);
		return -EINVAL;
	}
	net_dev->mtu = new_mtu;

	return 0;
}

/* .ndo_init callback */
static int dpa_ndo_init(struct net_device *net_dev)
{
	/*
	 * If fsl_fm_max_frm is set to a higher value than the all-common 1500,
	 * we choose conservatively and let the user explicitly set a higher
	 * MTU via ifconfig. Otherwise, the user may end up with different MTUs
	 * in the same LAN.
	 * If on the other hand fsl_fm_max_frm has been chosen below 1500,
	 * start with the maximum allowed.
	 */
	int init_mtu = min(dpa_get_max_mtu(), ETH_DATA_LEN);

	pr_debug("Setting initial MTU on net device: %d\n", init_mtu);
	net_dev->mtu = init_mtu;

	return 0;
}

static int dpa_set_mac_address(struct net_device *net_dev, void *addr)
{
	const struct dpa_priv_s	*priv;
	int			 _errno;

	priv = netdev_priv(net_dev);

	_errno = eth_mac_addr(net_dev, addr);
	if (_errno < 0) {
		if (netif_msg_drv(priv))
			netdev_err(net_dev,
				       "eth_mac_addr() = %d\n",
				       _errno);
		return _errno;
	}

	if (!priv->mac_dev)
		/* MAC-less interface, so nothing more to do here */
		return 0;

	_errno = priv->mac_dev->change_addr(priv->mac_dev, net_dev->dev_addr);
	if (_errno < 0) {
		if (netif_msg_drv(priv))
			netdev_err(net_dev,
				       "mac_dev->change_addr() = %d\n",
				       _errno);
		return _errno;
	}

	return 0;
}

static void dpa_set_rx_mode(struct net_device *net_dev)
{
	int			 _errno;
	const struct dpa_priv_s	*priv;

	priv = netdev_priv(net_dev);

	if (!priv->mac_dev)
		return;

	if (!!(net_dev->flags & IFF_PROMISC) != priv->mac_dev->promisc) {
		_errno = priv->mac_dev->change_promisc(priv->mac_dev);
		if (unlikely(_errno < 0) && netif_msg_drv(priv))
			netdev_err(net_dev,
					   "mac_dev->change_promisc() = %d\n",
					   _errno);
	}

	_errno = priv->mac_dev->set_multi(net_dev);
	if (unlikely(_errno < 0) && netif_msg_drv(priv))
		netdev_err(net_dev, "mac_dev->set_multi() = %d\n", _errno);
}

#if defined(CONFIG_FSL_DPAA_1588) || defined(CONFIG_FSL_DPAA_TS)
u64 dpa_get_timestamp_ns(const struct dpa_priv_s *priv, enum port_type rx_tx,
			const void *data)
{
	u64 *ts, ns;

	ts = FM_PORT_GetBufferTimeStamp(
		fm_port_get_handle(priv->mac_dev->port_dev[rx_tx]), data);

	if (!ts || *ts == 0)
		return 0;

	/* multiple DPA_PTP_NOMINAL_FREQ_PERIOD_NS for case of non power of 2 */
	ns = *ts << DPA_PTP_NOMINAL_FREQ_PERIOD_SHIFT;

	return ns;
}
#endif
#ifdef CONFIG_FSL_DPAA_TS
int dpa_get_ts(const struct dpa_priv_s *priv, enum port_type rx_tx,
	struct skb_shared_hwtstamps *shhwtstamps, const void *data)
{
	u64 ns;

	ns = dpa_get_timestamp_ns(priv, rx_tx, data);

	if (ns == 0)
		return -EINVAL;

	memset(shhwtstamps, 0, sizeof(*shhwtstamps));
	shhwtstamps->hwtstamp = ns_to_ktime(ns);

	return 0;
}

static void dpa_ts_tx_enable(struct net_device *dev)
{
	struct dpa_priv_s *priv = netdev_priv(dev);
	struct mac_device *mac_dev = priv->mac_dev;

	if (mac_dev->fm_rtc_enable)
		mac_dev->fm_rtc_enable(dev);
	if (mac_dev->ptp_enable)
		mac_dev->ptp_enable(mac_dev);

	priv->ts_tx_en = TRUE;
}

static void dpa_ts_tx_disable(struct net_device *dev)
{
	struct dpa_priv_s *priv = netdev_priv(dev);

#if 0
/*
 * the RTC might be needed by the Rx Ts, cannot disable here
 * no separate ptp_disable API for Rx/Tx, cannot disable here
 */
	struct mac_device *mac_dev = priv->mac_dev;

	if (mac_dev->fm_rtc_disable)
		mac_dev->fm_rtc_disable(dev);

	if (mac_dev->ptp_disable)
		mac_dev->ptp_disable(mac_dev);
#endif

	priv->ts_tx_en = FALSE;
}

static void dpa_ts_rx_enable(struct net_device *dev)
{
	struct dpa_priv_s *priv = netdev_priv(dev);
	struct mac_device *mac_dev = priv->mac_dev;

	if (mac_dev->fm_rtc_enable)
		mac_dev->fm_rtc_enable(dev);
	if (mac_dev->ptp_enable)
		mac_dev->ptp_enable(mac_dev);

	priv->ts_rx_en = TRUE;
}

static void dpa_ts_rx_disable(struct net_device *dev)
{
	struct dpa_priv_s *priv = netdev_priv(dev);

#if 0
/*
 * the RTC might be needed by the Tx Ts, cannot disable here
 * no separate ptp_disable API for Rx/Tx, cannot disable here
 */
	struct mac_device *mac_dev = priv->mac_dev;

	if (mac_dev->fm_rtc_disable)
		mac_dev->fm_rtc_disable(dev);

	if (mac_dev->ptp_disable)
		mac_dev->ptp_disable(mac_dev);
#endif

	priv->ts_rx_en = FALSE;
}

static int dpa_ts_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct hwtstamp_config config;

	if (copy_from_user(&config, rq->ifr_data, sizeof(config)))
		return -EFAULT;

	switch (config.tx_type) {
	case HWTSTAMP_TX_OFF:
		dpa_ts_tx_disable(dev);
		break;
	case HWTSTAMP_TX_ON:
		dpa_ts_tx_enable(dev);
		break;
	default:
		return -ERANGE;
	}

	if (config.rx_filter == HWTSTAMP_FILTER_NONE)
		dpa_ts_rx_disable(dev);
	else {
		dpa_ts_rx_enable(dev);
		/* TS is set for all frame types, not only those requested */
		config.rx_filter = HWTSTAMP_FILTER_ALL;
	}

	return copy_to_user(rq->ifr_data, &config, sizeof(config)) ?
			-EFAULT : 0;
}
#endif /* CONFIG_FSL_DPAA_TS */

static int dpa_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
#ifdef CONFIG_FSL_DPAA_1588
	struct dpa_priv_s *priv = netdev_priv(dev);
#endif
	int ret = 0;

/* at least one timestamping feature must be enabled to proceed */
#if defined(CONFIG_FSL_DPAA_1588) || defined(CONFIG_FSL_DPAA_TS)
	if (!netif_running(dev))
#endif
		return -EINVAL;

#ifdef CONFIG_FSL_DPAA_TS
	if (cmd == SIOCSHWTSTAMP)
		return dpa_ts_ioctl(dev, rq, cmd);
#endif /* CONFIG_FSL_DPAA_TS */

#ifdef CONFIG_FSL_DPAA_1588
	if ((cmd >= PTP_ENBL_TXTS_IOCTL) && (cmd <= PTP_CLEANUP_TS)) {
		if (priv->tsu && priv->tsu->valid)
			ret = dpa_ioctl_1588(dev, rq, cmd);
		else
			ret = -ENODEV;
	}
#endif

	return ret;
}

#ifndef CONFIG_FSL_DPAA_ETH_SG_SUPPORT
/*
 * When we put the buffer into the pool, we purposefully added
 * some padding to the address so that the buffers wouldn't all
 * be page-aligned. But the skb has been reset to a default state,
 * so it is pointing up to DPAA_ETH_MAX_PAD - L1_CACHE_BYTES bytes
 * before the actual data. We subtract skb->head from the fd addr,
 * and then mask off the translated part to get the actual distance.
 */
static int dpa_process_one(struct dpa_percpu_priv_s *percpu_priv,
		struct sk_buff *skb, struct dpa_bp *bp, const struct qm_fd *fd)
{
	dma_addr_t fd_addr = qm_fd_addr(fd);
	unsigned long skb_addr = virt_to_phys(skb->head);
	u32 pad = fd_addr - skb_addr;
	unsigned int data_start;

	(*percpu_priv->dpa_bp_count)--;

	/*
	 * The skb is currently pointed at head + headroom. The packet
	 * starts at skb->head + pad + fd offset.
	 */
	data_start = pad + dpa_fd_offset(fd) - skb_headroom(skb);
	skb_put(skb, dpa_fd_length(fd) + data_start);
	skb_pull(skb, data_start);

	return 0;
}
#endif

/*
 * Checks whether the checksum field in Parse Results array is valid
 * (equals 0xFFFF) and increments the .cse counter otherwise
 */
static inline void
dpa_csum_validation(const struct dpa_priv_s	*priv,
		struct dpa_percpu_priv_s *percpu_priv,
		const struct qm_fd *fd)
{
	dma_addr_t addr = qm_fd_addr(fd);
	struct dpa_bp *dpa_bp = priv->dpa_bp;
	void *frm = phys_to_virt(addr);
	t_FmPrsResult *parse_result;

	if (unlikely(!frm))
		return;

	dma_unmap_single(dpa_bp->dev, addr, dpa_bp->size, DMA_BIDIRECTIONAL);

	parse_result = (t_FmPrsResult *)(frm + DPA_RX_PRIV_DATA_SIZE);

	if (parse_result->cksum != DPA_CSUM_VALID)
		percpu_priv->rx_errors.cse++;
}

static void _dpa_rx_error(struct net_device *net_dev,
		const struct dpa_priv_s	*priv,
		struct dpa_percpu_priv_s *percpu_priv,
		const struct qm_fd *fd,
		u32 fqid)
{
	/*
	 * limit common, possibly innocuous Rx FIFO Overflow errors'
	 * interference with zero-loss convergence benchmark results.
	 */
	if (likely(fd->status & FM_FD_STAT_ERR_PHYSICAL))
		pr_warn_once("fsl-dpa: non-zero error counters " \
			"in fman statistics (sysfs)\n");
	else
		if (netif_msg_hw(priv) && net_ratelimit())
			netdev_err(net_dev, "Err FD status = 0x%08x\n",
					fd->status & FM_FD_STAT_ERRORS);

	if (dpaa_eth_hooks.rx_error &&
		dpaa_eth_hooks.rx_error(net_dev, fd, fqid) == DPAA_ETH_STOLEN)
		/* it's up to the hook to perform resource cleanup */
		return;

	percpu_priv->stats.rx_errors++;

	if (fd->status & FM_PORT_FRM_ERR_DMA)
		percpu_priv->rx_errors.dme++;
	if (fd->status & FM_PORT_FRM_ERR_PHYSICAL)
		percpu_priv->rx_errors.fpe++;
	if (fd->status & FM_PORT_FRM_ERR_SIZE)
		percpu_priv->rx_errors.fse++;
	if (fd->status & FM_PORT_FRM_ERR_PRS_HDR_ERR)
		percpu_priv->rx_errors.phe++;
	if (fd->status & FM_FD_STAT_L4CV)
		dpa_csum_validation(priv, percpu_priv, fd);

	dpa_fd_release(net_dev, fd);
}

static void _dpa_tx_error(struct net_device		*net_dev,
			  const struct dpa_priv_s	*priv,
			  struct dpa_percpu_priv_s	*percpu_priv,
			  const struct qm_fd		*fd,
			  u32				 fqid)
{
	struct sk_buff *skb;

	if (netif_msg_hw(priv) && net_ratelimit())
		netdev_warn(net_dev, "FD status = 0x%08x\n",
				fd->status & FM_FD_STAT_ERRORS);

	if (dpaa_eth_hooks.tx_error &&
		dpaa_eth_hooks.tx_error(net_dev, fd, fqid) == DPAA_ETH_STOLEN)
		/* now the hook must ensure proper cleanup */
		return;

	percpu_priv->stats.tx_errors++;

	skb = _dpa_cleanup_tx_fd(priv, fd);
	dev_kfree_skb(skb);
}

/*
 * Helper function to factor out frame validation logic on all Rx paths. Its
 * purpose is to extract from the Parse Results structure information about
 * the integrity of the frame, its checksum, the length of the parsed headers
 * and whether the frame is suitable for GRO.
 *
 * Assumes no parser errors, since any error frame is dropped before this
 * function is called.
 *
 * @skb		will have its ip_summed field overwritten;
 * @use_gro	will only be written with 0, if the frame is definitely not
 *		GRO-able; otherwise, it will be left unchanged;
 * @hdr_size	will be written with a safe value, at least the size of the
 *		headers' length.
 */
void __hot _dpa_process_parse_results(const t_FmPrsResult *parse_results,
				      const struct qm_fd *fd,
				      struct sk_buff *skb, int *use_gro)
{
	if (fd->status & FM_FD_STAT_L4CV) {
		/*
		 * The parser has run and performed L4 checksum validation.
		 * We know there were no parser errors (and implicitly no
		 * L4 csum error), otherwise we wouldn't be here.
		 */
		skb->ip_summed = CHECKSUM_UNNECESSARY;

		/*
		 * Don't go through GRO for certain types of traffic that
		 * we know are not GRO-able, such as dgram-based protocols.
		 * In the worst-case scenarios, such as small-pkt terminating
		 * UDP, the extra GRO processing would be overkill.
		 *
		 * The only protocol the Parser supports that is also GRO-able
		 * is currently TCP.
		 */
		if (!fm_l4_frame_is_tcp(parse_results))
			*use_gro = 0;

		return;
	}

	/*
	 * We're here because either the parser didn't run or the L4 checksum
	 * was not verified. This may include the case of a UDP frame with
	 * checksum zero or an L4 proto other than TCP/UDP
	 */
	skb->ip_summed = CHECKSUM_NONE;

	/* Bypass GRO for unknown traffic or if no PCDs are applied */
	*use_gro = 0;
}

#ifndef CONFIG_FSL_DPAA_ETH_SG_SUPPORT
void __hot _dpa_rx(struct net_device *net_dev,
		const struct dpa_priv_s *priv,
		struct dpa_percpu_priv_s *percpu_priv,
		const struct qm_fd *fd,
		u32 fqid)
{
	struct dpa_bp *dpa_bp;
	struct sk_buff *skb;
	struct sk_buff **skbh;
	dma_addr_t addr = qm_fd_addr(fd);
	u32 fd_status = fd->status;
	unsigned int skb_len;
	t_FmPrsResult *parse_result;
	int use_gro = net_dev->features & NETIF_F_GRO;

	skbh = (struct sk_buff **)phys_to_virt(addr);

	if (unlikely(fd_status & FM_FD_STAT_ERRORS) != 0) {
		if (netif_msg_hw(priv) && net_ratelimit())
			netdev_warn(net_dev, "FD status = 0x%08x\n",
					fd->status & FM_FD_STAT_ERRORS);

		percpu_priv->stats.rx_errors++;

		goto _return_dpa_fd_release;
	}

	if (unlikely(fd->format != qm_fd_contig)) {
		percpu_priv->stats.rx_dropped++;
		if (netif_msg_rx_status(priv) && net_ratelimit())
			netdev_warn(net_dev, "Dropping a SG frame\n");
		goto _return_dpa_fd_release;
	}

	dpa_bp = dpa_bpid2pool(fd->bpid);

	dma_unmap_single(dpa_bp->dev, addr, dpa_bp->size, DMA_BIDIRECTIONAL);
	/* Execute the Rx processing hook, if it exists. */
	if (dpaa_eth_hooks.rx_default && dpaa_eth_hooks.rx_default((void *)fd,
		net_dev, fqid) == DPAA_ETH_STOLEN)
		/* won't count the rx bytes in */
		goto skb_stolen;

	skb = *skbh;
	prefetch(skb);

	/* Fill the SKB */
	dpa_process_one(percpu_priv, skb, dpa_bp, fd);

	prefetch(skb_shinfo(skb));

#ifdef CONFIG_FSL_DPAA_1588
	if (priv->tsu && priv->tsu->valid && priv->tsu->hwts_rx_en_ioctl)
		dpa_ptp_store_rxstamp(priv, skb, (void *)skbh);
#endif

	skb->protocol = eth_type_trans(skb, net_dev);

	if (unlikely(dpa_check_rx_mtu(skb, net_dev->mtu))) {
		percpu_priv->stats.rx_dropped++;
		goto drop_large_frame;
	}


	skb_len = skb->len;

	/* Validate the skb csum and figure out whether GRO is appropriate */
	parse_result = (t_FmPrsResult *)((u8 *)skbh + DPA_RX_PRIV_DATA_SIZE);
	_dpa_process_parse_results(parse_result, fd, skb, &use_gro);

#ifdef CONFIG_FSL_DPAA_TS
	if (priv->ts_rx_en)
		dpa_get_ts(priv, RX, skb_hwtstamps(skb), (void *)skbh);
#endif /* CONFIG_FSL_DPAA_TS */

	if (use_gro) {
		gro_result_t gro_result;

		gro_result = napi_gro_receive(&percpu_priv->napi, skb);
		if (unlikely(gro_result == GRO_DROP)) {
			percpu_priv->stats.rx_dropped++;
			goto packet_dropped;
		}
	} else if (unlikely(netif_receive_skb(skb) == NET_RX_DROP)) {
		percpu_priv->stats.rx_dropped++;
		goto packet_dropped;
	}

	percpu_priv->stats.rx_packets++;
	percpu_priv->stats.rx_bytes += skb_len;

packet_dropped:
skb_stolen:
	return;

drop_large_frame:
	dev_kfree_skb(skb);
	return;

_return_dpa_fd_release:
	dpa_fd_release(net_dev, fd);
}
#endif /* CONFIG_FSL_DPAA_ETH_SG_SUPPORT */

static void dpaa_eth_napi_disable(struct dpa_priv_s *priv)
{
	struct dpa_percpu_priv_s *percpu_priv;
	int i;

	if (priv->shared)
		return;

	for_each_online_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);
		napi_disable(&percpu_priv->napi);
	}
}

static void dpaa_eth_napi_enable(struct dpa_priv_s *priv)
{
	struct dpa_percpu_priv_s *percpu_priv;
	int i;

	if (priv->shared)
		return;

	for_each_online_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);
		napi_enable(&percpu_priv->napi);
	}
}

static int dpaa_eth_poll(struct napi_struct *napi, int budget)
{
	int cleaned = qman_poll_dqrr(budget);

	if (cleaned < budget) {
		int tmp;
		napi_complete(napi);
		tmp = qman_irqsource_add(QM_PIRQ_DQRI);
		BUG_ON(tmp);
	}

	return cleaned;
}

static void __hot _dpa_tx_conf(struct net_device	*net_dev,
			  const struct dpa_priv_s	*priv,
			  struct dpa_percpu_priv_s	*percpu_priv,
			  const struct qm_fd		*fd,
			  u32				 fqid)
{
	struct sk_buff	*skb;

	/* do we need the timestamp for the error frames? */

	if (unlikely(fd->status & FM_FD_STAT_ERRORS) != 0) {
		if (netif_msg_hw(priv) && net_ratelimit())
			netdev_warn(net_dev, "FD status = 0x%08x\n",
					fd->status & FM_FD_STAT_ERRORS);

		percpu_priv->stats.tx_errors++;
	}

	/* hopefully we need not get the timestamp before the hook */

	if (dpaa_eth_hooks.tx_confirm && dpaa_eth_hooks.tx_confirm(net_dev,
		fd, fqid) == DPAA_ETH_STOLEN)
		/* it's the hook that must now perform cleanup */
		return;

	/* This might not perfectly reflect the reality, if the core dequeueing
	 * the Tx confirmation is different from the one that did the enqueue,
	 * but at least it'll show up in the total count. */
	percpu_priv->tx_confirm++;

	skb = _dpa_cleanup_tx_fd(priv, fd);

	dev_kfree_skb(skb);
}

static struct dpa_bp *dpa_size2pool(struct dpa_priv_s *priv, size_t size)
{
	int i;

	for (i = 0; i < priv->bp_count; i++)
		if ((size + priv->tx_headroom) <= priv->dpa_bp[i].size)
			return dpa_bpid2pool(priv->dpa_bp[i].bpid);
	return ERR_PTR(-ENODEV);
}

static void dpa_set_buffer_layout(struct dpa_priv_s *priv, struct fm_port *port,
				  struct dpa_buffer_layout_s *layout, int type)
{
	struct fm_port_params params;

	layout->priv_data_size = (type == RX ?
			DPA_RX_PRIV_DATA_SIZE : DPA_TX_PRIV_DATA_SIZE);
	layout->parse_results = true;
	layout->hash_results = true;
#if defined(CONFIG_FSL_DPAA_1588) || defined(CONFIG_FSL_DPAA_TS)
	layout->time_stamp = true;
#endif
	fm_port_get_buff_layout_ext_params(port, &params);
	layout->manip_extra_space = params.manip_extra_space;
	layout->data_align = params.data_align;
}

/**
 * Turn on HW checksum computation for this outgoing frame.
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
	struct sk_buff *skb, struct qm_fd *fd, char *parse_results)
{
	t_FmPrsResult *parse_result;
	struct iphdr *iph;
	struct ipv6hdr *ipv6h = NULL;
	int l4_proto;
	int ethertype = ntohs(skb->protocol);
	int retval = 0;

	if (!priv->mac_dev || skb->ip_summed != CHECKSUM_PARTIAL)
		return 0;

	/* Note: L3 csum seems to be already computed in sw, but we can't choose
	 * L4 alone from the FM configuration anyway. */

	/* Fill in some fields of the Parse Results array, so the FMan
	 * can find them as if they came from the FMan Parser. */
	parse_result = (t_FmPrsResult *)parse_results;

	/* If we're dealing with VLAN, get the real Ethernet type */
	if (ethertype == ETH_P_8021Q) {
		/* We can't always assume the MAC header is set correctly
		 * by the stack, so reset to beginning of skb->data */
		skb_reset_mac_header(skb);
		ethertype = ntohs(vlan_eth_hdr(skb)->h_vlan_encapsulated_proto);
	}

	/* Fill in the relevant L3 parse result fields
	 * and read the L4 protocol type */
	switch (ethertype) {
	case ETH_P_IP:
		parse_result->l3r = FM_L3_PARSE_RESULT_IPV4;
		iph = ip_hdr(skb);
		BUG_ON(iph == NULL);
		l4_proto = ntohs(iph->protocol);
		break;
	case ETH_P_IPV6:
		parse_result->l3r = FM_L3_PARSE_RESULT_IPV6;
		ipv6h = ipv6_hdr(skb);
		BUG_ON(ipv6h == NULL);
		l4_proto = ntohs(ipv6h->nexthdr);
		break;
	default:
		/* We shouldn't even be here */
		if (netif_msg_tx_err(priv) && net_ratelimit())
			netdev_alert(priv->net_dev, "Can't compute HW csum "
				"for L3 proto 0x%x\n", ntohs(skb->protocol));
		retval = -EIO;
		goto return_error;
	}

	/* Fill in the relevant L4 parse result fields */
	switch (l4_proto) {
	case IPPROTO_UDP:
		parse_result->l4r = FM_L4_PARSE_RESULT_UDP;
		break;
	case IPPROTO_TCP:
		parse_result->l4r = FM_L4_PARSE_RESULT_TCP;
		break;
	default:
		/* This can as well be a BUG() */
		if (netif_msg_tx_err(priv) && net_ratelimit())
			netdev_alert(priv->net_dev, "Can't compute HW csum "
				"for L4 proto 0x%x\n", l4_proto);
		retval = -EIO;
		goto return_error;
	}

	/* At index 0 is IPOffset_1 as defined in the Parse Results */
	parse_result->ip_off[0] = skb_network_offset(skb);
	parse_result->l4_off = skb_transport_offset(skb);

	/* Enable L3 (and L4, if TCP or UDP) HW checksum. */
	fd->cmd |= FM_FD_CMD_RPD | FM_FD_CMD_DTC;

	/*
	 * On P1023 and similar platforms fd->cmd interpretation could
	 * be disabled by setting CONTEXT_A bit ICMD; currently this bit
	 * is not set so we do not need to check; in the future, if/when
	 * using context_a we need to check this bit
	 */

return_error:
	return retval;
}

static int __hot dpa_shared_tx(struct sk_buff *skb, struct net_device *net_dev)
{
	struct dpa_bp *dpa_bp;
	struct bm_buffer bmb;
	struct dpa_percpu_priv_s *percpu_priv;
	struct dpa_priv_s *priv;
	struct qm_fd fd;
	int queue_mapping;
	int err;
	void *dpa_bp_vaddr;
	t_FmPrsResult parse_results;

	priv = netdev_priv(net_dev);
	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());

	memset(&fd, 0, sizeof(fd));
	fd.format = qm_fd_contig;

	queue_mapping = smp_processor_id();

	dpa_bp = dpa_size2pool(priv, skb_headlen(skb));
	if (unlikely(IS_ERR(dpa_bp))) {
		percpu_priv->stats.tx_errors++;
		err = PTR_ERR(dpa_bp);
		goto bpools_too_small_error;
	}

	err = bman_acquire(dpa_bp->pool, &bmb, 1, 0);
	if (unlikely(err <= 0)) {
		percpu_priv->stats.tx_errors++;
		if (err == 0)
			err = -ENOMEM;
		goto buf_acquire_failed;
	}
	fd.bpid = dpa_bp->bpid;

	fd.length20 = skb_headlen(skb);
	fd.addr_hi = bmb.hi;
	fd.addr_lo = bmb.lo;
	fd.offset = priv->tx_headroom;

	/*
	 * The virtual address of the buffer pool is expected to be NULL
	 * in scenarios like MAC-less or Shared-MAC between Linux and
	 * USDPAA. In this case the buffers are dynamically mapped/unmapped.
	 */
	if (dpa_bp->vaddr) {
		dpa_bp_vaddr = dpa_phys2virt(dpa_bp, bm_buf_addr(&bmb));

		/* Copy the packet payload */
		skb_copy_from_linear_data(skb,
		                          dpa_bp_vaddr + dpa_fd_offset(&fd),
		                          dpa_fd_length(&fd));

		/* Enable L3/L4 hardware checksum computation, if applicable */
		err = dpa_enable_tx_csum(priv, skb, &fd,
					 dpa_bp_vaddr + DPA_TX_PRIV_DATA_SIZE);
	} else {
		err = dpa_enable_tx_csum(priv, skb, &fd,
					 (char *)&parse_results);

		copy_to_unmapped_area(bm_buf_addr(&bmb) + DPA_TX_PRIV_DATA_SIZE,
				&parse_results,
				DPA_PARSE_RESULTS_SIZE);

		copy_to_unmapped_area(bm_buf_addr(&bmb) + dpa_fd_offset(&fd),
				skb->data,
				dpa_fd_length(&fd));
	}

	if (unlikely(err < 0)) {
		if (netif_msg_tx_err(priv) && net_ratelimit())
			netdev_err(net_dev, "Tx HW csum error: %d\n", err);
		percpu_priv->stats.tx_errors++;
		goto l3_l4_csum_failed;
	}

	err = dpa_xmit(priv, &percpu_priv->stats, queue_mapping, &fd);

l3_l4_csum_failed:
bpools_too_small_error:
buf_acquire_failed:
	/* We're done with the skb */
	dev_kfree_skb(skb);

	return NETDEV_TX_OK;
}

#ifndef CONFIG_FSL_DPAA_ETH_SG_SUPPORT
static int skb_to_sg_fd(struct dpa_priv_s *priv,
		struct sk_buff *skb, struct qm_fd *fd)
{
	struct dpa_bp *dpa_bp = priv->dpa_bp;
	void *vaddr;
	dma_addr_t paddr;
	struct sk_buff **skbh;
	struct qm_sg_entry *sg_entry;
	struct net_device *net_dev = priv->net_dev;
	int err;

	/* Allocate the first buffer in the FD (used for storing S/G table) */
	vaddr = kmalloc(sgt_buffer_size(priv), GFP_ATOMIC);
	if (unlikely(vaddr == NULL)) {
		if (netif_msg_tx_err(priv) && net_ratelimit())
			netdev_err(net_dev, "Memory allocation failed\n");
		return -ENOMEM;
	}
	/* Store skb backpointer at the beginning of the buffer */
	skbh = (struct sk_buff **)vaddr;
	*skbh = skb;

	/* Fill in FD */
	fd->format = qm_fd_sg;
	fd->offset = priv->tx_headroom;
	fd->length20 = skb->len;

	/* Enable hardware checksum computation */
	err = dpa_enable_tx_csum(priv, skb, fd,
		(char *)vaddr + DPA_TX_PRIV_DATA_SIZE);
	if (unlikely(err < 0)) {
		if (netif_msg_tx_err(priv) && net_ratelimit())
			netdev_err(net_dev, "HW csum error: %d\n", err);
		kfree(vaddr);
		return err;
	}

	/* Map the buffer and store its address in the FD */
	paddr = dma_map_single(dpa_bp->dev, vaddr, sgt_buffer_size(priv),
			       DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dpa_bp->dev, paddr))) {
		if (netif_msg_tx_err(priv) && net_ratelimit())
			netdev_err(net_dev, "DMA mapping failed\n");
		kfree(vaddr);
		return -EINVAL;
	}

	fd->addr_hi = upper_32_bits(paddr);
	fd->addr_lo = lower_32_bits(paddr);

	/* Fill in S/G entry */
	sg_entry = (struct qm_sg_entry *)(vaddr + fd->offset);

	sg_entry->extension = 0;
	sg_entry->final = 1;
	sg_entry->length = skb->len;
	/*
	 * Put the same offset in the data buffer as in the SGT (first) buffer.
	 * This is the format for S/G frames generated by FMan; the manual is
	 * not clear if same is required of Tx S/G frames, but since we know
	 * for sure we have at least tx_headroom bytes of skb headroom,
	 * lets not take any chances.
	 */
	sg_entry->offset = priv->tx_headroom;

	paddr = dma_map_single(dpa_bp->dev, skb->data - sg_entry->offset,
			       dpa_bp->size, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dpa_bp->dev, paddr))) {
		if (netif_msg_tx_err(priv) && net_ratelimit())
			netdev_err(net_dev, "DMA mapping failed\n");
		return -EINVAL;
	}
	sg_entry->addr_hi = upper_32_bits(paddr);
	sg_entry->addr_lo = lower_32_bits(paddr);

#ifdef CONFIG_FSL_DPAA_TS
	if (unlikely(priv->ts_tx_en &&
			skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)) {
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
	}
#endif /* CONFIG_FSL_DPAA_TS */

	return 0;
}

static int skb_to_contig_fd(struct dpa_priv_s *priv,
		struct dpa_percpu_priv_s *percpu_priv,
		struct sk_buff *skb, struct qm_fd *fd)
{
	struct sk_buff **skbh;
	dma_addr_t addr;
	struct dpa_bp *dpa_bp = priv->dpa_bp;
	struct net_device *net_dev = priv->net_dev;
	enum dma_data_direction dma_dir = DMA_TO_DEVICE;
	bool can_recycle = false;
	int offset, extra_offset;
	int err;

	/*
	 * We are guaranteed that we have at least tx_headroom bytes.
	 * Buffers we allocated are padded to improve cache usage. In order
	 * to increase buffer re-use, we aim to keep any such buffers the
	 * same. This means the address passed to the FM should be
	 * tx_headroom bytes before the data for forwarded frames.
	 *
	 * However, offer some flexibility in fd layout, to allow originating
	 * (termination) buffers to be also recycled when possible.
	 *
	 * First, see if the conditions needed to recycle the skb are met:
	 * - skb not cloned, not shared
	 * - buffer size is large enough to accomodate a maximum size Rx frame
	 * - buffer size does not exceed the maximum size allowed in the pool
	 *   (to avoid unbounded increase of buffer size in certain forwarding
	 *   conditions)
	 * - buffer address is 16 byte aligned, as per DPAARM
	 * - there's enough room in the buffer pool
	 */
	if (likely(skb_is_recycleable(skb, dpa_bp->size) &&
		   (skb_end_pointer(skb) - skb->head <= DPA_RECYCLE_MAX_SIZE) &&
		   (*percpu_priv->dpa_bp_count < dpa_bp->target_count))) {
		/* Compute the minimum necessary fd offset */
		offset = dpa_bp->size - skb->len - skb_tailroom(skb);

		/*
		 * And make sure the offset is no lower than the offset
		 * required by FMan
		 */
		offset = max_t(int, offset, priv->tx_headroom);

		/*
		 * We also need to align the buffer address to 16, such that
		 * Fman will be able to reuse it on Rx.
		 * Since the buffer going to FMan starts at (skb->data - offset)
		 * this is what we'll try to align. We already know that
		 * headroom is at least tx_headroom bytes long, but with
		 * the extra offset needed for alignment we may go beyond
		 * the beginning of the buffer.
		 *
		 * Also need to check that we don't go beyond the maximum
		 * offset that can be set for a contiguous FD.
		 */
		extra_offset = (unsigned long)(skb->data - offset) & 0xF;
		if (likely((offset + extra_offset) <= skb_headroom(skb) &&
			   (offset + extra_offset) <= DPA_MAX_FD_OFFSET)) {
			/* We're good to go for recycling*/
			offset += extra_offset;
			can_recycle = true;
		}
	}

#ifdef CONFIG_FSL_DPAA_TS
	if (unlikely(priv->ts_tx_en &&
			skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)) {
		/* we need the fd back to get the timestamp */
		can_recycle = false;
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
	}
#endif /* CONFIG_FSL_DPAA_TS */

	if (likely(can_recycle)) {
		/* Buffer will get recycled, setup fd accordingly */
		fd->cmd |= FM_FD_CMD_FCO;
		fd->bpid = dpa_bp->bpid;
		/*
		 * Since the buffer will get back to the Bman pool
		 * and be re-used on Rx, map it for both read and write
		 */
		dma_dir = DMA_BIDIRECTIONAL;
	} else {
		/*
		 * No recycling here, so we don't care about address alignment.
		 * Just use the smallest offset required by FMan
		 */
		offset = priv->tx_headroom;
	}

	skbh = (struct sk_buff **)(skb->data - offset);
	*skbh = skb;


	/* Enable L3/L4 hardware checksum computation.
	 *
	 * We must do this before dma_map_single(), because we may
	 * need to write into the skb. */
	err = dpa_enable_tx_csum(priv, skb, fd,
				 ((char *)skbh) + DPA_TX_PRIV_DATA_SIZE);
	if (unlikely(err < 0)) {
		if (netif_msg_tx_err(priv) && net_ratelimit())
			netdev_err(net_dev, "HW csum error: %d\n", err);
		return err;
	}

	fd->format = qm_fd_contig;
	fd->length20 = skb->len;
	fd->offset = offset;

	addr = dma_map_single(dpa_bp->dev, skbh, dpa_bp->size, dma_dir);
	if (unlikely(dma_mapping_error(dpa_bp->dev, addr))) {
		if (netif_msg_tx_err(priv)  && net_ratelimit())
			netdev_err(net_dev, "dma_map_single() failed\n");
		return -EINVAL;
	}

	fd->addr_hi = upper_32_bits(addr);
	fd->addr_lo = lower_32_bits(addr);

	return 0;
}

int __hot dpa_tx(struct sk_buff *skb, struct net_device *net_dev)
{
	struct dpa_priv_s	*priv;
	struct qm_fd		 fd;
	struct dpa_percpu_priv_s *percpu_priv;
	struct rtnl_link_stats64 *percpu_stats;
	int queue_mapping;
	int err;

	/* If there is a Tx hook, run it. */
	if (dpaa_eth_hooks.tx &&
		dpaa_eth_hooks.tx(skb, net_dev) == DPAA_ETH_STOLEN)
		/* won't update any Tx stats */
		goto done;

	priv = netdev_priv(net_dev);
	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());
	percpu_stats = &percpu_priv->stats;

	clear_fd(&fd);
	queue_mapping = dpa_get_queue_mapping(skb);

	if (unlikely(skb_headroom(skb) < priv->tx_headroom)) {
		struct sk_buff *skb_new;

		skb_new = skb_realloc_headroom(skb, priv->tx_headroom);
		if (unlikely(!skb_new)) {
			percpu_stats->tx_errors++;
			kfree_skb(skb);
			goto done;
		}
		kfree_skb(skb);
		skb = skb_new;
	}

#ifdef CONFIG_FSL_DPAA_1588
	if (priv->tsu && priv->tsu->valid && priv->tsu->hwts_tx_en_ioctl)
		fd.cmd |= FM_FD_CMD_UPD;
#endif
#ifdef CONFIG_FSL_DPAA_TS
	if (unlikely(priv->ts_tx_en &&
			skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP))
		fd.cmd |= FM_FD_CMD_UPD;
#endif /* CONFIG_FSL_DPAA_TS */

	/*
	 * We have two paths here:
	 *
	 * 1.If the skb is cloned, create a S/G frame to avoid unsharing it.
	 * The S/G table will contain only one entry, pointing to our skb
	 * data buffer.
	 * The private data area containing the skb backpointer will reside
	 * inside the first buffer, such that it won't risk being overwritten
	 * in case a second skb pointing to the same data buffer is being
	 * processed concurently.
	 * No recycling is possible in this case, as the data buffer is shared.
	 *
	 * 2.If skb is not cloned, then the private area inside it can be
	 * safely used to store the skb backpointer. Simply create a contiguous
	 * fd in this case.
	 * Recycling can happen if the right conditions are met.
	 */
	if (skb_cloned(skb) && (skb->len > DPA_SKB_COPY_MAX_SIZE))
		err = skb_to_sg_fd(priv, skb, &fd);
	else {
		/* If cloned skb, but length is below DPA_SKB_COPY_MAX_SIZE,
		 * it's more efficient to unshare it and then use the new skb */
		skb = skb_unshare(skb, GFP_ATOMIC);
		if (unlikely(!skb)) {
			percpu_stats->tx_errors++;
			goto done;
		}
		err = skb_to_contig_fd(priv, percpu_priv, skb, &fd);
	}
	if (unlikely(err < 0)) {
		percpu_stats->tx_errors++;
		goto fd_create_failed;
	}

	if (fd.cmd & FM_FD_CMD_FCO) {
		/* This skb is recycleable, and the fd generated from it
		 * has been filled in accordingly.
		 * NOTE: The recycling mechanism is fragile and dependant on
		 * upstream changes. It will be maintained for now, but plans
		 * are to remove it altoghether from the driver.
		 */
		skb_recycle(skb);
		skb = NULL;
		(*percpu_priv->dpa_bp_count)++;
		percpu_priv->tx_returned++;
	}

	if (unlikely(dpa_xmit(priv, percpu_stats, queue_mapping,
		&fd) < 0))
		goto xmit_failed;

	net_dev->trans_start = jiffies;
	goto done;

xmit_failed:
	if (fd.cmd & FM_FD_CMD_FCO) {
		(*percpu_priv->dpa_bp_count)--;
		percpu_priv->tx_returned--;
	}
fd_create_failed:
	_dpa_cleanup_tx_fd(priv, &fd);
	dev_kfree_skb(skb);

done:
	return NETDEV_TX_OK;
}
#endif /* CONFIG_FSL_DPAA_ETH_SG_SUPPORT */

/**
 * Congestion group state change notification callback.
 * Stops the device's egress queues while they are congested and
 * wakes them upon exiting congested state.
 * Also updates some CGR-related stats.
 */
static void dpaa_eth_cgscn(struct qman_portal *qm, struct qman_cgr *cgr,
	int congested)
{
	struct dpa_priv_s *priv = (struct dpa_priv_s *)container_of(cgr,
		struct dpa_priv_s, cgr_data.cgr);

	if (congested) {
		priv->cgr_data.congestion_start_jiffies = jiffies;
		netif_tx_stop_all_queues(priv->net_dev);
		priv->cgr_data.cgr_congested_count++;
	} else {
		priv->cgr_data.congested_jiffies +=
			(jiffies - priv->cgr_data.congestion_start_jiffies);
		netif_tx_wake_all_queues(priv->net_dev);
	}
}

static enum qman_cb_dqrr_result
ingress_rx_error_dqrr(struct qman_portal		*portal,
		      struct qman_fq			*fq,
		      const struct qm_dqrr_entry	*dq)
{
	struct net_device		*net_dev;
	struct dpa_priv_s		*priv;
	struct dpa_percpu_priv_s	*percpu_priv;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());

	if (dpaa_eth_napi_schedule(percpu_priv)) {
		percpu_priv->in_interrupt++;
		return qman_cb_dqrr_stop;
	}

	dpaa_eth_refill_bpools(percpu_priv);
	_dpa_rx_error(net_dev, priv, percpu_priv, &dq->fd, fq->fqid);

	return qman_cb_dqrr_consume;
}

static enum qman_cb_dqrr_result __hot
shared_rx_dqrr(struct qman_portal *portal, struct qman_fq *fq,
		const struct qm_dqrr_entry *dq)
{
	struct net_device		*net_dev;
	struct dpa_priv_s		*priv;
	struct dpa_percpu_priv_s	*percpu_priv;
	const struct qm_fd *fd = &dq->fd;
	struct dpa_bp *dpa_bp;
	struct sk_buff *skb;
	struct qm_sg_entry *sgt;
	int i;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());

	dpa_bp = dpa_bpid2pool(fd->bpid);
	BUG_ON(IS_ERR(dpa_bp));

	if (unlikely(fd->status & FM_FD_STAT_ERRORS) != 0) {
		if (netif_msg_hw(priv) && net_ratelimit())
			netdev_warn(net_dev, "FD status = 0x%08x\n",
					fd->status & FM_FD_STAT_ERRORS);

		percpu_priv->stats.rx_errors++;

		goto out;
	}

	skb = __netdev_alloc_skb(net_dev,
				 priv->tx_headroom + dpa_fd_length(fd),
				 GFP_ATOMIC);
	if (unlikely(skb == NULL)) {
		if (netif_msg_rx_err(priv) && net_ratelimit())
			netdev_err(net_dev, "Could not alloc skb\n");

		percpu_priv->stats.rx_dropped++;

		goto out;
	}

	skb_reserve(skb, priv->tx_headroom);

	if (fd->format == qm_fd_sg) {
		if (dpa_bp->vaddr) {
			sgt = dpa_phys2virt(dpa_bp,
					    qm_fd_addr(fd)) + dpa_fd_offset(fd);

			for (i = 0; i < DPA_SGT_MAX_ENTRIES; i++) {
				BUG_ON(sgt[i].extension);

				/* copy from sgt[i] */
				memcpy(skb_put(skb, sgt[i].length),
					dpa_phys2virt(dpa_bp,
							qm_sg_addr(&sgt[i]) +
							sgt[i].offset),
					sgt[i].length);
				if (sgt[i].final)
					break;
			}
		} else {
			sgt = kmalloc(DPA_SGT_MAX_ENTRIES * sizeof(*sgt),
					GFP_ATOMIC);
			if (unlikely(sgt == NULL)) {
				if (netif_msg_tx_err(priv) && net_ratelimit())
					netdev_err(net_dev,
						"Memory allocation failed\n");
				return -ENOMEM;
			}

			copy_from_unmapped_area(sgt,
					qm_fd_addr(fd) + dpa_fd_offset(fd),
					min(DPA_SGT_MAX_ENTRIES * sizeof(*sgt),
							dpa_bp->size));

			for (i = 0; i < DPA_SGT_MAX_ENTRIES; i++) {
				BUG_ON(sgt[i].extension);

				copy_from_unmapped_area(
					skb_put(skb, sgt[i].length),
					qm_sg_addr(&sgt[i]) + sgt[i].offset,
					sgt[i].length);

				if (sgt[i].final)
					break;
			}

			kfree(sgt);
		}
		goto skb_copied;
	}

	/* otherwise fd->format == qm_fd_contig */
	if (dpa_bp->vaddr) {
		/* Fill the SKB */
		memcpy(skb_put(skb, dpa_fd_length(fd)),
		       dpa_phys2virt(dpa_bp, qm_fd_addr(fd)) +
		       dpa_fd_offset(fd), dpa_fd_length(fd));
	} else {
		copy_from_unmapped_area(skb_put(skb, dpa_fd_length(fd)),
					qm_fd_addr(fd) + dpa_fd_offset(fd),
					dpa_fd_length(fd));
	}

skb_copied:
	skb->protocol = eth_type_trans(skb, net_dev);

	/* IP Reassembled frames are allowed to be larger than MTU */
	if (unlikely(dpa_check_rx_mtu(skb, net_dev->mtu) &&
		!(fd->status & FM_FD_IPR))) {
		percpu_priv->stats.rx_dropped++;
		dev_kfree_skb_any(skb);
		goto out;
	}

	if (unlikely(netif_rx(skb) != NET_RX_SUCCESS))
		percpu_priv->stats.rx_dropped++;
	else {
		percpu_priv->stats.rx_packets++;
		percpu_priv->stats.rx_bytes += dpa_fd_length(fd);
	}

out:
	if (fd->format == qm_fd_sg)
		dpa_fd_release_sg(net_dev, fd);
	else
		dpa_fd_release(net_dev, fd);

	return qman_cb_dqrr_consume;
}


static enum qman_cb_dqrr_result __hot
ingress_rx_default_dqrr(struct qman_portal		*portal,
			struct qman_fq			*fq,
			const struct qm_dqrr_entry	*dq)
{
	struct net_device		*net_dev;
	struct dpa_priv_s		*priv;
	struct dpa_percpu_priv_s	*percpu_priv;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	/* Trace the Rx fd */
	trace_dpa_rx_fd(net_dev, fq, &dq->fd);

	/* IRQ handler, non-migratable; safe to use __this_cpu_ptr here */
	percpu_priv = __this_cpu_ptr(priv->percpu_priv);

	if (unlikely(dpaa_eth_napi_schedule(percpu_priv))) {
		percpu_priv->in_interrupt++;
		return qman_cb_dqrr_stop;
	}

	/* Vale of plenty: make sure we didn't run out of buffers */
	dpaa_eth_refill_bpools(percpu_priv);
	_dpa_rx(net_dev, priv, percpu_priv, &dq->fd, fq->fqid);

	return qman_cb_dqrr_consume;
}

static enum qman_cb_dqrr_result
ingress_tx_error_dqrr(struct qman_portal		*portal,
		      struct qman_fq			*fq,
		      const struct qm_dqrr_entry	*dq)
{
	struct net_device		*net_dev;
	struct dpa_priv_s		*priv;
	struct dpa_percpu_priv_s	*percpu_priv;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());

	if (dpaa_eth_napi_schedule(percpu_priv)) {
		percpu_priv->in_interrupt++;
		return qman_cb_dqrr_stop;
	}

	_dpa_tx_error(net_dev, priv, percpu_priv, &dq->fd, fq->fqid);

	return qman_cb_dqrr_consume;
}

static enum qman_cb_dqrr_result __hot
ingress_tx_default_dqrr(struct qman_portal		*portal,
			struct qman_fq			*fq,
			const struct qm_dqrr_entry	*dq)
{
	struct net_device		*net_dev;
	struct dpa_priv_s		*priv;
	struct dpa_percpu_priv_s	*percpu_priv;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	/* Trace the fd */
	trace_dpa_tx_conf_fd(net_dev, fq, &dq->fd);

	/* Non-migratable context, safe to use __this_cpu_ptr */
	percpu_priv = __this_cpu_ptr(priv->percpu_priv);

	if (dpaa_eth_napi_schedule(percpu_priv)) {
		percpu_priv->in_interrupt++;
		return qman_cb_dqrr_stop;
	}

	_dpa_tx_conf(net_dev, priv, percpu_priv, &dq->fd, fq->fqid);

	return qman_cb_dqrr_consume;
}

static enum qman_cb_dqrr_result
shared_tx_error_dqrr(struct qman_portal                *portal,
		     struct qman_fq                    *fq,
		     const struct qm_dqrr_entry        *dq)
{
	struct net_device               *net_dev;
	struct dpa_priv_s               *priv;
	struct dpa_percpu_priv_s        *percpu_priv;
	struct dpa_bp			*dpa_bp;
	const struct qm_fd		*fd = &dq->fd;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	dpa_bp = dpa_bpid2pool(fd->bpid);
	BUG_ON(IS_ERR(dpa_bp));

	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());

	if (netif_msg_hw(priv) && net_ratelimit())
		netdev_warn(net_dev, "FD status = 0x%08x\n",
				fd->status & FM_FD_STAT_ERRORS);

	if ((fd->format == qm_fd_sg) && (!dpa_bp->vaddr))
		dpa_fd_release_sg(net_dev, fd);
	else
		dpa_fd_release(net_dev, fd);

	percpu_priv->stats.tx_errors++;

	return qman_cb_dqrr_consume;
}

static enum qman_cb_dqrr_result __hot
shared_tx_default_dqrr(struct qman_portal              *portal,
		       struct qman_fq                  *fq,
		       const struct qm_dqrr_entry      *dq)
{
	struct net_device               *net_dev;
	struct dpa_priv_s               *priv;
	struct dpa_percpu_priv_s        *percpu_priv;
	struct dpa_bp			*dpa_bp;
	const struct qm_fd		*fd = &dq->fd;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	dpa_bp = dpa_bpid2pool(fd->bpid);
	BUG_ON(IS_ERR(dpa_bp));

	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());

	if (unlikely(fd->status & FM_FD_STAT_ERRORS) != 0) {
		if (netif_msg_hw(priv) && net_ratelimit())
			netdev_warn(net_dev, "FD status = 0x%08x\n",
					fd->status & FM_FD_STAT_ERRORS);

		percpu_priv->stats.tx_errors++;
	}

	if ((fd->format == qm_fd_sg) && (!dpa_bp->vaddr))
		dpa_fd_release_sg(net_dev, fd);
	else
		dpa_fd_release(net_dev, fd);

	percpu_priv->tx_confirm++;

	return qman_cb_dqrr_consume;
}

static void count_ern(struct dpa_percpu_priv_s *percpu_priv,
		      const struct qm_mr_entry *msg)
{
	switch (msg->ern.rc & QM_MR_RC_MASK) {
	case QM_MR_RC_CGR_TAILDROP:
		percpu_priv->ern_cnt.cg_tdrop++;
		break;
	case QM_MR_RC_WRED:
		percpu_priv->ern_cnt.wred++;
		break;
	case QM_MR_RC_ERROR:
		percpu_priv->ern_cnt.err_cond++;
		break;
	case QM_MR_RC_ORPWINDOW_EARLY:
		percpu_priv->ern_cnt.early_window++;
		break;
	case QM_MR_RC_ORPWINDOW_LATE:
		percpu_priv->ern_cnt.late_window++;
		break;
	case QM_MR_RC_FQ_TAILDROP:
		percpu_priv->ern_cnt.fq_tdrop++;
		break;
	case QM_MR_RC_ORPWINDOW_RETIRED:
		percpu_priv->ern_cnt.fq_retired++;
		break;
	case QM_MR_RC_ORP_ZERO:
		percpu_priv->ern_cnt.orp_zero++;
		break;
	}
}

static void shared_ern(struct qman_portal	*portal,
		       struct qman_fq		*fq,
		       const struct qm_mr_entry	*msg)
{
	struct net_device *net_dev;
	const struct dpa_priv_s	*priv;
	struct dpa_percpu_priv_s *percpu_priv;
	struct dpa_fq *dpa_fq = (struct dpa_fq *)fq;

	net_dev = dpa_fq->net_dev;
	priv = netdev_priv(net_dev);
	percpu_priv = per_cpu_ptr(priv->percpu_priv,  smp_processor_id());

	dpa_fd_release(net_dev, &msg->ern.fd);

	percpu_priv->stats.tx_dropped++;
	percpu_priv->stats.tx_fifo_errors++;
	count_ern(percpu_priv, msg);
}

static void egress_ern(struct qman_portal	*portal,
		       struct qman_fq		*fq,
		       const struct qm_mr_entry	*msg)
{
	struct net_device	*net_dev;
	const struct dpa_priv_s	*priv;
	struct sk_buff *skb;
	struct dpa_percpu_priv_s	*percpu_priv;
	struct qm_fd fd = msg->ern.fd;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);
	/* Non-migratable context, safe to use __this_cpu_ptr */
	percpu_priv = __this_cpu_ptr(priv->percpu_priv);

	percpu_priv->stats.tx_dropped++;
	percpu_priv->stats.tx_fifo_errors++;
	count_ern(percpu_priv, msg);

	/*
	 * If we intended this buffer to go into the pool
	 * when the FM was done, we need to put it in
	 * manually.
	 */
	if (msg->ern.fd.cmd & FM_FD_CMD_FCO) {
		dpa_fd_release(net_dev, &fd);
		return;
	}

	skb = _dpa_cleanup_tx_fd(priv, &fd);
	dev_kfree_skb_any(skb);
}

static const struct qman_fq rx_shared_fq = {
	.cb = { .dqrr = shared_rx_dqrr }
};
static const struct qman_fq rx_private_defq = {
	.cb = { .dqrr = ingress_rx_default_dqrr }
};
static const struct qman_fq rx_private_errq = {
	.cb = { .dqrr = ingress_rx_error_dqrr }
};
static const struct qman_fq tx_private_defq = {
	.cb = { .dqrr = ingress_tx_default_dqrr }
};
static const struct qman_fq tx_private_errq = {
	.cb = { .dqrr = ingress_tx_error_dqrr }
};
static const struct qman_fq tx_shared_defq = {
	.cb = { .dqrr = shared_tx_default_dqrr }
};
static const struct qman_fq tx_shared_errq = {
	.cb = { .dqrr = shared_tx_error_dqrr }
};
static const struct qman_fq private_egress_fq = {
	.cb = { .ern = egress_ern }
};
static const struct qman_fq shared_egress_fq = {
	.cb = { .ern = shared_ern }
};

#ifdef CONFIG_FSL_DPAA_ETH_UNIT_TESTS
static bool tx_unit_test_passed = true;

static void tx_unit_test_ern(struct qman_portal	*portal,
		       struct qman_fq		*fq,
		       const struct qm_mr_entry	*msg)
{
	struct net_device *net_dev;
	struct dpa_priv_s *priv;
	struct sk_buff **skbh;
	struct sk_buff *skb;
	const struct qm_fd *fd;
	dma_addr_t addr;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	tx_unit_test_passed = false;

	fd = &msg->ern.fd;

	addr = qm_fd_addr(fd);

	skbh = (struct sk_buff **)phys_to_virt(addr);
	skb = *skbh;

	if (!skb || !is_kernel_addr((unsigned long)skb))
		panic("Corrupt skb in ERN!\n");

	kfree_skb(skb);
}

static unsigned char *tx_unit_skb_head;
static unsigned char *tx_unit_skb_end;
static int tx_unit_tested;

static enum qman_cb_dqrr_result tx_unit_test_dqrr(
		struct qman_portal *portal,
		struct qman_fq *fq,
		const struct qm_dqrr_entry *dq)
{
	struct net_device *net_dev;
	struct dpa_priv_s *priv;
	struct sk_buff **skbh;
	struct sk_buff *skb;
	const struct qm_fd *fd;
	dma_addr_t addr;
	unsigned char *startaddr;
	struct dpa_percpu_priv_s *percpu_priv;

	tx_unit_test_passed = false;

	tx_unit_tested++;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());

	fd = &dq->fd;

	addr = qm_fd_addr(fd);

	skbh = (struct sk_buff **)phys_to_virt(addr);
	startaddr = (unsigned char *)skbh;
	skb = *skbh;

	if (!skb || !is_kernel_addr((unsigned long)skb))
		panic("Invalid skb address in TX Unit Test FD\n");

	/* Make sure we're dealing with the same skb */
	if (skb->head != tx_unit_skb_head
			|| skb_end_pointer(skb) != tx_unit_skb_end)
		goto out;

	/*
	 * If we recycled, then there must be enough room between fd.addr
	 * and skb->end for a new RX buffer
	 */
	if (fd->cmd & FM_FD_CMD_FCO) {
		size_t bufsize = skb_end_pointer(skb) - startaddr;

		if (bufsize < dpa_get_max_frm())
			goto out;
	} else {
		/*
		 * If we didn't recycle, but the buffer was big enough,
		 * increment the counter to put it back
		 */
		if (skb_end_pointer(skb) - skb->head >=
			dpa_get_max_frm())
			(*percpu_priv->dpa_bp_count)++;

		/* If we didn't recycle, the data pointer should be good */
		if (skb->data != startaddr + dpa_fd_offset(fd))
			goto out;
	}

	tx_unit_test_passed = true;
out:
	/* The skb is no longer needed, and belongs to us */
	kfree_skb(skb);

	return qman_cb_dqrr_consume;
}

static const struct qman_fq tx_unit_test_fq = {
	.cb = { .dqrr = tx_unit_test_dqrr, .ern = tx_unit_test_ern }
};

static struct dpa_fq unit_fq;
#ifdef CONFIG_FSL_DPAA_TX_RECYCLE
static struct dpa_fq unit_recycle_fq;
#endif
static bool tx_unit_test_ran; /* Starts as false */

static int dpa_tx_unit_test(struct net_device *net_dev)
{
	/* Create a new FQ */
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	struct qman_fq *oldq;
	int size, headroom;
	struct dpa_percpu_priv_s *percpu_priv;
	cpumask_var_t old_cpumask;
	int test_count = 0;
	int err = 0;
	int tests_failed = 0;
	const cpumask_t *cpus = qman_affine_cpus();
#ifdef CONFIG_FSL_DPAA_TX_RECYCLE
	struct qman_fq *oldrecycleq;
#endif

	if (!alloc_cpumask_var(&old_cpumask, GFP_KERNEL)) {
		pr_err("UNIT test cpumask allocation failed\n");
		return -ENOMEM;
	}

	cpumask_copy(old_cpumask, tsk_cpus_allowed(current));
	set_cpus_allowed_ptr(current, cpus);
	/* disable bottom halves */
	local_bh_disable();

	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());

	qman_irqsource_remove(QM_PIRQ_DQRI);
	unit_fq.net_dev = net_dev;
	unit_fq.fq_base = tx_unit_test_fq;

	/* Save old queue */
	oldq = priv->egress_fqs[smp_processor_id()];

	err = qman_create_fq(0, QMAN_FQ_FLAG_DYNAMIC_FQID, &unit_fq.fq_base);

	if (err < 0) {
		pr_err("UNIT test FQ create failed: %d\n", err);
		goto fq_create_fail;
	}

	err = qman_init_fq(&unit_fq.fq_base,
			QMAN_INITFQ_FLAG_SCHED | QMAN_INITFQ_FLAG_LOCAL, NULL);
	if (err < 0) {
		pr_err("UNIT test FQ init failed: %d\n", err);
		goto fq_init_fail;
	}

	/* Replace queue 0 with this queue */
	priv->egress_fqs[smp_processor_id()] = &unit_fq.fq_base;

#ifdef CONFIG_FSL_DPAA_TX_RECYCLE
	oldrecycleq = priv->recycle_fqs[smp_processor_id()];
	unit_recycle_fq.net_dev = net_dev;
	unit_recycle_fq.fq_base = tx_unit_test_fq;

	err = qman_create_fq(0, QMAN_FQ_FLAG_DYNAMIC_FQID,
			&unit_recycle_fq.fq_base);

	if (err < 0) {
		pr_err("UNIT test Recycle FQ create failed: %d\n", err);
		goto recycle_fq_create_fail;
	}

	err = qman_init_fq(&unit_recycle_fq.fq_base,
			QMAN_INITFQ_FLAG_SCHED | QMAN_INITFQ_FLAG_LOCAL, NULL);
	if (err < 0) {
		pr_err("UNIT test Recycle FQ init failed: %d\n", err);
		goto recycle_fq_init_fail;
	}

	priv->recycle_fqs[smp_processor_id()] = &unit_recycle_fq.fq_base;

	pr_err("TX Unit Test using FQ: %d - Recycle FQ: %d\n",
		qman_fq_fqid(&unit_fq.fq_base),
		qman_fq_fqid(&unit_recycle_fq.fq_base));
#else
	pr_err("TX Unit Test using FQ %d\n", qman_fq_fqid(&unit_fq.fq_base));
#endif

	/* Try packet sizes from 64-bytes to just above the maximum */
	for (size = 64; size <= 9600 + 128; size += 64) {
		for (headroom = priv->tx_headroom; headroom < 0x800;
		     headroom += 16) {
			int ret;
			struct sk_buff *skb;

			test_count++;

			skb = dev_alloc_skb(size + headroom);

			if (!skb) {
				pr_err("Failed to allocate skb\n");
				err = -ENOMEM;
				goto end_test;
			}

			if (skb_end_pointer(skb) - skb->head >=
					dpa_get_max_frm())
				(*percpu_priv->dpa_bp_count)--;

			skb_put(skb, size + headroom);
			skb_pull(skb, headroom);

			tx_unit_skb_head = skb->head;
			tx_unit_skb_end = skb_end_pointer(skb);

			skb_set_queue_mapping(skb, smp_processor_id());

			/* tx */
			ret = net_dev->netdev_ops->ndo_start_xmit(skb, net_dev);

			if (ret != NETDEV_TX_OK) {
				pr_err("Failed to TX with err %d\n", ret);
				err = -EIO;
				goto end_test;
			}

			/* Wait for it to arrive */
			ret = spin_event_timeout(qman_poll_dqrr(1) != 0,
					100000, 1);

			if (!ret) {
				pr_err("TX Packet never arrived\n");
				/*
				 * Count the test as failed.
				 */
				tests_failed++;
			}

			/* Was it good? */
			if (tx_unit_test_passed == false) {
				pr_err("Test failed:\n");
				pr_err("size: %d pad: %d head: %p end: %p\n",
					size, headroom, tx_unit_skb_head,
					tx_unit_skb_end);
				tests_failed++;
			}
		}
	}

end_test:
	err = qman_retire_fq(&unit_fq.fq_base, NULL);
	if (unlikely(err < 0))
		pr_err("Could not retire TX Unit Test FQ (%d)\n", err);

	err = qman_oos_fq(&unit_fq.fq_base);
	if (unlikely(err < 0))
		pr_err("Could not OOS TX Unit Test FQ (%d)\n", err);

#ifdef CONFIG_FSL_DPAA_TX_RECYCLE
	err = qman_retire_fq(&unit_recycle_fq.fq_base, NULL);
	if (unlikely(err < 0))
		pr_err("Could not retire Recycle TX Unit Test FQ (%d)\n", err);

	err = qman_oos_fq(&unit_recycle_fq.fq_base);
	if (unlikely(err < 0))
		pr_err("Could not OOS Recycle TX Unit Test FQ (%d)\n", err);

recycle_fq_init_fail:
	qman_destroy_fq(&unit_recycle_fq.fq_base, 0);

recycle_fq_create_fail:
	priv->recycle_fqs[smp_processor_id()] = oldrecycleq;
#endif

fq_init_fail:
	qman_destroy_fq(&unit_fq.fq_base, 0);

fq_create_fail:
	priv->egress_fqs[smp_processor_id()] = oldq;
	local_bh_enable();
	qman_irqsource_add(QM_PIRQ_DQRI);
	tx_unit_test_ran = true;
	set_cpus_allowed_ptr(current, old_cpumask);
	free_cpumask_var(old_cpumask);

	pr_err("Tested %d/%d packets. %d failed\n", test_count, tx_unit_tested,
		tests_failed);

	if (tests_failed)
		err = -EINVAL;

	/* Reset counters */
	memset(&percpu_priv->stats, 0, sizeof(percpu_priv->stats));

	return err;
}
#endif

static int __cold dpa_start(struct net_device *net_dev)
{
	int err, i;
	struct dpa_priv_s *priv;
	struct mac_device *mac_dev;
	struct dpa_percpu_priv_s *percpu_priv;

	priv = netdev_priv(net_dev);
	mac_dev = priv->mac_dev;

	if (!mac_dev)
		goto no_mac;

	/* Seed the global buffer pool at the first ifconfig up
	 * of a private port. Update the percpu buffer counters
	 * of each private interface.
	 */
	if (!priv->shared && !default_pool_seeded) {
		default_pool->size = default_buf_size;
		dpa_make_private_pool(default_pool);
		default_pool_seeded = true;
	}
	for_each_online_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);
		if (!priv->shared && !percpu_priv->dpa_bp) {
			percpu_priv->dpa_bp = priv->dpa_bp;
			percpu_priv->dpa_bp_count =
				per_cpu_ptr(priv->dpa_bp->percpu_count, i);
		}
	}

	dpaa_eth_napi_enable(priv);

	err = mac_dev->init_phy(net_dev);
	if (err < 0) {
		if (netif_msg_ifup(priv))
			netdev_err(net_dev, "init_phy() = %d\n", err);
		goto init_phy_failed;
	}

	for_each_port_device(i, mac_dev->port_dev)
		fm_port_enable(mac_dev->port_dev[i]);

	err = priv->mac_dev->start(mac_dev);
	if (err < 0) {
		if (netif_msg_ifup(priv))
			netdev_err(net_dev, "mac_dev->start() = %d\n", err);
		goto mac_start_failed;
	}

no_mac:
	netif_tx_start_all_queues(net_dev);

	return 0;

mac_start_failed:
	for_each_port_device(i, mac_dev->port_dev)
		fm_port_disable(mac_dev->port_dev[i]);

init_phy_failed:
	dpaa_eth_napi_disable(priv);

	return err;
}

static int __cold dpa_stop(struct net_device *net_dev)
{
	int _errno, i;
	struct dpa_priv_s *priv;
	struct mac_device *mac_dev;

	priv = netdev_priv(net_dev);
	mac_dev = priv->mac_dev;

	netif_tx_stop_all_queues(net_dev);

	if (!mac_dev)
		return 0;

	_errno = mac_dev->stop(mac_dev);
	if (unlikely(_errno < 0))
		if (netif_msg_ifdown(priv))
			netdev_err(net_dev, "mac_dev->stop() = %d\n",
					_errno);

	for_each_port_device(i, mac_dev->port_dev)
		fm_port_disable(mac_dev->port_dev[i]);

	if (mac_dev->phy_dev)
		phy_disconnect(mac_dev->phy_dev);
	mac_dev->phy_dev = NULL;

	dpaa_eth_napi_disable(priv);

	return _errno;
}

static void __cold dpa_timeout(struct net_device *net_dev)
{
	const struct dpa_priv_s	*priv;
	struct dpa_percpu_priv_s *percpu_priv;

	priv = netdev_priv(net_dev);
	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());

	if (netif_msg_timer(priv))
		netdev_crit(net_dev, "Transmit timeout latency: %u ms\n",
			jiffies_to_msecs(jiffies - net_dev->trans_start));

	percpu_priv->stats.tx_errors++;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
static void dpaa_eth_poll_controller(struct net_device *net_dev)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	struct dpa_percpu_priv_s *percpu_priv =
		this_cpu_ptr(priv->percpu_priv);
	struct napi_struct napi = percpu_priv->napi;

	qman_irqsource_remove(QM_PIRQ_DQRI);
	qman_poll_dqrr(napi.weight);
	qman_irqsource_add(QM_PIRQ_DQRI);
}
#endif

static int dpa_bp_cmp(const void *dpa_bp0, const void *dpa_bp1)
{
	return ((struct dpa_bp *)dpa_bp0)->size -
			((struct dpa_bp *)dpa_bp1)->size;
}

static struct dpa_bp * __cold __must_check __attribute__((nonnull))
dpa_bp_probe(struct platform_device *_of_dev, size_t *count)
{
	int			 i, lenp, na, ns;
	struct device		*dev;
	struct device_node	*dev_node;
	const phandle		*phandle_prop;
	const uint32_t		*bpid;
	const uint32_t		*bpool_cfg;
	struct dpa_bp		*dpa_bp;
	int has_kernel_pool = 0;
	int has_shared_pool = 0;

	dev = &_of_dev->dev;

	/* The default is one, if there's no property */
	*count = 1;

	/* There are three types of buffer pool configuration:
	 * 1) No bp assignment
	 * 2) A static assignment to an empty configuration
	 * 3) A static assignment to one or more configured pools
	 *
	 * We don't support using multiple unconfigured pools.
	 */

	/* Get the buffer pools to be used */
	phandle_prop = of_get_property(dev->of_node,
					"fsl,bman-buffer-pools", &lenp);

	if (phandle_prop)
		*count = lenp / sizeof(phandle);
	else {
		if (default_pool)
			return default_pool;

		has_kernel_pool = 1;
	}

	dpa_bp = devm_kzalloc(dev, *count * sizeof(*dpa_bp), GFP_KERNEL);
	if (unlikely(dpa_bp == NULL)) {
		dev_err(dev, "devm_kzalloc() failed\n");
		return ERR_PTR(-ENOMEM);
	}

	dev_node = of_find_node_by_path("/");
	if (unlikely(dev_node == NULL)) {
		dev_err(dev, "of_find_node_by_path(/) failed\n");
		return ERR_PTR(-EINVAL);
	}

	na = of_n_addr_cells(dev_node);
	ns = of_n_size_cells(dev_node);

	for (i = 0; i < *count && phandle_prop; i++) {
		of_node_put(dev_node);
		dev_node = of_find_node_by_phandle(phandle_prop[i]);
		if (unlikely(dev_node == NULL)) {
			dev_err(dev, "of_find_node_by_phandle() failed\n");
			return ERR_PTR(-EFAULT);
		}

		if (unlikely(!of_device_is_compatible(dev_node, "fsl,bpool"))) {
			dev_err(dev,
				"!of_device_is_compatible(%s, fsl,bpool)\n",
				dev_node->full_name);
			dpa_bp = ERR_PTR(-EINVAL);
			goto _return_of_node_put;
		}

		bpid = of_get_property(dev_node, "fsl,bpid", &lenp);
		if ((bpid == NULL) || (lenp != sizeof(*bpid))) {
			dev_err(dev, "fsl,bpid property not found.\n");
			dpa_bp = ERR_PTR(-EINVAL);
			goto _return_of_node_put;
		}
		dpa_bp[i].bpid = *bpid;

		bpool_cfg = of_get_property(dev_node, "fsl,bpool-ethernet-cfg",
					&lenp);
		if (bpool_cfg && (lenp == (2 * ns + na) * sizeof(*bpool_cfg))) {
			const uint32_t *seed_pool;

			dpa_bp[i].config_count =
				(int)of_read_number(bpool_cfg, ns);
			dpa_bp[i].size	= of_read_number(bpool_cfg + ns, ns);
			dpa_bp[i].paddr	=
				of_read_number(bpool_cfg + 2 * ns, na);

			seed_pool = of_get_property(dev_node,
					"fsl,bpool-ethernet-seeds", &lenp);
			dpa_bp[i].seed_pool = !!seed_pool;

			has_shared_pool = 1;
		} else {
			has_kernel_pool = 1;
		}

		if (i > 0)
			has_shared_pool = 1;
	}

	if (has_kernel_pool && has_shared_pool) {
		dev_err(dev, "Invalid buffer pool configuration "
			"for node %s\n", dev_node->full_name);
		dpa_bp = ERR_PTR(-EINVAL);
		goto _return_of_node_put;
	} else if (has_kernel_pool) {
		dpa_bp->target_count = CONFIG_FSL_DPAA_ETH_MAX_BUF_COUNT;
		dpa_bp->kernel_pool = 1;
	}

	sort(dpa_bp, *count, sizeof(*dpa_bp), dpa_bp_cmp, NULL);

	return dpa_bp;

_return_of_node_put:
	if (dev_node)
		of_node_put(dev_node);

	return dpa_bp;
}

static int dpa_bp_create(struct net_device *net_dev, struct dpa_bp *dpa_bp,
			size_t count)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	int i;

	if (dpa_bp->kernel_pool) {
		priv->shared = 0;

		if (netif_msg_probe(priv))
			dev_info(net_dev->dev.parent,
				"Using private BM buffer pools\n");
	} else {
		priv->shared = 1;
	}

	priv->dpa_bp = dpa_bp;
	priv->bp_count = count;

	for (i = 0; i < count; i++) {
		int err;
		err = dpa_bp_alloc(&dpa_bp[i]);
		if (err < 0) {
			dpa_bp_free(priv, dpa_bp);
			priv->dpa_bp = NULL;
			return err;
		}

		/* For now, just point to the default pool.
		 * We can add support for more pools, later
		 */
		if (dpa_bp->kernel_pool)
			priv->dpa_bp = default_pool;
	}

	return 0;
}

static struct mac_device * __cold __must_check
__attribute__((nonnull))
dpa_mac_probe(struct platform_device *_of_dev)
{
	struct device		*dpa_dev, *dev;
	struct device_node	*mac_node;
	int			 lenp;
	const phandle		*phandle_prop;
	struct platform_device	*of_dev;
	struct mac_device	*mac_dev;
#ifdef CONFIG_FSL_DPAA_1588
	struct net_device	*net_dev = NULL;
	struct dpa_priv_s	*priv = NULL;
	struct device_node	*timer_node;
#endif

	phandle_prop = of_get_property(_of_dev->dev.of_node,
					"fsl,fman-mac", &lenp);
	if (phandle_prop == NULL)
		return NULL;

	BUG_ON(lenp != sizeof(phandle));

	dpa_dev = &_of_dev->dev;

	mac_node = of_find_node_by_phandle(*phandle_prop);
	if (unlikely(mac_node == NULL)) {
		dev_err(dpa_dev, "of_find_node_by_phandle() failed\n");
		return ERR_PTR(-EFAULT);
	}

	of_dev = of_find_device_by_node(mac_node);
	if (unlikely(of_dev == NULL)) {
		dev_err(dpa_dev, "of_find_device_by_node(%s) failed\n",
				mac_node->full_name);
		of_node_put(mac_node);
		return ERR_PTR(-EINVAL);
	}
	of_node_put(mac_node);

	dev = &of_dev->dev;

	mac_dev = dev_get_drvdata(dev);
	if (unlikely(mac_dev == NULL)) {
		dev_err(dpa_dev, "dev_get_drvdata(%s) failed\n",
				dev_name(dev));
		return ERR_PTR(-EINVAL);
	}

#ifdef CONFIG_FSL_DPAA_1588
	phandle_prop = of_get_property(mac_node, "ptimer-handle", &lenp);
	if (phandle_prop && ((mac_dev->phy_if != PHY_INTERFACE_MODE_SGMII) ||
			((mac_dev->phy_if == PHY_INTERFACE_MODE_SGMII) &&
			 (mac_dev->speed == SPEED_1000)))) {
		timer_node = of_find_node_by_phandle(*phandle_prop);
		if (timer_node && (net_dev = dev_get_drvdata(dpa_dev))) {
			priv = netdev_priv(net_dev);
			if (!dpa_ptp_init(priv))
				dev_info(dev, "%s: ptp 1588 is initialized.\n",
						mac_node->full_name);
		}
	}
#endif

	return mac_dev;
}

static const char fsl_qman_frame_queues[][25] = {
	[RX] = "fsl,qman-frame-queues-rx",
	[TX] = "fsl,qman-frame-queues-tx"
};


#ifdef CONFIG_FSL_DPAA_ETH_USE_NDO_SELECT_QUEUE
static u16 dpa_select_queue(struct net_device *net_dev, struct sk_buff *skb)
{
	return smp_processor_id();
}
#endif

static netdev_features_t dpa_fix_features(struct net_device *dev,
					  netdev_features_t features)
{
	struct dpa_priv_s *priv = netdev_priv(dev);
	netdev_features_t unsupported_features = 0;

	/* In theory we should never be requested to enable features that
	 * we didn't set in netdev->features and netdev->hw_features at probe
	 * time, but double check just to be on the safe side.
	 */
	if (!priv->mac_dev)
		unsupported_features |= NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM;
	/* We don't support enabling Rx csum through ethtool yet */
	unsupported_features |= NETIF_F_RXCSUM;

	features &= ~unsupported_features;

	return features;
}

static int dpa_set_features(struct net_device *dev, netdev_features_t features)
{
	/* Not much to do here for now */
	dev->features = features;
	return 0;
}


static const struct net_device_ops dpa_private_ops = {
	.ndo_open = dpa_start,
	.ndo_start_xmit = dpa_tx,
	.ndo_stop = dpa_stop,
	.ndo_tx_timeout = dpa_timeout,
	.ndo_get_stats64 = dpa_get_stats64,
	.ndo_set_mac_address = dpa_set_mac_address,
	.ndo_validate_addr = eth_validate_addr,
#ifdef CONFIG_FSL_DPAA_ETH_USE_NDO_SELECT_QUEUE
	.ndo_select_queue = dpa_select_queue,
#endif
	.ndo_change_mtu = dpa_change_mtu,
	.ndo_set_rx_mode = dpa_set_rx_mode,
	.ndo_init = dpa_ndo_init,
	.ndo_set_features = dpa_set_features,
	.ndo_fix_features = dpa_fix_features,
	.ndo_do_ioctl = dpa_ioctl,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller = dpaa_eth_poll_controller,
#endif
};

static const struct net_device_ops dpa_shared_ops = {
	.ndo_open = dpa_start,
	.ndo_start_xmit = dpa_shared_tx,
	.ndo_stop = dpa_stop,
	.ndo_tx_timeout = dpa_timeout,
	.ndo_get_stats64 = dpa_get_stats64,
	.ndo_set_mac_address = dpa_set_mac_address,
	.ndo_validate_addr = eth_validate_addr,
#ifdef CONFIG_FSL_DPAA_ETH_USE_NDO_SELECT_QUEUE
	.ndo_select_queue = dpa_select_queue,
#endif
	.ndo_change_mtu = dpa_change_mtu,
	.ndo_set_rx_mode = dpa_set_rx_mode,
	.ndo_init = dpa_ndo_init,
	.ndo_set_features = dpa_set_features,
	.ndo_fix_features = dpa_fix_features,
	.ndo_do_ioctl = dpa_ioctl,
};

static u32 rx_pool_channel;
static DEFINE_SPINLOCK(rx_pool_channel_init);

static int dpa_get_channel(struct device *dev,
					struct device_node *dpa_node)
{
	spin_lock(&rx_pool_channel_init);
	if (!rx_pool_channel) {
		u32 pool;
		int ret = qman_alloc_pool(&pool);
		if (!ret)
			rx_pool_channel = pool;
	}
	spin_unlock(&rx_pool_channel_init);
	if (!rx_pool_channel)
		return -ENOMEM;
	return rx_pool_channel;
}

struct fqid_cell {
	uint32_t start;
	uint32_t count;
};

static const struct fqid_cell default_fqids[][3] = {
	[RX] = { {0, 1}, {0, 1}, {0, DPAA_ETH_RX_QUEUES} },
	[TX] = { {0, 1}, {0, 1}, {0, DPAA_ETH_TX_QUEUES} }
};

static const struct fqid_cell tx_confirm_fqids[] = {
	{0, DPAA_ETH_TX_QUEUES}
};

#ifdef CONFIG_FSL_DPAA_TX_RECYCLE
static const struct fqid_cell tx_recycle_fqids[] = {
	{0, DPAA_ETH_TX_QUEUES}
};
#endif

static int
dpa_fq_probe(struct platform_device *_of_dev, struct list_head *list,
		struct dpa_fq **defq, struct dpa_fq **errq,
		struct dpa_fq **fqs, struct dpa_fq **txconfq,
		struct dpa_fq **txrecycle, int ptype)
{
	struct device *dev = &_of_dev->dev;
	struct device_node *np = dev->of_node;
	const struct fqid_cell *fqids;
	int i, j, lenp;
	int num_fqids;
	struct dpa_fq *dpa_fq;
	int err = 0;

	/* per-core tx confirmation queues */
	if (txconfq) {
		fqids = tx_confirm_fqids;
		dpa_fq = devm_kzalloc(dev, sizeof(*dpa_fq) * fqids[0].count,
					GFP_KERNEL);
		if (dpa_fq == NULL) {
			dev_err(dev, "devm_kzalloc() failed\n");
			return -ENOMEM;
		}
		*txconfq = dpa_fq;
		for (j = 0; j < fqids[0].count; j++)
			dpa_fq[j].fq_type = FQ_TYPE_TX_CONFIRM;

		for (j = 0; j < fqids[0].count; j++) {
			dpa_fq[j].fqid = fqids[0].start ?
				fqids[0].start + j : 0;
			_dpa_assign_wq(dpa_fq + j);
			list_add_tail(&dpa_fq[j].list, list);
		}
	}

#ifdef CONFIG_FSL_DPAA_TX_RECYCLE
	/* per-core tx queues for recycleable frames (FManv3 only) */
	if (txrecycle) {
		fqids = tx_recycle_fqids;
		dpa_fq = devm_kzalloc(dev, sizeof(*dpa_fq) * fqids[0].count,
					GFP_KERNEL);
		if (dpa_fq == NULL) {
			dev_err(dev, "devm_kzalloc() failed\n");
			return -ENOMEM;
		}

		*txrecycle = dpa_fq;
		for (j = 0; j < fqids[0].count; j++)
			dpa_fq[j].fq_type = FQ_TYPE_TX_RECYCLE;

		for (j = 0; j < fqids[0].count; j++) {
			dpa_fq[j].fqid = fqids[0].start ?
				fqids[0].start + j : 0;
			_dpa_assign_wq(dpa_fq + j);
			list_add_tail(&dpa_fq[j].list, list);
		}
	}
#endif

	fqids = of_get_property(np, fsl_qman_frame_queues[ptype], &lenp);
	if (fqids == NULL) {
		fqids = default_fqids[ptype];
		num_fqids = 3;
	} else
		num_fqids = lenp / sizeof(*fqids);

	for (i = 0; i < num_fqids; i++) {
		dpa_fq = devm_kzalloc(dev, sizeof(*dpa_fq) * fqids[i].count,
					GFP_KERNEL);
		if (dpa_fq == NULL) {
			dev_err(dev, "devm_kzalloc() failed\n");
			return -ENOMEM;
		}

		/* The first queue is the Error queue */
		if (i == 0 && errq) {
			*errq = dpa_fq;

			if (fqids[i].count != 1) {
				dev_err(dev, "Too many error queues!\n");
				err = -EINVAL;
				goto invalid_error_queues;
			}

			dpa_fq[0].fq_type = (ptype == RX ?
				FQ_TYPE_RX_ERROR : FQ_TYPE_TX_ERROR);
		}

		/* The second queue is the the Default queue */
		if (i == 1 && defq) {
			*defq = dpa_fq;

			if (fqids[i].count != 1) {
				dev_err(dev, "Too many default queues!\n");
				err = -EINVAL;
				goto invalid_default_queues;
			}

			dpa_fq[0].fq_type = (ptype == RX ?
				FQ_TYPE_RX_DEFAULT : FQ_TYPE_TX_CONFIRM);
		}

		/*
		 * All subsequent queues are gathered together.
		 * The first 8 will be used by the private linux interface
		 * if these are TX queues
		 */
		if (i == 2 || (!errq && i == 0 && fqs)) {
			*fqs = dpa_fq;

			for (j = 0; j < fqids[i].count; j++)
				dpa_fq[j].fq_type = (ptype == RX ?
					FQ_TYPE_RX_PCD : FQ_TYPE_TX);
		}

		for (j = 0; j < fqids[i].count; j++) {
			dpa_fq[j].fqid = fqids[i].start ?
				fqids[i].start + j : 0;
			_dpa_assign_wq(dpa_fq + j);
			list_add_tail(&dpa_fq[j].list, list);
		}
	}

invalid_default_queues:
invalid_error_queues:
	return err;
}

static void dpa_setup_ingress(struct dpa_priv_s *priv, struct dpa_fq *fq,
			const struct qman_fq *template)
{
	fq->fq_base = *template;
	fq->net_dev = priv->net_dev;

	fq->flags = QMAN_FQ_FLAG_NO_ENQUEUE;
	fq->channel = priv->channel;
}

static void dpa_setup_egress(struct dpa_priv_s *priv,
				struct list_head *head, struct dpa_fq *fq,
				struct fm_port *port)
{
	struct list_head *ptr = &fq->list;
	struct dpa_fq *iter;
	int i = 0;

	while (true) {
		iter = list_entry(ptr, struct dpa_fq, list);
		if (priv->shared)
			iter->fq_base = shared_egress_fq;
		else
			iter->fq_base = private_egress_fq;

		iter->net_dev = priv->net_dev;
		if (port) {
			iter->flags = QMAN_FQ_FLAG_TO_DCPORTAL;
			iter->channel = fm_get_tx_port_channel(port);
		} else
			iter->flags = QMAN_FQ_FLAG_NO_MODIFY;

		if (list_is_last(ptr, head))
			break;

		ptr = ptr->next;
	}

	/* Allocate frame queues to all available CPUs no matter the number of
	 * queues specified in device tree.
	 */
	for (i = 0, ptr = &fq->list; i < DPAA_ETH_TX_QUEUES; i++) {
		iter = list_entry(ptr, struct dpa_fq, list);
		priv->egress_fqs[i] = &iter->fq_base;

		if (list_is_last(ptr, head)) {
			ptr = &fq->list;
			continue;
		}

		ptr = ptr->next;
	}
}

#ifdef CONFIG_FSL_DPAA_TX_RECYCLE
static void dpa_setup_recycle_queues(struct dpa_priv_s *priv, struct dpa_fq *fq,
				     struct fm_port *port)
{
	int i = 0;
	struct list_head *ptr = &fq->list;

	for (i = 0; i < DPAA_ETH_TX_QUEUES; i++) {
		struct dpa_fq *iter = list_entry(ptr, struct dpa_fq, list);

		iter->fq_base = private_egress_fq;
		iter->net_dev = priv->net_dev;

		priv->recycle_fqs[i] = &iter->fq_base;

		iter->flags = QMAN_FQ_FLAG_TO_DCPORTAL;
		iter->channel = fm_get_tx_port_channel(port);

		ptr = ptr->next;
	}
}
#endif

static void dpa_setup_conf_queues(struct dpa_priv_s *priv, struct dpa_fq *fq)
{
	struct list_head *ptr = &fq->list;
	int i;

	/*
	 * Configure the queues to be core affine.
	 * The implicit assumption here is that each cpu has its own Tx queue
	 */
	for (i = 0; i < DPAA_ETH_TX_QUEUES; i++) {
		struct dpa_fq *iter = list_entry(ptr, struct dpa_fq, list);

		dpa_setup_ingress(priv, iter, &tx_private_defq);
		/* Leave the confirmation queue in the default pool channel */
		priv->conf_fqs[i] = &iter->fq_base;

		ptr = ptr->next;
	}
}

static void dpa_setup_ingress_queues(struct dpa_priv_s *priv,
		struct list_head *head, struct dpa_fq *fq)
{
	struct list_head *ptr = &fq->list;
	u32 fqid;
	int portals[NR_CPUS];
	int i, cpu, num_portals = 0;
	const cpumask_t *affine_cpus = qman_affine_cpus();

	for_each_cpu(cpu, affine_cpus)
		portals[num_portals++] = qman_affine_channel(cpu);
	if (num_portals == 0) {
		dev_err(fq->net_dev->dev.parent,
			     "No Qman software (affine) channels found");
		return;
	}

	i = 0;
	fqid = 0;
	if (priv->mac_dev)
		fqid = (priv->mac_dev->res->start & 0x1fffff) >> 6;

	while (true) {
		struct dpa_fq *iter = list_entry(ptr, struct dpa_fq, list);

		if (priv->shared)
			dpa_setup_ingress(priv, iter, &rx_shared_fq);
		else
			dpa_setup_ingress(priv, iter, &rx_private_defq);

		if (!iter->fqid)
			iter->fqid = fqid++;

		/* Assign the queues to a channel in a round-robin fashion */
		iter->channel = portals[i];
		i = (i + 1) % num_portals;

		if (list_is_last(ptr, head))
			break;

		ptr = ptr->next;
	}
}

static void
dpaa_eth_init_tx_port(struct fm_port *port, struct dpa_fq *errq,
		struct dpa_fq *defq, struct dpa_buffer_layout_s *buf_layout)
{
	struct fm_port_params tx_port_param;
	bool frag_enabled = false;

	memset(&tx_port_param, 0, sizeof(tx_port_param));
	dpaa_eth_init_port(tx, port, tx_port_param, errq->fqid, defq->fqid,
			   buf_layout, frag_enabled);
}

static void
dpaa_eth_init_rx_port(struct fm_port *port, struct dpa_bp *bp, size_t count,
		struct dpa_fq *errq, struct dpa_fq *defq,
		struct dpa_buffer_layout_s *buf_layout)
{
	struct fm_port_params rx_port_param;
	int i;
	bool frag_enabled = false;

	memset(&rx_port_param, 0, sizeof(rx_port_param));
	count = min(ARRAY_SIZE(rx_port_param.pool_param), count);
	rx_port_param.num_pools = count;
	for (i = 0; i < count; i++) {
		if (i >= rx_port_param.num_pools)
			break;
		rx_port_param.pool_param[i].id = bp[i].bpid;
		rx_port_param.pool_param[i].size = bp[i].size;
	}

	dpaa_eth_init_port(rx, port, rx_port_param, errq->fqid, defq->fqid,
			   buf_layout, frag_enabled);
}

static void dpa_rx_fq_init(struct dpa_priv_s *priv, struct list_head *head,
			struct dpa_fq *defq, struct dpa_fq *errq,
			struct dpa_fq *fqs)
{
	if (fqs)
		dpa_setup_ingress_queues(priv, head, fqs);

	/* Only real devices need default/error queues set up */
	if (!priv->mac_dev)
		return;

	if (defq->fqid == 0 && netif_msg_probe(priv))
		pr_info("Using dynamic RX QM frame queues\n");

	if (priv->shared) {
		dpa_setup_ingress(priv, defq, &rx_shared_fq);
		dpa_setup_ingress(priv, errq, &rx_shared_fq);
	} else {
		dpa_setup_ingress(priv, defq, &rx_private_defq);
		dpa_setup_ingress(priv, errq, &rx_private_errq);
	}
}

static void dpa_tx_fq_init(struct dpa_priv_s *priv, struct list_head *head,
			struct dpa_fq *defq, struct dpa_fq *errq,
			struct dpa_fq *fqs, struct dpa_fq *confqs,
			struct dpa_fq *recyclefqs, struct fm_port *port)
{
	if (fqs)
		dpa_setup_egress(priv, head, fqs, port);

	/* Only real devices need default/error queues set up */
	if (!priv->mac_dev)
		return;

	if (defq->fqid == 0 && netif_msg_probe(priv))
		pr_info("Using dynamic TX QM frame queues\n");

	/* The shared driver doesn't use tx confirmation */
	if (priv->shared) {
		dpa_setup_ingress(priv, defq, &tx_shared_defq);
		dpa_setup_ingress(priv, errq, &tx_shared_errq);
	} else {
		dpa_setup_ingress(priv, defq, &tx_private_defq);
		dpa_setup_ingress(priv, errq, &tx_private_errq);
		if (confqs)
			dpa_setup_conf_queues(priv, confqs);
#ifdef CONFIG_FSL_DPAA_TX_RECYCLE
		if (recyclefqs)
			dpa_setup_recycle_queues(priv, recyclefqs, port);
#endif

	}
}

static int dpa_netdev_init(struct device_node *dpa_node,
		struct net_device *net_dev)
{
	int err;
	const uint8_t *mac_addr;
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	struct device *dev = net_dev->dev.parent;

	net_dev->hw_features |= DPA_NETIF_FEATURES;

	if (!priv->mac_dev) {
		/* Get the MAC address */
		mac_addr = of_get_mac_address(dpa_node);
		if (mac_addr == NULL) {
			if (netif_msg_probe(priv))
				dev_err(dev, "No MAC address found!\n");
			return -EINVAL;
		}
	} else {
		net_dev->mem_start = priv->mac_dev->res->start;
		net_dev->mem_end = priv->mac_dev->res->end;

		mac_addr = priv->mac_dev->addr;
		net_dev->hw_features |= (NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM |
			NETIF_F_LLTX);

		/*
		 * Advertise S/G and HIGHDMA support for MAC-ful,
		 * private interfaces
		 */
		if (!priv->shared) {
#ifdef CONFIG_FSL_DPAA_ETH_SG_SUPPORT
			net_dev->hw_features |= NETIF_F_SG | NETIF_F_HIGHDMA;
			/* Recent kernels enable GSO automatically, if
			 * we declare NETIF_F_SG. For conformity, we'll
			 * still declare GSO explicitly.
			 */
			net_dev->features |= NETIF_F_GSO;
#endif
			/* Advertise GRO support */
			net_dev->features |= NETIF_F_GRO;
		}
	}

	net_dev->priv_flags |= IFF_LIVE_ADDR_CHANGE;

	net_dev->features |= net_dev->hw_features;
	net_dev->vlan_features = net_dev->features;

	memcpy(net_dev->perm_addr, mac_addr, net_dev->addr_len);
	memcpy(net_dev->dev_addr, mac_addr, net_dev->addr_len);

	SET_ETHTOOL_OPS(net_dev, &dpa_ethtool_ops);

	net_dev->needed_headroom = priv->tx_headroom;
	net_dev->watchdog_timeo = msecs_to_jiffies(tx_timeout);

	err = register_netdev(net_dev);
	if (err < 0) {
		dev_err(dev, "register_netdev() = %d\n", err);
		return err;
	}

#ifdef CONFIG_FSL_DPAA_ETH_DEBUGFS
	/* create debugfs entry for this net_device */
	err = dpa_netdev_debugfs_create(net_dev);
	if (err) {
		unregister_netdev(net_dev);
		return err;
	}
#endif /* CONFIG_FSL_DPAA_ETH_DEBUGFS */

	return 0;
}

static int dpa_shared_netdev_init(struct device_node *dpa_node,
				struct net_device *net_dev)
{
	net_dev->netdev_ops = &dpa_shared_ops;

	return dpa_netdev_init(dpa_node, net_dev);
}

static int dpa_private_netdev_init(struct device_node *dpa_node,
				struct net_device *net_dev)
{
	int i;
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	struct dpa_percpu_priv_s *percpu_priv;

	/*
	 * Although we access another CPU's private data here
	 * we do it at initialization so it is safe
	 */
	for_each_online_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);
		percpu_priv->net_dev = net_dev;

		netif_napi_add(net_dev, &percpu_priv->napi, dpaa_eth_poll,
			       DPA_NAPI_WEIGHT);
	}

	net_dev->netdev_ops = &dpa_private_ops;

	return dpa_netdev_init(dpa_node, net_dev);
}

int dpa_alloc_pcd_fqids(struct device *dev, uint32_t num,
				uint8_t alignment, uint32_t *base_fqid)
{
	dev_crit(dev, "callback not implemented!\n");
	BUG();

	return 0;
}

int dpa_free_pcd_fqids(struct device *dev, uint32_t base_fqid)
{

	dev_crit(dev, "callback not implemented!\n");
	BUG();

	return 0;
}

static int dpaa_eth_add_channel(void *__arg)
{
	const cpumask_t *cpus = qman_affine_cpus();
	u32 pool = QM_SDQCR_CHANNELS_POOL_CONV((u32)(unsigned long)__arg);
	int cpu;

	for_each_cpu(cpu, cpus) {
		set_cpus_allowed_ptr(current, get_cpu_mask(cpu));
		qman_static_dequeue_add(pool);
	}
	return 0;
}

static int dpaa_eth_cgr_init(struct dpa_priv_s *priv)
{
	struct qm_mcc_initcgr initcgr;
	u32 cs_th;
	int err;

	err = qman_alloc_cgrid(&priv->cgr_data.cgr.cgrid);
	if (err < 0) {
		pr_err("Error %d allocating CGR ID\n", err);
		goto out_error;
	}
	priv->cgr_data.cgr.cb = dpaa_eth_cgscn;

	/* Enable Congestion State Change Notifications and CS taildrop */
	initcgr.we_mask = QM_CGR_WE_CSCN_EN | QM_CGR_WE_CS_THRES;
	initcgr.cgr.cscn_en = QM_CGR_EN;
	/*
	 * Set different thresholds based on the MAC speed.
	 * TODO: this may turn suboptimal if the MAC is reconfigured at a speed
	 * lower than its max, e.g. if a dTSEC later negotiates a 100Mbps link.
	 * In such cases, we ought to reconfigure the threshold, too.
	 */
	if (priv->mac_dev->if_support & SUPPORTED_10000baseT_Full)
		cs_th = DPA_CS_THRESHOLD_10G;
	else
		cs_th = DPA_CS_THRESHOLD_1G;
	qm_cgr_cs_thres_set64(&initcgr.cgr.cs_thres, cs_th, 1);

	initcgr.we_mask |= QM_CGR_WE_CSTD_EN;
	initcgr.cgr.cstd_en = QM_CGR_EN;

	err = qman_create_cgr(&priv->cgr_data.cgr, QMAN_CGR_FLAG_USE_INIT,
		&initcgr);
	if (err < 0) {
		pr_err("Error %d creating CGR with ID %d\n", err,
			priv->cgr_data.cgr.cgrid);
		qman_release_cgrid(priv->cgr_data.cgr.cgrid);
		goto out_error;
	}
	pr_debug("Created CGR %d for netdev with hwaddr %pM on "
		"QMan channel %d\n", priv->cgr_data.cgr.cgrid,
		priv->mac_dev->addr, priv->cgr_data.cgr.chan);

out_error:
	return err;
}

static const struct of_device_id dpa_match[];
static int
dpaa_eth_probe(struct platform_device *_of_dev)
{
	int err = 0, i;
	struct device *dev;
	struct device_node *dpa_node;
	struct dpa_bp *dpa_bp;
	struct dpa_fq *dpa_fq, *tmp;
	struct list_head rxfqlist;
	struct list_head txfqlist;
	size_t count;
	struct net_device *net_dev = NULL;
	struct dpa_priv_s *priv = NULL;
	struct dpa_percpu_priv_s *percpu_priv;
	struct dpa_fq *rxdefault = NULL;
	struct dpa_fq *txdefault = NULL;
	struct dpa_fq *rxerror = NULL;
	struct dpa_fq *txerror = NULL;
	struct dpa_fq *rxextra = NULL;
	struct dpa_fq *txfqs = NULL;
	struct dpa_fq *txconf = NULL;
	struct dpa_fq *txrecycle = NULL;
	struct fm_port *rxport = NULL;
	struct fm_port *txport = NULL;
	struct dpa_buffer_layout_s *buf_layout = NULL;
	bool is_shared = false;
	struct mac_device *mac_dev;
	int proxy_enet;
	const struct of_device_id *match;

	dev = &_of_dev->dev;

	dpa_node = dev->of_node;

	match = of_match_device(dpa_match, dev);
	if (!match)
		return -EINVAL;

	if (!of_device_is_available(dpa_node))
		return -ENODEV;

	/*
	 * If it's not an fsl,dpa-ethernet node, we just serve as a proxy
	 * initializer driver, and don't do any linux device setup
	 */
	proxy_enet = strcmp(match->compatible, "fsl,dpa-ethernet");

	/*
	 * Allocate this early, so we can store relevant information in
	 * the private area
	 */
	if (!proxy_enet) {
		net_dev = alloc_etherdev_mq(sizeof(*priv), DPAA_ETH_TX_QUEUES);
		if (!net_dev) {
			dev_err(dev, "alloc_etherdev_mq() failed\n");
			return -ENOMEM;
		}

		/* Do this here, so we can be verbose early */
		SET_NETDEV_DEV(net_dev, dev);
		dev_set_drvdata(dev, net_dev);

		priv = netdev_priv(net_dev);
		priv->net_dev = net_dev;

		priv->msg_enable = netif_msg_init(debug, -1);
	}

	/* Get the buffer pools assigned to this interface */
	dpa_bp = dpa_bp_probe(_of_dev, &count);
	if (IS_ERR(dpa_bp)) {
		err = PTR_ERR(dpa_bp);
		goto bp_probe_failed;
	}

	mac_dev = dpa_mac_probe(_of_dev);
	if (IS_ERR(mac_dev)) {
		err = PTR_ERR(mac_dev);
		goto mac_probe_failed;
	} else if (mac_dev) {
		rxport = mac_dev->port_dev[RX];
		txport = mac_dev->port_dev[TX];

		/* We have physical ports, so we need to establish
		 * the buffer layout.
		 */
		buf_layout = devm_kzalloc(dev, 2 * sizeof(*buf_layout),
					  GFP_KERNEL);
		if (!buf_layout) {
			dev_err(dev, "devm_kzalloc() failed\n");
			goto alloc_failed;
		}
		dpa_set_buffer_layout(priv, rxport, &buf_layout[RX], RX);
		dpa_set_buffer_layout(priv, txport, &buf_layout[TX], TX);
	}

	if (!dpa_bp->kernel_pool) {
		is_shared = true;
	} else {
		/* For private ports, need to compute the size of the default
		 * buffer pool, based on FMan port buffer layout;also update
		 * the maximum buffer size for private ports if necessary
		 */
		dpa_bp->size = dpa_bp_size(&buf_layout[RX]);
		if (dpa_bp->size > default_buf_size)
			default_buf_size = dpa_bp->size;
	}

	INIT_LIST_HEAD(&rxfqlist);
	INIT_LIST_HEAD(&txfqlist);

	if (rxport)
		err = dpa_fq_probe(_of_dev, &rxfqlist, &rxdefault, &rxerror,
				&rxextra, NULL, NULL, RX);
	else
		err = dpa_fq_probe(_of_dev, &rxfqlist, NULL, NULL,
				&rxextra, NULL, NULL, RX);

	if (err < 0)
		goto rx_fq_probe_failed;

	if (txport)
#ifdef CONFIG_FSL_DPAA_TX_RECYCLE
		err = dpa_fq_probe(_of_dev, &txfqlist, &txdefault, &txerror,
				&txfqs, (is_shared ? NULL : &txconf),
				(is_shared ? NULL : &txrecycle), TX);
#else
		err = dpa_fq_probe(_of_dev, &txfqlist, &txdefault, &txerror,
				&txfqs, (is_shared ? NULL : &txconf), NULL, TX);
#endif
	else
		err = dpa_fq_probe(_of_dev, &txfqlist, NULL, NULL, &txfqs,
				NULL, NULL, TX);

	if (err < 0)
		goto tx_fq_probe_failed;

	/*
	 * Now we have all of the configuration information.
	 * We support a number of configurations:
	 * 1) Private interface - An optimized linux ethernet driver with
	 *    a real network connection.
	 * 2) Shared interface - A device intended for virtual connections
	 *    or for a real interface that is shared between partitions
	 * 3) Proxy initializer - Just configures the MAC on behalf of
	 *    another partition
	 */

	/* bp init */
	if (net_dev) {
		struct task_struct *kth;

		err = dpa_bp_create(net_dev, dpa_bp, count);

		if (err < 0)
			goto bp_create_failed;

		priv->mac_dev = mac_dev;

		priv->channel = dpa_get_channel(dev, dpa_node);

		if (priv->channel < 0) {
			err = priv->channel;
			goto get_channel_failed;
		}

		/* Start a thread that will walk the cpus with affine portals
		 * and add this pool channel to each's dequeue mask. */
		kth = kthread_run(dpaa_eth_add_channel,
				  (void *)(unsigned long)priv->channel,
				  "dpaa_%p:%d", net_dev, priv->channel);
		if (!kth) {
			err = -ENOMEM;
			goto add_channel_failed;
		}

		dpa_rx_fq_init(priv, &rxfqlist, rxdefault, rxerror, rxextra);
		dpa_tx_fq_init(priv, &txfqlist, txdefault, txerror, txfqs,
				txconf, txrecycle, txport);

		/*
		 * Create a congestion group for this netdev, with
		 * dynamically-allocated CGR ID.
		 * Must be executed after probing the MAC, but before
		 * assigning the egress FQs to the CGRs.
		 * Don't create a congestion group for MAC-less interfaces.
		 */
		if (priv->mac_dev) {
			err = dpaa_eth_cgr_init(priv);
			if (err < 0) {
				dev_err(dev, "Error initializing CGR\n");
				goto cgr_init_failed;
			}
		}

		/* Add the FQs to the interface, and make them active */
		INIT_LIST_HEAD(&priv->dpa_fq_list);

		list_for_each_entry_safe(dpa_fq, tmp, &rxfqlist, list) {
			err = _dpa_fq_alloc(&priv->dpa_fq_list, dpa_fq);
			if (err < 0)
				goto fq_alloc_failed;
		}

		list_for_each_entry_safe(dpa_fq, tmp, &txfqlist, list) {
			err = _dpa_fq_alloc(&priv->dpa_fq_list, dpa_fq);
			if (err < 0)
				goto fq_alloc_failed;
		}

		if (mac_dev) {
			priv->buf_layout = buf_layout;
			priv->tx_headroom =
				dpa_get_headroom(&priv->buf_layout[TX]);
		} else {
			priv->tx_headroom = DPA_DEFAULT_TX_HEADROOM;
		}
	}

	/* All real interfaces need their ports initialized */
	if (mac_dev) {
		struct fm_port_pcd_param rx_port_pcd_param;

		dpaa_eth_init_tx_port(txport, txerror, txdefault,
				      &buf_layout[TX]);
		dpaa_eth_init_rx_port(rxport, dpa_bp, count, rxerror,
				      rxdefault, &buf_layout[RX]);

		rx_port_pcd_param.cba = dpa_alloc_pcd_fqids;
		rx_port_pcd_param.cbf = dpa_free_pcd_fqids;
		rx_port_pcd_param.dev = dev;
		fm_port_pcd_bind(rxport, &rx_port_pcd_param);
	}

	/*
	 * Proxy interfaces need to be started, and the allocated
	 * memory freed
	 */
	if (!net_dev) {
		devm_kfree(&_of_dev->dev, dpa_bp);
		devm_kfree(&_of_dev->dev, rxdefault);
		devm_kfree(&_of_dev->dev, rxerror);
		devm_kfree(&_of_dev->dev, txdefault);
		devm_kfree(&_of_dev->dev, txerror);

		if (mac_dev)
			for_each_port_device(i, mac_dev->port_dev)
				fm_port_enable(mac_dev->port_dev[i]);

		return 0;
	}

	/* Now we need to initialize either a private or shared interface */
	priv->percpu_priv = alloc_percpu(*priv->percpu_priv);

	if (priv->percpu_priv == NULL) {
		dev_err(dev, "alloc_percpu() failed\n");
		err = -ENOMEM;
		goto alloc_percpu_failed;
	}
	for_each_online_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);
		memset(percpu_priv, 0, sizeof(*percpu_priv));
	}

	if (priv->shared)
		err = dpa_shared_netdev_init(dpa_node, net_dev);
	else
		err = dpa_private_netdev_init(dpa_node, net_dev);

	if (err < 0)
		goto netdev_init_failed;

	dpaa_eth_sysfs_init(&net_dev->dev);

#ifdef CONFIG_FSL_DPAA_ETH_UNIT_TESTS
	/* The unit test is designed to test private interfaces */
	if (!priv->shared && !tx_unit_test_ran) {
		err = dpa_tx_unit_test(net_dev);

		WARN_ON(err);
	}
#endif

	return 0;

netdev_init_failed:
	if (net_dev)
		free_percpu(priv->percpu_priv);
alloc_percpu_failed:
fq_alloc_failed:
	if (net_dev) {
		dpa_fq_free(dev, &priv->dpa_fq_list);
		qman_release_cgrid(priv->cgr_data.cgr.cgrid);
		qman_delete_cgr(&priv->cgr_data.cgr);
	}
cgr_init_failed:
add_channel_failed:
get_channel_failed:
	if (net_dev)
		dpa_bp_free(priv, priv->dpa_bp);
bp_create_failed:
tx_fq_probe_failed:
rx_fq_probe_failed:
alloc_failed:
mac_probe_failed:
bp_probe_failed:
	dev_set_drvdata(dev, NULL);
	if (net_dev)
		free_netdev(net_dev);

	return err;
}

static const struct of_device_id dpa_match[] = {
	{
		.compatible	= "fsl,dpa-ethernet"
	},
	{
		.compatible	= "fsl,dpa-ethernet-init"
	},
	{}
};
MODULE_DEVICE_TABLE(of, dpa_match);

static int __cold dpa_remove(struct platform_device *of_dev)
{
	int			err;
	struct device		*dev;
	struct net_device	*net_dev;
	struct dpa_priv_s	*priv;

	dev = &of_dev->dev;
	net_dev = dev_get_drvdata(dev);
	priv = netdev_priv(net_dev);

	dpaa_eth_sysfs_remove(dev);

	dev_set_drvdata(dev, NULL);
	unregister_netdev(net_dev);

	err = dpa_fq_free(dev, &priv->dpa_fq_list);

	free_percpu(priv->percpu_priv);

	dpa_bp_free(priv, priv->dpa_bp);

#ifdef CONFIG_FSL_DPAA_ETH_DEBUGFS
	/* remove debugfs entry for this net_device */
	dpa_netdev_debugfs_remove(net_dev);
#endif /* CONFIG_FSL_DPAA_ETH_DEBUGFS */

#ifdef CONFIG_FSL_DPAA_1588
	if (priv->tsu && priv->tsu->valid)
		dpa_ptp_cleanup(priv);
#endif

	free_netdev(net_dev);

	return err;
}

static struct platform_driver dpa_driver = {
	.driver = {
		.name		= KBUILD_MODNAME,
		.of_match_table	= dpa_match,
		.owner		= THIS_MODULE,
	},
	.probe		= dpaa_eth_probe,
	.remove		= dpa_remove
};

static int __init __cold dpa_load(void)
{
	int	 _errno;

	pr_info(KBUILD_MODNAME ": " DPA_DESCRIPTION " (" VERSION ")\n");

	/* initialise dpaa_eth mirror values */
	dpa_rx_extra_headroom = fm_get_rx_extra_headroom();
	dpa_max_frm = fm_get_max_frm();

	_errno = platform_driver_register(&dpa_driver);
	if (unlikely(_errno < 0)) {
		pr_err(KBUILD_MODNAME
			": %s:%hu:%s(): platform_driver_register() = %d\n",
			KBUILD_BASENAME".c", __LINE__, __func__, _errno);
	}

	pr_debug(KBUILD_MODNAME ": %s:%s() ->\n",
		KBUILD_BASENAME".c", __func__);

	return _errno;
}
module_init(dpa_load);

static void __exit __cold dpa_unload(void)
{
	pr_debug(KBUILD_MODNAME ": -> %s:%s()\n",
		KBUILD_BASENAME".c", __func__);

	platform_driver_unregister(&dpa_driver);

	pr_debug(KBUILD_MODNAME ": %s:%s() ->\n",
		KBUILD_BASENAME".c", __func__);
}
module_exit(dpa_unload);
