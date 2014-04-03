/*
 * Copyright 2012 Freescale Semiconductor Inc.
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
#include <linux/skbuff.h>
#include <linux/highmem.h>
#include <linux/fsl_bman.h>

#include "dpaa_eth.h"
#include "dpaa_1588.h"

#ifdef CONFIG_FSL_DPAA_ETH_SG_SUPPORT
#define DPA_SGT_MAX_ENTRIES 16 /* maximum number of entries in SG Table */

/*
 * It does not return a page as you get the page from the fd,
 * this is only for refcounting and DMA unmapping
 */
static inline void dpa_bp_removed_one_page(struct dpa_bp *dpa_bp,
					   dma_addr_t dma_addr)
{
	int *count_ptr;

	count_ptr = __this_cpu_ptr(dpa_bp->percpu_count);
	(*count_ptr)--;

	dma_unmap_single(dpa_bp->dev, dma_addr, dpa_bp->size,
		DMA_BIDIRECTIONAL);
}

/* DMA map and add a page into the bpool */
static void dpa_bp_add_page(struct dpa_bp *dpa_bp, unsigned long vaddr)
{
	struct bm_buffer bmb;
	int *count_ptr;
	dma_addr_t addr;
	int offset;

	count_ptr = __this_cpu_ptr(dpa_bp->percpu_count);

	/* Make sure we don't map beyond end of page */
	offset = vaddr & (PAGE_SIZE - 1);
	if (unlikely(dpa_bp->size + offset > PAGE_SIZE)) {
		free_page(vaddr);
		return;
	}
	addr = dma_map_single(dpa_bp->dev, (void *)vaddr, dpa_bp->size,
			      DMA_BIDIRECTIONAL);
	if (unlikely(dma_mapping_error(dpa_bp->dev, addr))) {
		dev_err(dpa_bp->dev, "DMA mapping failed");
		return;
	}

	bm_buffer_set64(&bmb, addr);

	while (bman_release(dpa_bp->pool, &bmb, 1, 0))
		cpu_relax();

	(*count_ptr)++;
}

int _dpa_bp_add_8_pages(const struct dpa_bp *dpa_bp)
{
	struct bm_buffer bmb[8];
	unsigned long new_page;
	dma_addr_t addr;
	int i;
	struct device *dev = dpa_bp->dev;

	for (i = 0; i < 8; i++) {
		new_page = __get_free_page(GFP_ATOMIC);
		if (likely(new_page)) {
			addr = dma_map_single(dev, (void *)new_page,
					dpa_bp->size, DMA_BIDIRECTIONAL);
			if (likely(!dma_mapping_error(dev, addr))) {
				bm_buffer_set64(&bmb[i], addr);
				continue;
			} else
				free_page(new_page);
		}

		/* Something went wrong */
		goto bail_out;
	}

release_bufs:
	/*
	 * Release the buffers. In case bman is busy, keep trying
	 * until successful. bman_release() is guaranteed to succeed
	 * in a reasonable amount of time
	 */
	while (unlikely(bman_release(dpa_bp->pool, bmb, i, 0)))
		cpu_relax();

	return i;

bail_out:
	net_err_ratelimited("dpa_bp_add_8_pages() failed\n");
	WARN_ONCE(1, "Memory allocation failure on Rx\n");

	bm_buffer_set64(&bmb[i], 0);
	/*
	 * Avoid releasing a completely null buffer; bman_release() requires
	 * at least one buffer.
	 */
	if (likely(i))
		goto release_bufs;

	return 0;
}

/*
 * Cold path wrapper over _dpa_bp_add_8_pages().
 */
void dpa_bp_add_8_pages(const struct dpa_bp *dpa_bp, int cpu)
{
	int *count_ptr = per_cpu_ptr(dpa_bp->percpu_count, cpu);
	*count_ptr += _dpa_bp_add_8_pages(dpa_bp);
}

void dpa_make_private_pool(struct dpa_bp *dpa_bp)
{
	int i;

	dpa_bp->percpu_count = alloc_percpu(*dpa_bp->percpu_count);

	/* Give each CPU an allotment of "page_count" buffers */
	for_each_online_cpu(i) {
		int j;

		/*
		 * Although we access another CPU's counters here
		 * we do it at boot time so it is safe
		 */
		for (j = 0; j < dpa_bp->config_count; j += 8)
			dpa_bp_add_8_pages(dpa_bp, i);
	}
}

/*
 * Cleanup function for outgoing frame descriptors that were built on Tx path,
 * either contiguous frames or scatter/gather ones.
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
	const struct qm_sg_entry *sgt;
	int i;
	struct dpa_bp *dpa_bp = priv->dpa_bp;
	dma_addr_t addr = qm_fd_addr(fd);
	struct sk_buff **skbh;
	struct sk_buff *skb = NULL;
	const enum dma_data_direction dma_dir = DMA_TO_DEVICE;
	int nr_frags;

	dma_unmap_single(dpa_bp->dev, addr, dpa_bp->size, dma_dir);

	/* retrieve skb back pointer */
	skbh = (struct sk_buff **)phys_to_virt(addr);
	skb = *skbh;
	nr_frags = skb_shinfo(skb)->nr_frags;

	if (fd->format == qm_fd_sg) {
		/*
		 * The sgt buffer has been allocated with netdev_alloc_frag(),
		 * it's from lowmem.
		 */
		sgt = phys_to_virt(addr + dpa_fd_offset(fd));
#ifdef CONFIG_FSL_DPAA_1588
		if (priv->tsu && priv->tsu->valid &&
				priv->tsu->hwts_tx_en_ioctl)
			dpa_ptp_store_txstamp(priv, skb, (void *)skbh);
#endif
#ifdef CONFIG_FSL_DPAA_TS
		if (unlikely(priv->ts_tx_en &&
			skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)) {
			struct skb_shared_hwtstamps shhwtstamps;

			dpa_get_ts(priv, TX, &shhwtstamps, (void *)skbh);
			skb_tstamp_tx(skb, &shhwtstamps);
		}
#endif /* CONFIG_FSL_DPAA_TS */

		/* sgt[0] is from lowmem, was dma_map_single()-ed */
		dma_unmap_single(dpa_bp->dev, sgt[0].addr,
				sgt[0].length, dma_dir);

		/* remaining pages were mapped with dma_map_page() */
		for (i = 1; i < nr_frags; i++) {
			BUG_ON(sgt[i].extension);

			dma_unmap_page(dpa_bp->dev, sgt[i].addr,
					dpa_bp->size, dma_dir);
		}

		/*
		 * TODO: dpa_bp_add_page() ?
		 * We could put these in the pool, since we allocated them
		 * and we know they're not used by anyone else
		 */

		/* Free the page frag that we allocated on Tx */
		put_page(virt_to_head_page(sgt));
	}
#if defined(CONFIG_FSL_DPAA_1588) || defined(CONFIG_FSL_DPAA_TS)
	else {
		/* get the timestamp for non-SG frames */
#ifdef CONFIG_FSL_DPAA_1588
		if (priv->tsu && priv->tsu->valid &&
						priv->tsu->hwts_tx_en_ioctl)
			dpa_ptp_store_txstamp(priv, skb, (void *)skbh);
#endif
#ifdef CONFIG_FSL_DPAA_TS
		if (unlikely(priv->ts_tx_en &&
				skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)) {
			struct skb_shared_hwtstamps shhwtstamps;

			dpa_get_ts(priv, TX, &shhwtstamps, (void *)skbh);
			skb_tstamp_tx(skb, &shhwtstamps);
		}
#endif
	}
#endif

	return skb;
}

/*
 * Build a linear skb around the received buffer.
 * We are guaranteed there is enough room at the end of the data buffer to
 * accomodate the shared info area of the skb.
 */
static struct sk_buff *__hot contig_fd_to_skb(const struct dpa_priv_s *priv,
	const struct qm_fd *fd, int *use_gro)
{
	dma_addr_t addr = qm_fd_addr(fd);
	void *vaddr;
	struct dpa_bp *dpa_bp = priv->dpa_bp;
	const t_FmPrsResult *parse_results;
	struct sk_buff *skb = NULL;

	vaddr = phys_to_virt(addr);

	/* do we need the timestamp for bad frames? */
#ifdef CONFIG_FSL_DPAA_1588
	if (priv->tsu && priv->tsu->valid && priv->tsu->hwts_rx_en_ioctl)
		dpa_ptp_store_rxstamp(priv, skb, vaddr);
#endif

	/* Build the skb and adjust data and tail pointers */
	skb = build_skb(vaddr, dpa_bp->size + DPA_SKB_TAILROOM);
	if (unlikely(!skb))
		return NULL;

	/* Make sure forwarded skbs will have enough space on Tx,
	 * if extra headers are added.
	 */
	skb_reserve(skb, priv->tx_headroom + dpa_get_rx_extra_headroom());
	skb_put(skb, dpa_fd_length(fd));

	/* Peek at the parse results for csum validation */
	parse_results = (const t_FmPrsResult *)(vaddr + DPA_RX_PRIV_DATA_SIZE);
	_dpa_process_parse_results(parse_results, fd, skb, use_gro);

#ifdef CONFIG_FSL_DPAA_TS
	if (priv->ts_rx_en)
		dpa_get_ts(priv, RX, skb_hwtstamps(skb), vaddr);
#endif /* CONFIG_FSL_DPAA_TS */

	return skb;
}


/*
 * Build an skb with the data of the first S/G entry in the linear portion and
 * the rest of the frame as skb fragments.
 *
 * The page holding the S/G Table is recycled here.
 */
static struct sk_buff *__hot sg_fd_to_skb(const struct dpa_priv_s *priv,
			       const struct qm_fd *fd, int *use_gro)
{
	const struct qm_sg_entry *sgt;
	dma_addr_t addr = qm_fd_addr(fd);
	dma_addr_t sg_addr;
	void *vaddr, *sg_vaddr;
	struct dpa_bp *dpa_bp;
	struct page *page;
	int frag_offset, frag_len;
	int page_offset;
	int i;
	const t_FmPrsResult *parse_results;
	struct sk_buff *skb = NULL;

	vaddr = phys_to_virt(addr);
#ifdef CONFIG_FSL_DPAA_1588
	if (priv->tsu && priv->tsu->valid && priv->tsu->hwts_rx_en_ioctl)
		dpa_ptp_store_rxstamp(priv, skb, vaddr);
#endif

#ifdef CONFIG_FSL_DPAA_TS
	if (priv->ts_rx_en)
		dpa_get_ts(priv, RX, skb_hwtstamps(skb), vaddr);
#endif /* CONFIG_FSL_DPAA_TS */

	/* Iterate through the SGT entries and add data buffers to the skb */
	sgt = vaddr + dpa_fd_offset(fd);
	for (i = 0; i < DPA_SGT_MAX_ENTRIES; i++) {
		/* Extension bit is not supported */
		BUG_ON(sgt[i].extension);

		dpa_bp = dpa_bpid2pool(sgt[i].bpid);
		BUG_ON(IS_ERR(dpa_bp));

		sg_addr = qm_sg_addr(&sgt[i]);
		sg_vaddr = phys_to_virt(sg_addr);

		dpa_bp_removed_one_page(dpa_bp, sg_addr);

		if (i == 0) {
			/* This is the first S/G entry, so build the skb
			 * around its data buffer
			 */
			skb = build_skb(sg_vaddr,
					dpa_bp->size + DPA_SKB_TAILROOM);
			if (unlikely(!skb))
				return NULL;

			/* In the case of a SG frame, FMan stores the Internal
			 * Context in the buffer containing the sgt.
			 * Inspect the parse results before anything else.
			 */
			parse_results = (const t_FmPrsResult *)(vaddr +
						DPA_RX_PRIV_DATA_SIZE);
			_dpa_process_parse_results(parse_results, fd, skb,
						   use_gro);

			/* Make sure forwarded skbs will have enough space
			 * on Tx, if extra headers are added.
			 */
			skb_reserve(skb, priv->tx_headroom +
				dpa_get_rx_extra_headroom());
			skb_put(skb, sgt[i].length);
		} else {
			/*
			 * Not the first S/G entry; all data from buffer will
			 * be added in an skb fragment; fragment index is offset
			 * by one since first S/G entry was incorporated in the
			 * linear part of the skb.
			 */
			page = pfn_to_page(sg_addr >> PAGE_SHIFT);
			page_offset = (unsigned long)sg_vaddr & (PAGE_SIZE - 1);
			frag_offset = sgt[i].offset + page_offset;
			frag_len = sgt[i].length;
			/* TODO kernel 3.8 fixup; we might want to account for
			 * the true-truesize.
			 */
			skb_add_rx_frag(skb, i - 1, page, frag_offset, frag_len,
					frag_len);
		}

		if (sgt[i].final)
			break;
	}

	/* recycle the SGT page */
	dpa_bp = dpa_bpid2pool(fd->bpid);
	BUG_ON(IS_ERR(dpa_bp));
	dpa_bp_add_page(dpa_bp, (unsigned long)vaddr);
	return skb;
}

void __hot _dpa_rx(struct net_device *net_dev,
		const struct dpa_priv_s *priv,
		struct dpa_percpu_priv_s *percpu_priv,
		const struct qm_fd *fd,
		u32 fqid)
{
	struct dpa_bp *dpa_bp;
	struct sk_buff *skb;
	dma_addr_t addr = qm_fd_addr(fd);
	u32 fd_status = fd->status;
	unsigned int skb_len;
	struct rtnl_link_stats64 *percpu_stats = &percpu_priv->stats;
	int use_gro = net_dev->features & NETIF_F_GRO;

	if (unlikely(fd_status & FM_FD_STAT_ERRORS) != 0) {
		if (netif_msg_hw(priv) && net_ratelimit())
			netdev_warn(net_dev, "FD status = 0x%08x\n",
					fd_status & FM_FD_STAT_ERRORS);

		percpu_stats->rx_errors++;
		goto _release_frame;
	}

	dpa_bp = dpa_bpid2pool(fd->bpid);
	dpa_bp_removed_one_page(dpa_bp, addr);

	/* prefetch the first 64 bytes of the frame or the SGT start */
	prefetch(phys_to_virt(addr) + dpa_fd_offset(fd));

	if (likely(fd->format == qm_fd_contig))
		skb = contig_fd_to_skb(priv, fd, &use_gro);
	else if (fd->format == qm_fd_sg)
		skb = sg_fd_to_skb(priv, fd, &use_gro);
	else
		/* The only FD types that we may receive are contig and S/G */
		BUG();
	if (unlikely(!skb))
		goto _release_frame;

	skb->protocol = eth_type_trans(skb, net_dev);

	/* IP Reassembled frames are allowed to be larger than MTU */
	if (unlikely(dpa_check_rx_mtu(skb, net_dev->mtu) &&
		!(fd_status & FM_FD_IPR))) {
		percpu_stats->rx_dropped++;
		goto drop_bad_frame;
	}

	skb_len = skb->len;

	if (use_gro) {
		gro_result_t gro_result;

		gro_result = napi_gro_receive(&percpu_priv->napi, skb);
		if (unlikely(gro_result == GRO_DROP)) {
			percpu_stats->rx_dropped++;
			goto packet_dropped;
		}
	} else if (unlikely(netif_receive_skb(skb) == NET_RX_DROP)) {
		percpu_stats->rx_dropped++;
		goto packet_dropped;
	}

	percpu_stats->rx_packets++;
	percpu_stats->rx_bytes += skb_len;

packet_dropped:
	return;

drop_bad_frame:
	dev_kfree_skb(skb);
	return;

_release_frame:
	dpa_fd_release(net_dev, fd);
}

static int __hot skb_to_contig_fd(struct dpa_priv_s *priv,
				  struct sk_buff *skb, struct qm_fd *fd)
{
	struct sk_buff **skbh;
	dma_addr_t addr;
	struct dpa_bp *dpa_bp;
	struct net_device *net_dev = priv->net_dev;
	int err;

	/* We are guaranteed that we have at least tx_headroom bytes */
	skbh = (struct sk_buff **)(skb->data - priv->tx_headroom);

	*skbh = skb;

	dpa_bp = priv->dpa_bp;

	/*
	 * Enable L3/L4 hardware checksum computation.
	 *
	 * We must do this before dma_map_single(DMA_TO_DEVICE), because we may
	 * need to write into the skb.
	 */
	err = dpa_enable_tx_csum(priv, skb, fd,
				 ((char *)skbh) + DPA_TX_PRIV_DATA_SIZE);
	if (unlikely(err < 0)) {
		if (netif_msg_tx_err(priv) && net_ratelimit())
			netdev_err(net_dev, "HW csum error: %d\n", err);
		return err;
	}

	/* Fill in the FD */
	fd->format = qm_fd_contig;
	fd->length20 = skb->len;
	fd->offset = priv->tx_headroom; /* This is now guaranteed */

	addr = dma_map_single(dpa_bp->dev, skbh, dpa_bp->size, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dpa_bp->dev, addr))) {
		if (netif_msg_tx_err(priv) && net_ratelimit())
			netdev_err(net_dev, "dma_map_single() failed\n");
		return -EINVAL;
	}
	fd->addr_hi = upper_32_bits(addr);
	fd->addr_lo = lower_32_bits(addr);

	return 0;
}

static int __hot skb_to_sg_fd(struct dpa_priv_s *priv,
			      struct sk_buff *skb, struct qm_fd *fd)
{
	struct dpa_bp *dpa_bp = priv->dpa_bp;
	dma_addr_t addr;
	struct sk_buff **skbh;
	struct net_device *net_dev = priv->net_dev;
	int err;

	struct qm_sg_entry *sgt;
	void *sgt_buf;
	void *buffer_start;
	skb_frag_t *frag;
	int i, j;
	const enum dma_data_direction dma_dir = DMA_TO_DEVICE;
	const int nr_frags = skb_shinfo(skb)->nr_frags;

	fd->format = qm_fd_sg;

	/* get a page frag to store the SGTable */
	sgt_buf = netdev_alloc_frag(priv->tx_headroom +
		sizeof(struct qm_sg_entry) * (1 + nr_frags));
	if (unlikely(!sgt_buf)) {
		dev_err(dpa_bp->dev, "netdev_alloc_frag() failed\n");
		return -ENOMEM;
	}

	/*
	 * Enable L3/L4 hardware checksum computation.
	 *
	 * We must do this before dma_map_single(DMA_TO_DEVICE), because we may
	 * need to write into the skb.
	 */
	err = dpa_enable_tx_csum(priv, skb, fd,
				 sgt_buf + DPA_TX_PRIV_DATA_SIZE);
	if (unlikely(err < 0)) {
		if (netif_msg_tx_err(priv) && net_ratelimit())
			netdev_err(net_dev, "HW csum error: %d\n", err);
		goto csum_failed;
	}

	sgt = (struct qm_sg_entry *)(sgt_buf + priv->tx_headroom);
	sgt[0].bpid = dpa_bp->bpid;
	sgt[0].offset = 0;
	sgt[0].length = skb_headlen(skb);
	sgt[0].extension = 0;
	sgt[0].final = 0;
	addr = dma_map_single(dpa_bp->dev, skb->data, sgt[0].length, dma_dir);
	if (unlikely(dma_mapping_error(dpa_bp->dev, addr))) {
		dev_err(dpa_bp->dev, "DMA mapping failed");
		err = -EINVAL;
		goto sg0_map_failed;

	}
	sgt[0].addr_hi = upper_32_bits(addr);
	sgt[0].addr_lo = lower_32_bits(addr);

	/* populate the rest of SGT entries */
	for (i = 1; i <= nr_frags; i++) {
		frag = &skb_shinfo(skb)->frags[i - 1];
		sgt[i].bpid = dpa_bp->bpid;
		sgt[i].offset = 0;
		sgt[i].length = frag->size;
		sgt[i].extension = 0;
		sgt[i].final = 0;

		/* This shouldn't happen */
		BUG_ON(!frag->page.p);

		addr = skb_frag_dma_map(dpa_bp->dev, frag, 0, dpa_bp->size,
					dma_dir);

		if (unlikely(dma_mapping_error(dpa_bp->dev, addr))) {
			dev_err(dpa_bp->dev, "DMA mapping failed");
			err = -EINVAL;
			goto sg_map_failed;
		}

		/* keep the offset in the address */
		sgt[i].addr_hi = upper_32_bits(addr);
		sgt[i].addr_lo = lower_32_bits(addr);
	}
	sgt[i - 1].final = 1;

	fd->length20 = skb->len;
	fd->offset = priv->tx_headroom;

	/* DMA map the SGT page */
	buffer_start = (void *)sgt - dpa_fd_offset(fd);
	skbh = (struct sk_buff **)buffer_start;
	*skbh = skb;

	addr = dma_map_single(dpa_bp->dev, buffer_start, dpa_bp->size, dma_dir);
	if (unlikely(dma_mapping_error(dpa_bp->dev, addr))) {
		dev_err(dpa_bp->dev, "DMA mapping failed");
		err = -EINVAL;
		goto sgt_map_failed;
	}
	fd->addr_hi = upper_32_bits(addr);
	fd->addr_lo = lower_32_bits(addr);

	return 0;

sgt_map_failed:
sg_map_failed:
	for (j = 0; j < i; j++)
		dma_unmap_page(dpa_bp->dev, qm_sg_addr(&sgt[j]),
			dpa_bp->size, dma_dir);
sg0_map_failed:
csum_failed:
	put_page(virt_to_head_page(sgt_buf));

	return err;
}

int __hot dpa_tx(struct sk_buff *skb, struct net_device *net_dev)
{
	struct dpa_priv_s	*priv;
	struct qm_fd		 fd;
	struct dpa_percpu_priv_s *percpu_priv;
	struct rtnl_link_stats64 *percpu_stats;
	int err = 0;
	const int queue_mapping = dpa_get_queue_mapping(skb);
	const bool nonlinear = skb_is_nonlinear(skb);

	priv = netdev_priv(net_dev);
	/* Non-migratable context, safe to use __this_cpu_ptr */
	percpu_priv = __this_cpu_ptr(priv->percpu_priv);
	percpu_stats = &percpu_priv->stats;

	clear_fd(&fd);

#ifdef CONFIG_FSL_DPAA_1588
	if (priv->tsu && priv->tsu->valid && priv->tsu->hwts_tx_en_ioctl)
		fd.cmd |= FM_FD_CMD_UPD;
#endif
#ifdef CONFIG_FSL_DPAA_TS
	if (unlikely(priv->ts_tx_en &&
			skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP))
		fd.cmd |= FM_FD_CMD_UPD;
	skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
#endif /* CONFIG_FSL_DPAA_TS */

	/*
	 * MAX_SKB_FRAGS is larger than our DPA_SGT_MAX_ENTRIES; make sure
	 * we don't feed FMan with more fragments than it supports.
	 * Btw, we're using the first sgt entry to store the linear part of
	 * the skb, so we're one extra frag short.
	 */
	if (nonlinear &&
		likely(skb_shinfo(skb)->nr_frags < DPA_SGT_MAX_ENTRIES)) {
		/* Just create a S/G fd based on the skb */
		err = skb_to_sg_fd(priv, skb, &fd);
		percpu_priv->tx_frag_skbuffs++;
	} else {
		/*
		 * Make sure we have enough headroom to accomodate private
		 * data, parse results, etc. Normally this shouldn't happen if
		 * we're here via the standard kernel stack.
		 */
		if (unlikely(skb_headroom(skb) < priv->tx_headroom)) {
			struct sk_buff *skb_new;

			skb_new = skb_realloc_headroom(skb, priv->tx_headroom);
			if (unlikely(!skb_new)) {
				dev_kfree_skb(skb);
				percpu_stats->tx_errors++;
				return NETDEV_TX_OK;
			}
			dev_kfree_skb(skb);
			skb = skb_new;
		}

		/*
		 * We're going to store the skb backpointer at the beginning
		 * of the data buffer, so we need a privately owned skb
		 */

		/* Code borrowed from skb_unshare(). */
		if (skb_cloned(skb)) {
			struct sk_buff *nskb = skb_copy(skb, GFP_ATOMIC);
			kfree_skb(skb);
			skb = nskb;
			/* skb_copy() has now linearized the skbuff. */
		} else if (unlikely(nonlinear)) {
			/*
			 * We are here because the egress skb contains
			 * more fragments than we support. In this case,
			 * we have no choice but to linearize it ourselves.
			 */
			err = __skb_linearize(skb);
		}
		if (unlikely(!skb || err < 0))
			/* Common out-of-memory error path */
			goto enomem;

		/* Finally, create a contig FD from this skb */
		err = skb_to_contig_fd(priv, skb, &fd);
	}
	if (unlikely(err < 0))
		goto skb_to_fd_failed;

	if (unlikely(dpa_xmit(priv, percpu_stats, queue_mapping, &fd) < 0))
		goto xmit_failed;

	net_dev->trans_start = jiffies;

	return NETDEV_TX_OK;

xmit_failed:
	_dpa_cleanup_tx_fd(priv, &fd);
skb_to_fd_failed:
enomem:
	percpu_stats->tx_errors++;
	dev_kfree_skb(skb);
	return NETDEV_TX_OK;
}

#endif /* CONFIG_FSL_DPAA_ETH_SG_SUPPORT */
