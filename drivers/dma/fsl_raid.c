/*
 * drivers/dma/fsl_raid.c
 *
 * Freescale RAID Engine device driver
 *
 * Author:
 *	Harninder Rai <harninder.rai@freescale.com>
 *	Naveen Burmi <naveenburmi@freescale.com>
 *
 * Copyright (c) 2010-2013 Freescale Semiconductor, Inc.
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
 * Theory of operation:
 *
 * General capabilities:
 *	RAID Engine (RE) block is capable of offloading XOR, memcpy and P/Q
 *	calculations required in RAID5 and RAID6 operations. RE driver
 *	registers with Linux's ASYNC layer as dma driver. RE hardware
 *	maintains strict ordering of the requests through chained
 *	command queueing.
 *
 * Data flow:
 *	Software RAID layer of Linux (MD layer) maintains RAID partitions,
 *	strips, stripes etc. It sends requests to the underlying AYSNC layer
 *	which further passes it to RE driver. ASYNC layer decides which request
 *	goes to which job ring of RE hardware. For every request processed by
 *	RAID Engine, driver gets an interrupt unless coalescing is set. The
 *	per job ring interrupt handler checks the status register for errors,
 *	clears the interrupt and schedules a tasklet. Main request processing
 *	is done in tasklet. A software shadow copy of the HW ring is kept to
 *	maintain virtual to physical translation. Based on the internal indexes
 *	maintained, the tasklet picks the descriptor address from shadow copy,
 *	updates the corresponding cookie, updates the outbound ring job removed
 *	register in RE hardware and eventually calls the callback function. This
 *	callback function gets passed as part of request from MD layer.
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/slab.h>

#include "dmaengine.h"
#include "fsl_raid.h"

#define MAX_XOR_SRCS		16
#define MAX_PQ_SRCS		16
#define MAX_INITIAL_DESCS	256
#define FRAME_FORMAT		0x1
#define MAX_DATA_LENGTH		(1024*1024)

#define to_fsl_re_dma_desc(tx) container_of(tx, \
		struct fsl_re_dma_async_tx_desc, async_tx)

/* Add descriptors into per jr software queue - submit_q */
static dma_cookie_t re_jr_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct fsl_re_dma_async_tx_desc *desc = NULL;
	struct re_jr *jr = NULL;
	dma_cookie_t cookie;

	desc = container_of(tx, struct fsl_re_dma_async_tx_desc, async_tx);
	jr = container_of(tx->chan, struct re_jr, chan);

	spin_lock_bh(&jr->desc_lock);

	jr->timer.data = (unsigned long)tx->chan;
	cookie = jr->chan.cookie + 1;
	if (cookie < 0)
		cookie = 1;

	desc->async_tx.cookie = cookie;
	jr->chan.cookie = desc->async_tx.cookie;
	jr->pend_count++;

	if (!timer_pending(&jr->timer))
		add_timer(&jr->timer);

	spin_unlock_bh(&jr->desc_lock);

	return cookie;
}

static void re_jr_unmap_dest_src(struct fsl_re_dma_async_tx_desc *desc)
{
	int i, j;
	struct cmpnd_frame *cf;
	dma_addr_t dest1 = 0, dest2 = 0, src;
	struct device *dev;
	enum dma_ctrl_flags flags;
	enum dma_data_direction dir;

	cf = desc->cf_addr;
	dest1 = cf[1].address;
	j = 2;
	if (desc->dest_cnt == 2) {
		dest2 = cf[2].address;
		j = 3;
	}
	dev = desc->jr->chan.device->dev;
	flags = desc->async_tx.flags;
	if (!(flags & DMA_COMPL_SKIP_DEST_UNMAP)) {
		if (desc->cdb_opcode == RE_MOVE_OPCODE)
			dir = DMA_FROM_DEVICE;
		else
			dir = DMA_BIDIRECTIONAL;

		dma_unmap_page(dev, dest1, desc->dma_len, dir);

		if (dest2)
			dma_unmap_page(dev, dest2, desc->dma_len, dir);
	}

	if (!(flags & DMA_COMPL_SKIP_SRC_UNMAP)) {
		dir = DMA_TO_DEVICE;
		for (i = j; i < desc->src_cnt + j; i++) {
			src = cf[i].address;
			if (src == dest1 || src == dest2)
				continue;
			dma_unmap_page(dev, src, desc->dma_len, dir);
		}
	}
}

static void re_jr_desc_done(struct fsl_re_dma_async_tx_desc *desc)
{
	struct dma_chan *chan = &desc->jr->chan;
	dma_async_tx_callback callback;
	void *callback_param;

	callback = desc->async_tx.callback;
	callback_param = desc->async_tx.callback_param;

	dma_run_dependencies(&desc->async_tx);

	if (chan->completed_cookie < desc->async_tx.cookie) {
		chan->completed_cookie = desc->async_tx.cookie;
		if (chan->completed_cookie == DMA_MAX_COOKIE)
			chan->completed_cookie = DMA_MIN_COOKIE;
	}
	re_jr_unmap_dest_src(desc);

	if (callback)
		callback(callback_param);
}

/*
 * Get the virtual address of software desc from virt_addr.
 * Storing the address of software desc like this makes the
 * order of alogorithm as O(1)
 */
static void re_jr_dequeue(unsigned long data)
{
	struct device *dev;
	struct re_jr *jr;
	struct fsl_re_dma_async_tx_desc *desc;
	unsigned int count;
	struct fsl_re_dma_async_tx_desc *ack_desc = NULL, *_ack_desc = NULL;

	dev = (struct device *)data;
	jr = dev_get_drvdata(dev);

	spin_lock_bh(&jr->desc_lock);
	count =	RE_JR_OUB_SLOT_FULL(in_be32(&jr->jrregs->oubring_slot_full));
	if (count) {
		out_be32(&jr->jrregs->oubring_job_rmvd,
			 RE_JR_OUB_JOB_REMOVE(count));
		while (count--) {
			jr->oub_count &= RING_SIZE_MASK;
			desc = &jr->descs[jr->oub_count++];
			list_add_tail(&desc->node, &jr->ack_q);
			re_jr_desc_done(desc);
		}
	}
	spin_unlock_bh(&jr->desc_lock);

	/* To save memory, parse the ack_q and free up descs */
	list_for_each_entry_safe(ack_desc, _ack_desc, &jr->ack_q, node) {
		if (async_tx_test_ack(&ack_desc->async_tx)) {
			spin_lock_bh(&jr->desc_lock);
			list_del(&ack_desc->node);
			ack_desc->state = RE_DESC_EMPTY;
			ack_desc->async_tx.flags = 0;
			spin_unlock_bh(&jr->desc_lock);
		}
	}
}

/* Per Job Ring interrupt handler */
static irqreturn_t re_jr_interrupt(int irq, void *data)
{
	struct device *dev = data;
	struct re_jr *jr = dev_get_drvdata(dev);

	u32 irqstate, status;
	irqstate = in_be32(&jr->jrregs->jr_interrupt_status);
	if (!irqstate)
		return IRQ_NONE;

	/*
	 * There's no way in upper layer (read MD layer) to recover from
	 * error conditions except restart everything. In long term we
	 * need to do something more than just crashing
	 */
	if (irqstate & RE_JR_ERROR) {
		status = in_be32(&jr->jrregs->jr_status);
		dev_err(dev, "jr error irqstate: %x, status: %x\n", irqstate,
			status);
	}

	/* Clear interrupt */
	out_be32(&jr->jrregs->jr_interrupt_status, RE_JR_CLEAR_INT);

	tasklet_schedule(&jr->irqtask);
	return IRQ_HANDLED;
}

static enum dma_status re_jr_tx_status(struct dma_chan *chan,
		dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	return dma_cookie_status(chan, cookie, txstate);
}


/* Copy descriptor from per jr software queue into hardware job ring */
void re_jr_issue_pending(struct dma_chan *chan)
{
	struct re_jr *jr = NULL;
	int avail = 0;

	jr = container_of(chan, struct re_jr, chan);
	if (timer_pending(&jr->timer))
		del_timer_sync(&jr->timer);

	spin_lock_bh(&jr->desc_lock);
	if (!jr->pend_count)
		goto out_unlock;

	avail = RE_JR_INB_SLOT_AVAIL(in_be32(&jr->jrregs->inbring_slot_avail));
	if (!avail)
		goto out_unlock;

	if (avail > jr->pend_count)
		avail = jr->pend_count;

	jr->pend_count -= avail;
	jr->inb_count = (jr->inb_count + avail) & RING_SIZE_MASK;

	/* add jobs into job ring */
	out_be32(&jr->jrregs->inbring_add_job, RE_JR_INB_JOB_ADD(avail));

out_unlock:
	spin_unlock_bh(&jr->desc_lock);
}

/* Per Job Ring timer handler */
static void raide_timer_handler(unsigned long data)
{
	struct dma_chan *chan = NULL;
	chan = (struct dma_chan *)data;

	re_jr_issue_pending(chan);

	return;
}

void fill_cfd_frame(struct cmpnd_frame *cf, u8 index,
		size_t length, dma_addr_t addr, bool final)
{
	cf[index].final = final;
	cf[index].length = length;
	cf[index].address = addr;
}

static struct fsl_re_dma_async_tx_desc *re_jr_init_desc(struct re_jr *jr,
	struct fsl_re_dma_async_tx_desc *desc, void *cf, dma_addr_t paddr)
{
	desc->jr = jr;
	desc->async_tx.tx_submit = re_jr_tx_submit;
	dma_async_tx_descriptor_init(&desc->async_tx, &jr->chan);
	INIT_LIST_HEAD(&desc->node);

	desc->hwdesc->format = FRAME_FORMAT;
	desc->hwdesc->address = paddr;
	desc->cf_addr = cf;

	desc->cdb_addr = (void *)(cf + RE_CF_DESC_SIZE);
	desc->cdb_paddr = paddr + RE_CF_DESC_SIZE;

	return desc;
}

static struct fsl_re_dma_async_tx_desc *re_jr_alloc_desc(struct re_jr *jr,
		unsigned long flags)
{
	struct fsl_re_dma_async_tx_desc *desc;

	spin_lock_bh(&jr->desc_lock);
	desc = &jr->descs[jr->inb_count];
	if (desc->state != RE_DESC_EMPTY) {
		spin_unlock_bh(&jr->desc_lock);
		re_jr_issue_pending(&jr->chan);
		return NULL;
	}
	desc->state = RE_DESC_ALLOC;
	desc->async_tx.flags = flags;
	spin_unlock_bh(&jr->desc_lock);

	return desc;
}

static struct dma_async_tx_descriptor *re_jr_prep_genq(
		struct dma_chan *chan, dma_addr_t dest, dma_addr_t *src,
		unsigned int src_cnt, const unsigned char *scf, size_t len,
		unsigned long flags)
{
	struct re_jr *jr;
	struct fsl_re_dma_async_tx_desc *desc;
	struct xor_cdb *xor;
	struct cmpnd_frame *cf;
	unsigned int i, j;

	if (len > MAX_DATA_LENGTH) {
		pr_err("Length greater than %d not supported\n",
		       MAX_DATA_LENGTH);
		return NULL;
	}

	jr = container_of(chan, struct re_jr, chan);
	desc = re_jr_alloc_desc(jr, flags);
	if (desc <= 0)
		return NULL;

	desc->dma_len = len;
	desc->dest_cnt = 1;
	desc->src_cnt = src_cnt;

	desc->cdb_opcode = RE_XOR_OPCODE;
	desc->cdb_len = sizeof(struct xor_cdb);

	/* Filling xor CDB */
	xor = desc->cdb_addr;
	xor->opcode = RE_XOR_OPCODE;
	xor->nrcs = src_cnt - 1;
	xor->blk_size = RE_BLOCK_SIZE;
	xor->error_attrib = INTERRUPT_ON_ERROR;
	xor->data_depend = DATA_DEPENDENCY;

	if (scf != NULL) {
		/* compute q = src0*coef0^src1*coef1^..., * is GF(8) mult */
		for (i = 0; i < src_cnt; i++)
			xor->gfm[i] = scf[i];
	} else {
		/* compute P, that is XOR all srcs */
		for (i = 0; i < src_cnt; i++)
			xor->gfm[i] = 1;
	}

	/* Filling frame 0 of compound frame descriptor with CDB */
	cf = desc->cf_addr;
	fill_cfd_frame(cf, 0, desc->cdb_len, desc->cdb_paddr, 0);

	/* Fill CFD's 1st frame with dest buffer */
	fill_cfd_frame(cf, 1, len, dest, 0);

	/* Fill CFD's rest of the frames with source buffers */
	for (i = 2, j = 0; j < src_cnt; i++, j++)
		fill_cfd_frame(cf, i, len, src[j], 0);

	/* Setting the final bit in the last source buffer frame in CFD */
	cf[i - 1].final = 1;

	return &desc->async_tx;
}

/*
 * Prep function for P parity calculation.In RAID Engine terminology,
 * XOR calculation is called GenQ calculation done through GenQ command
 */
static struct dma_async_tx_descriptor *re_jr_prep_dma_xor(
		struct dma_chan *chan, dma_addr_t dest, dma_addr_t *src,
		unsigned int src_cnt, size_t len, unsigned long flags)
{
	/* NULL let genq take all coef as 1 */
	return re_jr_prep_genq(chan, dest, src, src_cnt, NULL, len, flags);
}

/*
 * Prep function for P/Q parity calculation.In RAID Engine terminology,
 * P/Q calculation is called GenQQ done through GenQQ command
 */
static struct dma_async_tx_descriptor *re_jr_prep_pq(
		struct dma_chan *chan, dma_addr_t *dest, dma_addr_t *src,
		unsigned int src_cnt, const unsigned char *scf, size_t len,
		unsigned long flags)
{
	struct re_jr *jr;
	struct fsl_re_dma_async_tx_desc *desc;
	struct pq_cdb *pq;
	struct cmpnd_frame *cf;
	u8 *p;
	int gfmq_len, i, j;

	if (len > MAX_DATA_LENGTH) {
		pr_err("Length greater than %d not supported\n",
		       MAX_DATA_LENGTH);
		return NULL;
	}

	/*
	 * RE requires at least 2 sources, if given only one source, we pass the
	 * second source same as the first one.
	 * With only one source, generating P is meaningless, only generate Q.
	 */
	if (src_cnt == 1) {
		struct dma_async_tx_descriptor *tx;
		dma_addr_t dma_src[2];
		unsigned char coef[2];

		dma_src[0] = *src;
		coef[0] = *scf;
		dma_src[1] = *src;
		coef[1] = 0;
		tx = re_jr_prep_genq(chan, dest[1], dma_src, 2, coef, len,
				flags);
		if (tx) {
			desc = to_fsl_re_dma_desc(tx);
			desc->src_cnt = 1;
		}
		return tx;
	}

	/*
	 * During RAID6 array creation, Linux's MD layer gets P and Q
	 * calculated separately in two steps. But our RAID Engine has
	 * the capability to calculate both P and Q with a single command
	 * Hence to merge well with MD layer, we need to provide a hook
	 * here and call re_jq_prep_genq() function
	 */

	if (flags & DMA_PREP_PQ_DISABLE_P)
		return re_jr_prep_genq(chan, dest[1], src, src_cnt,
				scf, len, flags);

	jr = container_of(chan, struct re_jr, chan);
	desc = re_jr_alloc_desc(jr, flags);
	if (desc <= 0)
		return NULL;

	desc->dma_len = len;
	desc->dest_cnt = 2;
	desc->src_cnt = src_cnt;

	desc->cdb_opcode = RE_PQ_OPCODE;
	desc->cdb_len = sizeof(struct pq_cdb);

	/* Filling GenQQ CDB */
	pq = desc->cdb_addr;
	pq->opcode = RE_PQ_OPCODE;
	pq->blk_size = RE_BLOCK_SIZE;
	pq->buffer_attrib = BUFFERABLE_OUTPUT;
	pq->data_depend = DATA_DEPENDENCY;
	pq->nrcs = (src_cnt - 1);

	p = pq->gfm_q1;
	/* Init gfm_q1[] */
	for (i = 0; i < src_cnt; i++)
		p[i] = 1;

	/* Align gfm[] to 32bit */
	gfmq_len = ALIGN(src_cnt, 4);

	/* Init gfm_q2[] */
	p += gfmq_len;
	for (i = 0; i < src_cnt; i++)
		p[i] = scf[i];

	/* Filling frame 0 of compound frame descriptor with CDB */
	cf = desc->cf_addr;
	fill_cfd_frame(cf, 0, desc->cdb_len, desc->cdb_paddr, 0);

	/* Fill CFD's 1st & 2nd frame with dest buffers */
	for (i = 1, j = 0; i < 3; i++, j++)
		fill_cfd_frame(cf, i, len, dest[j], 0);

	/* Fill CFD's rest of the frames with source buffers */
	for (i = 3, j = 0; j < src_cnt; i++, j++)
		fill_cfd_frame(cf, i, len, src[j], 0);

	/* Setting the final bit in the last source buffer frame in CFD */
	cf[i - 1].final = 1;

	return &desc->async_tx;
}

/*
 * Prep function for memcpy. In RAID Engine, memcpy is done through MOVE
 * command. Logic of this function will need to be modified once multipage
 * support is added in Linux's MD/ASYNC Layer
 */
static struct dma_async_tx_descriptor *re_jr_prep_memcpy(
		struct dma_chan *chan, dma_addr_t dest, dma_addr_t src,
		size_t len, unsigned long flags)
{
	struct re_jr *jr;
	struct fsl_re_dma_async_tx_desc *desc;
	size_t length;
	struct cmpnd_frame *cf;
	struct move_cdb *move;

	jr = container_of(chan, struct re_jr, chan);

	if (len > MAX_DATA_LENGTH) {
		pr_err("Length greater than %d not supported\n",
		       MAX_DATA_LENGTH);
		return NULL;
	}

	desc = re_jr_alloc_desc(jr, flags);
	if (desc <= 0)
		return NULL;

	desc->dma_len = len;
	desc->src_cnt = 1;
	desc->dest_cnt = 1;

	desc->cdb_opcode = RE_MOVE_OPCODE;
	desc->cdb_len = sizeof(struct move_cdb);

	/* Filling move CDB */
	move = desc->cdb_addr;
	move->opcode = RE_MOVE_OPCODE; /* Unicast move */
	move->blk_size = RE_BLOCK_SIZE;
	move->error_attrib = INTERRUPT_ON_ERROR;
	move->data_depend = DATA_DEPENDENCY;

	/* Filling frame 0 of CFD with move CDB */
	cf = desc->cf_addr;
	fill_cfd_frame(cf, 0, desc->cdb_len, desc->cdb_paddr, 0);

	length = min_t(size_t, len, MAX_DATA_LENGTH);

	/* Fill CFD's 1st frame with dest buffer */
	fill_cfd_frame(cf, 1, length, dest, 0);

	/* Fill CFD's 2nd frame with src buffer */
	fill_cfd_frame(cf, 2, length, src, 1);

	return &desc->async_tx;
}

static int re_jr_alloc_chan_resources(struct dma_chan *chan)
{
	int i;
	struct fsl_re_dma_async_tx_desc *desc;
	struct re_jr *jr = container_of(chan, struct re_jr, chan);
	void *cf;
	dma_addr_t paddr;

	jr->descs = kzalloc(sizeof(*desc) * RING_SIZE, GFP_KERNEL);
	if (!jr->descs) {
		dev_err(jr->dev, "No memory for sw descriptor ring\n");
		goto err_free;
	}

	cf = dma_pool_alloc(jr->re_dev->desc_pool, GFP_ATOMIC, &paddr);
	if (!cf) {
		dev_err(jr->dev, "No memory for dma descriptor ring\n");
		goto err_free;
	}
	memset(cf, 0, RE_CF_CDB_SIZE * RING_SIZE);
	jr->cfs = cf;
	jr->phys = paddr;

	for (i = 0; i < RING_SIZE; i++) {
		u32 offset = i * RE_CF_CDB_SIZE;
		desc = &jr->descs[i];
		desc->hwdesc = &jr->inb_ring_virt_addr[i];
		re_jr_init_desc(jr, desc, cf + offset, paddr + offset);
		desc->state = RE_DESC_EMPTY;
	}
	return 0;

err_free:
	kfree(jr->descs);
	return -ENOMEM;
}

static void re_jr_free_chan_resources(struct dma_chan *chan)
{
	struct re_jr *jr = container_of(chan, struct re_jr, chan);
	dma_pool_free(jr->re_dev->desc_pool, jr->cfs, jr->phys);
	kfree(jr->descs);
	return;
}

int re_jr_probe(struct platform_device *ofdev,
		struct device_node *np, u8 q, u32 off)
{
	struct device *dev;
	struct re_drv_private *repriv;
	struct re_jr *jr;
	struct dma_device *dma_dev;
	u32 ptr;
	u32 status;
	int ret = 0, rc;
	struct platform_device *jr_ofdev;

	dev = &ofdev->dev;
	repriv = dev_get_drvdata(dev);
	dma_dev = &repriv->dma_dev;

	jr = kzalloc(sizeof(struct re_jr), GFP_KERNEL);
	if (!jr) {
		dev_err(dev, "No free memory for allocating JR struct\n");
		return -ENOMEM;
	}

	jr_ofdev = of_platform_device_create(np, NULL, dev);
	if (jr_ofdev == NULL) {
		dev_err(dev, "Not able to create ofdev for jr %d\n", q);
		ret = -EINVAL;
		goto err_free;
	}
	dev_set_drvdata(&jr_ofdev->dev, jr);

	rc = of_property_read_u32(np, "reg", &ptr);
	if (rc) {
		dev_err(dev, "Reg property not found in JR number %d\n", q);
		ret = -ENODEV;
		goto err_free;
	}

	jr->jrregs = (struct jr_config_regs *)((u8 *)repriv->re_regs +
			off + ptr);

	jr->irq = irq_of_parse_and_map(np, 0);
	if (jr->irq == NO_IRQ) {
		dev_err(dev, "No IRQ defined for JR %d\n", q);
		ret = -ENODEV;
		goto err_free;
	}

	tasklet_init(&jr->irqtask, re_jr_dequeue,
		     (unsigned long)&jr_ofdev->dev);

	ret = request_irq(jr->irq, re_jr_interrupt, 0, "re-jr", &jr_ofdev->dev);
	if (ret) {
		dev_err(dev, "Unable to register JR interrupt for JR %d\n", q);
		ret = -EINVAL;
		goto err_free;
	}

	repriv->re_jrs[q] = jr;
	jr->chan.device = dma_dev;
	jr->chan.private = jr;
	jr->dev = &jr_ofdev->dev;
	jr->re_dev = repriv;
	jr->pend_count = 0;
	INIT_LIST_HEAD(&jr->ack_q);
	spin_lock_init(&jr->desc_lock);

	init_timer(&jr->timer);
	jr->timer.expires = jiffies + 10*HZ;
	jr->timer.function = raide_timer_handler;

	list_add_tail(&jr->chan.device_node, &dma_dev->channels);
	dma_dev->chancnt++;

	jr->inb_ring_virt_addr = dma_pool_alloc(jr->re_dev->hw_desc_pool,
		GFP_ATOMIC, &jr->inb_phys_addr);

	if (!jr->inb_ring_virt_addr) {
		dev_err(dev, "No dma memory for inb_ring_virt_addr\n");
		ret = -ENOMEM;
		goto err_free;
	}

	jr->oub_ring_virt_addr = dma_pool_alloc(jr->re_dev->hw_desc_pool,
		GFP_ATOMIC, &jr->oub_phys_addr);

	if (!jr->oub_ring_virt_addr) {
		dev_err(dev, "No dma memory for oub_ring_virt_addr\n");
		ret = -ENOMEM;
		goto err_free_1;
	}

	jr->inb_count = 0;
	jr->pend_count = 0;
	jr->oub_count = 0;

	/* Program the Inbound/Outbound ring base addresses and size */
	out_be32(&jr->jrregs->inbring_base_h,
		 jr->inb_phys_addr & RE_JR_ADDRESS_BIT_MASK);
	out_be32(&jr->jrregs->oubring_base_h,
		 jr->oub_phys_addr & RE_JR_ADDRESS_BIT_MASK);
	out_be32(&jr->jrregs->inbring_base_l,
		 jr->inb_phys_addr >> RE_JR_ADDRESS_BIT_SHIFT);
	out_be32(&jr->jrregs->oubring_base_l,
		 jr->oub_phys_addr >> RE_JR_ADDRESS_BIT_SHIFT);
	out_be32(&jr->jrregs->inbring_size, RING_SIZE << RING_SIZE_SHIFT);
	out_be32(&jr->jrregs->oubring_size, RING_SIZE << RING_SIZE_SHIFT);

	/* Read LIODN value from u-boot */
	status = in_be32(&jr->jrregs->jr_config_1) & RE_JR_REG_LIODN_MASK;

	/* Program the CFG reg */
	out_be32(&jr->jrregs->jr_config_1,
		 RE_JR_CFG1_CBSI | RE_JR_CFG1_CBS0 | status);

	/* Enable RE/JR */
	out_be32(&jr->jrregs->jr_command, RE_JR_ENABLE);

	return 0;

err_free_1:
	dma_pool_free(jr->re_dev->hw_desc_pool, jr->inb_ring_virt_addr,
		      jr->inb_phys_addr);
err_free:
	kfree(jr);
	return ret;
}

/* Probe function for RAID Engine */
static int raide_probe(struct platform_device *ofdev)
{
	struct re_drv_private *repriv;
	struct device *dev;
	struct device_node *np;
	struct device_node *child;
	u32 off;
	u8 ridx = 0;
	struct dma_device *dma_dev;
	int ret = 0, rc;

	dev_info(&ofdev->dev, "Freescale RAID Engine driver\n");
	dev = &ofdev->dev;

	repriv = kzalloc(sizeof(struct re_drv_private), GFP_KERNEL);
	if (!repriv) {
		dev_err(dev, "No memory for repriv\n");
		return -ENOMEM;
	}

	dev_set_drvdata(dev, repriv);

	/* IOMAP the entire RAID Engine region */
	repriv->re_regs = of_iomap(ofdev->dev.of_node, 0);
	if (repriv->re_regs == NULL) {
		dev_err(dev, "of_iomap failed\n");
		kfree(repriv);
		ret = -ENOMEM;
		goto err_free_4;
	}

	/* Print the RE version */
	dev_info(dev, "Ver = %x\n", in_be32(&repriv->re_regs->re_version_id));

	/* Program the RE mode */
	out_be32(&repriv->re_regs->global_config, RE_NON_DPAA_MODE);
	dev_info(dev, "RE mode is %x\n",
		 in_be32(&repriv->re_regs->global_config));

	/* Program Galois Field polynomial */
	out_be32(&repriv->re_regs->galois_field_config, RE_GFM_POLY);
	dev_info(dev, "Galois Field Polynomial is %x\n",
		 in_be32(&repriv->re_regs->galois_field_config));

	dma_dev = &repriv->dma_dev;
	dma_dev->dev = dev;
	INIT_LIST_HEAD(&dma_dev->channels);
	dma_set_mask(dev, DMA_BIT_MASK(40));

	dma_dev->device_alloc_chan_resources = re_jr_alloc_chan_resources;
	dma_dev->device_tx_status = re_jr_tx_status;
	dma_dev->device_issue_pending = re_jr_issue_pending;

	dma_dev->max_xor = MAX_XOR_SRCS;
	dma_dev->device_prep_dma_xor = re_jr_prep_dma_xor;
	dma_cap_set(DMA_XOR, dma_dev->cap_mask);

	dma_dev->max_pq = MAX_PQ_SRCS;
	dma_dev->device_prep_dma_pq = re_jr_prep_pq;
	dma_cap_set(DMA_PQ, dma_dev->cap_mask);

	dma_dev->device_prep_dma_memcpy = re_jr_prep_memcpy;
	dma_cap_set(DMA_MEMCPY, dma_dev->cap_mask);

	dma_dev->device_free_chan_resources = re_jr_free_chan_resources;

	repriv->total_jrs = 0;

	repriv->desc_pool = dma_pool_create("re_dma_desc_pool", dev,
					RE_CF_CDB_SIZE * RING_SIZE,
					RE_CF_CDB_ALIGN, 0);

	if (!repriv->desc_pool) {
		pr_err("No memory for dma desc pool\n");
		ret = -ENOMEM;
		goto err_free_3;
	}

	repriv->hw_desc_pool = dma_pool_create("re_hw_desc_pool", dev,
				sizeof(struct jr_hw_desc) * RING_SIZE,
				FRAME_DESC_ALIGNMENT, 0);
	if (!repriv->hw_desc_pool) {
		pr_err("No memory for hw desc pool\n");
		ret = -ENOMEM;
		goto err_free_2;
	}

	/* Parse Device tree to find out the total number of JQs present */
	for_each_compatible_node(np, NULL, "fsl,raideng-v1.0-job-queue") {
		rc = of_property_read_u32(np, "reg", &off);
		if (rc) {
			dev_err(dev, "Reg property not found in JQ node\n");
			return -ENODEV;
		}
		/* Find out the Job Rings present under each JQ */
		for_each_child_of_node(np, child) {
			rc = of_device_is_compatible(child,
					"fsl,raideng-v1.0-job-ring");
			if (rc) {
				re_jr_probe(ofdev, child, ridx++, off);
				repriv->total_jrs++;
			}
		}
	}

	dma_async_device_register(dma_dev);
	return 0;

err_free_2:
	dma_pool_destroy(repriv->desc_pool);
err_free_3:
	iounmap(repriv->re_regs);
err_free_4:
	kfree(repriv);

	return ret;
}

static void release_jr(struct re_jr *jr)
{
	/* Free the memory allocated from DMA pools and destroy them */
	dma_pool_free(jr->re_dev->hw_desc_pool, jr->inb_ring_virt_addr,
		      jr->inb_phys_addr);

	kfree(jr);
}

static int raide_remove(struct platform_device *ofdev)
{
	struct re_drv_private *repriv;
	struct device *dev;
	int i;

	dev = &ofdev->dev;
	repriv = dev_get_drvdata(dev);

	/* Cleanup JR related memory areas */
	for (i = 0; i < repriv->total_jrs; i++)
		release_jr(repriv->re_jrs[i]);

	dma_pool_destroy(repriv->hw_desc_pool);
	dma_pool_destroy(repriv->desc_pool);

	/* Unregister the driver */
	dma_async_device_unregister(&repriv->dma_dev);

	/* Unmap the RAID Engine region */
	iounmap(repriv->re_regs);

	kfree(repriv);

	return 0;
}

static struct of_device_id raide_ids[] = {
	{ .compatible = "fsl,raideng-v1.0", },
	{}
};

static struct platform_driver raide_driver = {
	.driver = {
		.name = "fsl-raideng",
		.owner = THIS_MODULE,
		.of_match_table = raide_ids,
	},
	.probe = raide_probe,
	.remove = raide_remove,
};

module_platform_driver(raide_driver);

MODULE_AUTHOR("Harninder Rai <harninder.rai@freescale.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Freescale RAID Engine Device Driver");
