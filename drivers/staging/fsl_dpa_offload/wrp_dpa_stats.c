
/* Copyright 2008-2013 Freescale Semiconductor, Inc.
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

/*
 * DPA Stats Wrapper implementation.
 */
#include "wrp_dpa_stats.h"
#include "dpa_stats_ioctl.h"

/* Other includes */
#include <linux/crc8.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/fdtable.h>
#include <linux/atomic.h>
#include <linux/export.h>

#include "lnxwrp_fm.h"
#include "fm_port_ioctls.h"
#ifdef CONFIG_COMPAT
#include "lnxwrp_ioctls_fm_compat.h"
#endif /* CONFIG_COMPAT */

#define CRC8_WCDMA_POLY						0x9b

static const struct file_operations dpa_stats_fops = {
	.owner = THIS_MODULE,
	.open = wrp_dpa_stats_open,
	.read = wrp_dpa_stats_read,
	.write = NULL,
	.unlocked_ioctl = wrp_dpa_stats_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl		= wrp_dpa_stats_compat_ioctl,
#endif /* CONFIG_COMPAT */
	.release = wrp_dpa_stats_release
};

DECLARE_CRC8_TABLE(crc8_table);

static int dpa_stats_cdev_major = -1;

struct wrp_dpa_stats_cb wrp_dpa_stats;

static void wrp_dpa_stats_event_queue_init(
		struct dpa_stats_event_queue *ev_queue);

static void wrp_dpa_stats_event_queue_free(
		struct dpa_stats_event_queue *ev_queue);

static int wrp_dpa_stats_queue_event(
		struct dpa_stats_event_queue *ev_queue,
		struct dpa_stats_event *event);

static struct dpa_stats_event *wrp_dpa_stats_dequeue_event(
		struct dpa_stats_event_queue *ev_queue,
		unsigned int block);

static long wrp_dpa_stats_do_ioctl(struct file *filp,
				   unsigned int cmd, unsigned long args);

static int copy_key_descriptor(struct dpa_offload_lookup_key *src,
			       struct dpa_offload_lookup_key *dst);

static int copy_class_members(void *objs, unsigned int size, void *dst);

static long store_get_cnts_async_params(
		struct ioc_dpa_stats_cnt_request_params *kprm);

#ifdef CONFIG_COMPAT
static long wrp_dpa_stats_do_compat_ioctl(struct file *filp,
					  unsigned int cmd,
					  unsigned long args);

static int copy_key_descriptor_compatcpy(
		struct dpa_offload_lookup_key *kprm,
		const struct compat_ioc_dpa_offld_lookup_key *uprm);

static void dpa_stats_init_compatcpy(
		struct ioc_dpa_stats_params *kprm,
		struct compat_ioc_dpa_stats_params *uprm);

static void dpa_stats_reass_cnt_compatcpy(
		struct dpa_stats_cnt_reass *kprm,
		struct dpa_stats_compat_cnt_reass *uprm);

static void dpa_stats_frag_cnt_compatcpy(
		struct dpa_stats_cnt_frag *kprm,
		struct dpa_stats_compat_cnt_frag *uprm);

static void dpa_stats_plcr_cnt_compatcpy(
		struct dpa_stats_cnt_plcr *kprm,
		struct dpa_stats_compat_cnt_plcr *uprm);

static long dpa_stats_tbl_cnt_compatcpy(
		struct dpa_stats_cnt_classif_tbl *kprm,
		struct dpa_stats_compat_cnt_classif_tbl *uprm);

static long dpa_stats_ccnode_cnt_compatcpy(
		struct dpa_stats_cnt_classif_node *kprm,
		struct dpa_stats_compat_cnt_classif_node *uprm);

static long dpa_stats_eth_cls_compatcpy(
		struct dpa_stats_cls_cnt_eth *kprm,
		struct dpa_stats_compat_cls_cnt_eth *uprm,
		uint32_t cls_members);

static long dpa_stats_reass_cls_compatcpy(
		struct dpa_stats_cls_cnt_reass *kprm,
		struct dpa_stats_compat_cnt_reass *uprm,
		uint32_t cls_members);

static long dpa_stats_frag_cls_compatcpy(
		struct dpa_stats_cls_cnt_frag *kprm,
		struct dpa_stats_compat_cnt_frag *uprm,
		uint32_t cls_members);

static long dpa_stats_plcr_cls_compatcpy(
		struct dpa_stats_cls_cnt_plcr *kprm,
		struct dpa_stats_compat_cnt_plcr *uprm,
		uint32_t cls_members);

static long dpa_stats_tbl_cls_compatcpy(
		struct dpa_stats_cls_cnt_classif_tbl *kprm,
		struct dpa_stats_compat_cls_cnt_classif_tbl *uprm,
		uint32_t cls_members);

static long dpa_stats_ccnode_cls_compatcpy(
		struct dpa_stats_cls_cnt_classif_node *kprm,
		struct dpa_stats_compat_cls_cnt_classif_node *uprm,
		uint32_t cls_members);

static long dpa_stats_ipsec_cls_compatcpy(
		struct dpa_stats_cls_cnt_ipsec *kprm,
		struct dpa_stats_compat_cls_cnt_ipsec *uprm,
		uint32_t cls_members);
#endif

int wrp_dpa_stats_init(void)
{
	/* Cannot initialize the wrapper twice */
	if (dpa_stats_cdev_major >= 0)
		return -EBUSY;

	dpa_stats_cdev_major =
	    register_chrdev(0, DPA_STATS_CDEV, &dpa_stats_fops);
	if (dpa_stats_cdev_major < 0) {
		pr_err("Could not register DPA Stats character device\n");
		return dpa_stats_cdev_major;
	}

	/* Initialize the event queue */
	wrp_dpa_stats_event_queue_init(&wrp_dpa_stats.ev_queue);

	return 0;
}

int wrp_dpa_stats_exit(void)
{
	if (dpa_stats_cdev_major < 0)
		return 0;
	unregister_chrdev(dpa_stats_cdev_major, DPA_STATS_CDEV);
	dpa_stats_cdev_major = -1;

	/* Destroy the event queue */
	wrp_dpa_stats_event_queue_free(&wrp_dpa_stats.ev_queue);

	return 0;
}

int wrp_dpa_stats_open(struct inode *inode, struct file *filp)
{
	return 0;
}


int wrp_dpa_stats_release(struct inode *inode, struct file *filp)
{
	return 0;
}

#ifdef CONFIG_COMPAT
long wrp_dpa_stats_compat_ioctl(struct file *filp, unsigned int	cmd,
				unsigned long args)
{
	return wrp_dpa_stats_do_compat_ioctl(filp, cmd, args);
}
#endif /* CONFIG_COMPAT */

long wrp_dpa_stats_ioctl(struct file *filp, unsigned int cmd,
			    unsigned long args)
{
	return wrp_dpa_stats_do_ioctl(filp, cmd, args);
}

#ifdef CONFIG_COMPAT
ssize_t wrp_dpa_stats_read(struct file *file,
			   char *buf, size_t count, loff_t *off)
{
	struct dpa_stats_event  *event;
	struct compat_dpa_stats_event_params ev_prm;
	size_t c = 0;

	/*
	 * Make sure that the size of the buffer requested by the user is
	 * at least the size of an event
	 */
	if (count < sizeof(struct compat_dpa_stats_event_params))
		return -EINVAL;

	/* Dequeue first event by using a blocking call */
	event = wrp_dpa_stats_dequeue_event(&wrp_dpa_stats.ev_queue, 0);

	while (event) {
		memset(&ev_prm, 0,
		       sizeof(struct compat_dpa_stats_event_params));

		if (event->params.bytes_written > 0 && wrp_dpa_stats.k_mem) {
			if (copy_to_user(wrp_dpa_stats.us_mem +
					 event->params.storage_area_offset,
					 wrp_dpa_stats.k_mem +
					 event->params.storage_area_offset,
					 event->params.bytes_written)) {
				pr_err("Couldn't copy counters values to storage area\n");
				return -EFAULT;
			}
		}

		ev_prm.bytes_written = event->params.bytes_written;
		ev_prm.cnts_written = event->params.cnts_written;
		ev_prm.dpa_stats_id = event->params.dpa_stats_id;
		ev_prm.storage_area_offset = event->params.storage_area_offset;
		ev_prm.request_done = ptr_to_compat(event->params.request_done);

		if (copy_to_user(buf + c, &ev_prm, sizeof(ev_prm)) != 0) {
			kfree(event);
			return -EFAULT;
		}

		kfree(event);

		count   -= sizeof(struct compat_dpa_stats_event_params);
		c       += sizeof(struct compat_dpa_stats_event_params);

		if (count < sizeof(struct compat_dpa_stats_event_params))
			break;

		/* For subsequent events, don't block */
		event = wrp_dpa_stats_dequeue_event(
				&wrp_dpa_stats.ev_queue, O_NONBLOCK);
	}

	return c;
}
#else
ssize_t wrp_dpa_stats_read(struct file *file,
			   char *buf, size_t count, loff_t *off)
{
	struct dpa_stats_event  *event;
	size_t c = 0;

	/*
	 * Make sure that the size of the buffer requested by the user is
	 * at least the size of an event
	 */
	if (count < sizeof(struct dpa_stats_event_params))
		return -EINVAL;

	/* Dequeue first event by using a blocking call */
	event = wrp_dpa_stats_dequeue_event(&wrp_dpa_stats.ev_queue, 0);
	while (event) {
		if (event->params.bytes_written > 0 && wrp_dpa_stats.k_mem) {
			if (copy_to_user(wrp_dpa_stats.us_mem +
					 event->params.storage_area_offset,
					 wrp_dpa_stats.k_mem +
					 event->params.storage_area_offset,
					 event->params.bytes_written)) {
				pr_err("Couldn't copy counters values to storage area\n");
				return -EFAULT;
			}
		}

		if (copy_to_user(buf + c,
				 &event->params,
				 sizeof(struct dpa_stats_event_params)) != 0) {
			kfree(event);
			return -EFAULT;
		}

		kfree(event);

		count   -= sizeof(struct dpa_stats_event_params);
		c       += sizeof(struct dpa_stats_event_params);

		if (count < sizeof(struct dpa_stats_event_params))
			break;

		/* For subsequent events, don't block */
		event = wrp_dpa_stats_dequeue_event(
				&wrp_dpa_stats.ev_queue, O_NONBLOCK);
	}

	return c;
}
#endif

static void wrp_dpa_stats_event_queue_init(
		struct dpa_stats_event_queue *event_queue)
{
	INIT_LIST_HEAD(&event_queue->lh);
	mutex_init(&wrp_dpa_stats.event_queue_lock);
	atomic_set(&event_queue->count, 0);
	init_waitqueue_head(&event_queue->wq);
}

static void wrp_dpa_stats_event_queue_free(
		struct dpa_stats_event_queue *event_queue)
{
	struct dpa_stats_event   *entry, *tmp;

	/* Remove remaining events from the event queue */
	mutex_lock(&wrp_dpa_stats.event_queue_lock);
	list_for_each_entry_safe(entry, tmp, &event_queue->lh, lh) {
		list_del(&entry->lh);
		atomic_dec(&event_queue->count);
		kfree(entry);
	}
	mutex_unlock(&wrp_dpa_stats.event_queue_lock);
}

static int wrp_dpa_stats_queue_event(struct dpa_stats_event_queue *event_queue,
				     struct dpa_stats_event *event)
{
	/* If the event queue is already full, abort: */
	if (atomic_read(&event_queue->count) >= QUEUE_MAX_EVENTS) {
		pr_err("Event queue is full!\n");
		return -EBUSY;
	}

	/* Add the event to the event queue */
	mutex_lock(&wrp_dpa_stats.event_queue_lock);
	list_add_tail(&event->lh, &event_queue->lh);
	atomic_inc(&event_queue->count);
	mutex_unlock(&wrp_dpa_stats.event_queue_lock);

	/* Wake up consumers */
	wake_up_interruptible(&event_queue->wq);
	return 0;
}

static struct dpa_stats_event *wrp_dpa_stats_dequeue_event(
		struct dpa_stats_event_queue *event_queue, unsigned int block)
{
	struct dpa_stats_event	*event;

	/*
	 * If the event queue is empty we perform an interruptible sleep
	 * until an event is inserted into the queue. We use the event queue
	 * spinlock to protect ourselves from race conditions.
	 */
	mutex_lock(&wrp_dpa_stats.event_queue_lock);

	while (list_empty(&event_queue->lh)) {
		mutex_unlock(&wrp_dpa_stats.event_queue_lock);

		/* If a non blocking action was requested, return failure: */
		if (block & O_NONBLOCK)
			return NULL;

		if (wait_event_interruptible(event_queue->wq,
			!list_empty(&event_queue->lh)))
			/* Woken up by some signal... */
			return NULL;

		mutex_lock(&wrp_dpa_stats.event_queue_lock);
	}

	/* Consume one event */
	event = list_entry((&event_queue->lh)->next,
			struct dpa_stats_event, lh);
	list_del(&event->lh);
	atomic_dec(&event_queue->count);
	mutex_unlock(&wrp_dpa_stats.event_queue_lock);

	return event;
}

void do_ioctl_req_done_cb(int dpa_stats_id,
			  unsigned int storage_area_offset,
			  unsigned int cnts_written, int bytes_written)
{
	struct dpa_stats_event *event = NULL;
	struct dpa_stats_async_req_ev *async_req_ev;
	struct list_head *async_req_grp, *pos;
	uint8_t grp_idx = 0;
	bool found = false;

	/* Obtain the group the request belongs to */
	grp_idx = crc8(crc8_table,
			(uint8_t *)&storage_area_offset,
			sizeof(unsigned int),
			0);
	async_req_grp = &wrp_dpa_stats.async_req_group[grp_idx];
	mutex_lock(&wrp_dpa_stats.async_req_lock);
	BUG_ON(list_empty(async_req_grp));

	/* Search in the request group the request event */
	list_for_each(pos, async_req_grp) {
		async_req_ev = list_entry(pos,
					  struct dpa_stats_async_req_ev, node);

		if (async_req_ev->storage_area_offset == storage_area_offset) {
			list_del(&async_req_ev->node);
			list_add_tail(&async_req_ev->node,
				      &wrp_dpa_stats.async_req_pool);
			found = true;
			break;
		}
	}

	if (!found) {
		pr_err("Event was not found in the event list!\n");
		mutex_unlock(&wrp_dpa_stats.async_req_lock);
		return;
	}

	/* Generate new event description: */
	event = kmalloc(sizeof(struct dpa_stats_event), GFP_KERNEL);
	if (!event) {
		pr_err("No more memory for events !\n");
		mutex_unlock(&wrp_dpa_stats.async_req_lock);
		return;
	}

	/* Fill up the event parameters data structure */
	event->params.dpa_stats_id = dpa_stats_id;
	event->params.storage_area_offset = storage_area_offset;
	event->params.cnts_written = cnts_written;
	event->params.bytes_written = bytes_written;
	event->params.request_done = async_req_ev->request_done;

	mutex_unlock(&wrp_dpa_stats.async_req_lock);

	/* Queue this event */
	if (wrp_dpa_stats_queue_event(&wrp_dpa_stats.ev_queue, event) != 0) {
		pr_err("Failed to queue event.\n");
		kfree(event);
		return;
	}

	return;
}

static long do_ioctl_stats_init(struct ioc_dpa_stats_params *prm)
{
	struct dpa_stats_async_req_ev *async_req_ev;
	struct dpa_stats_params params;
	long ret = 0;
	uint16_t i;

	/* Save user-provided parameters */
	params.max_counters = prm->max_counters;
	params.storage_area_len = prm->storage_area_len;

	if (prm->stg_area_mapped) {
		/*
		 * Storage area is mapped, obtain the kernel-space memory area
		 * pointer from the physical address
		 */
		params.storage_area = phys_to_virt(prm->phys_stg_area);
		if (!params.storage_area) {
			pr_err("Invalid physical memory address\n");
			return -EINVAL;
		}
		wrp_dpa_stats.k_mem = NULL;
	} else {
		/* Save user-space memory area pointer */
		wrp_dpa_stats.us_mem = prm->virt_stg_area;

		/* Allocate kernel-space memory to store the statistics */
		params.storage_area = kzalloc(
				prm->storage_area_len, GFP_KERNEL);
		if (!params.storage_area) {
			pr_err("Could not allocate kernel storage area\n");
			return -ENOMEM;
		}

		/* Save kernel-space memory area pointer */
		wrp_dpa_stats.k_mem = params.storage_area;
	}

	/* Call init function */
	ret = dpa_stats_init(&params, &prm->dpa_stats_id);
	if (ret < 0)
		return ret;

	/* Init CRC8 table */
	crc8_populate_msb(crc8_table, CRC8_WCDMA_POLY);

	/* Allocate asynchronous requests groups lists */
	wrp_dpa_stats.async_req_group = kmalloc(DPA_STATS_MAX_NUM_OF_REQUESTS *
				sizeof(struct list_head), GFP_KERNEL);
	if (!wrp_dpa_stats.async_req_group) {
		pr_err("Could not allocate memory for async requests group\n");
		return -ENOMEM;
	}

	/* Initialize list of free async requests nodes */
	INIT_LIST_HEAD(&wrp_dpa_stats.async_req_pool);

	for (i = 0; i < DPA_STATS_MAX_NUM_OF_REQUESTS; i++) {

		/* Initialize the list of async requests in the same group */
		INIT_LIST_HEAD(&wrp_dpa_stats.async_req_group[i]);

		/* Allocate an asynchronous request event node */
		async_req_ev = kzalloc(sizeof(*async_req_ev), GFP_KERNEL);
		if (!async_req_ev) {
			struct dpa_stats_async_req_ev *tmp;

			list_for_each_entry_safe(async_req_ev, tmp,
				&wrp_dpa_stats.async_req_pool, node) {
				list_del(&async_req_ev->node);
				kfree(async_req_ev);
			}
			pr_err("Could not allocate "
				"memory for asynchronous request event\n");
			return -ENOMEM;
		}

		list_add_tail(&async_req_ev->node,
			      &wrp_dpa_stats.async_req_pool);
	}

	mutex_init(&wrp_dpa_stats.async_req_lock);

	return ret;
}

static long do_ioctl_stats_free(void *args)
{
	struct dpa_stats_async_req_ev *async_req_ev, *tmp;
	int dpa_stats_id;
	long ret;

	if (copy_from_user(&dpa_stats_id, (int *)args, sizeof(int))) {
		pr_err("Could not copy parameters\n");
		return -EINVAL;
	}

	/* Release kernel allocated memory */
	kfree(wrp_dpa_stats.k_mem);

	mutex_lock(&wrp_dpa_stats.async_req_lock);
	list_for_each_entry_safe(async_req_ev,
				 tmp, &wrp_dpa_stats.async_req_pool, node) {
		list_del(&async_req_ev->node);
		kfree(async_req_ev);
	}
	mutex_unlock(&wrp_dpa_stats.async_req_lock);

	ret = dpa_stats_free(dpa_stats_id);
	if (ret < 0)
		return ret;

	return ret;
}

static int do_ioctl_stats_create_counter(void *args)
{
	struct ioc_dpa_stats_cnt_params prm;
	struct dpa_offload_lookup_key key;
	long ret = 0;

	if (copy_from_user(&prm, args, sizeof(prm))) {
		pr_err("Could not copy Counter parameters\n");
		return -EINVAL;
	}

	if (prm.cnt_params.type == DPA_STATS_CNT_CLASSIF_NODE)
		ret = copy_key_descriptor(
				&prm.cnt_params.classif_node_params.key, &key);
	else if (prm.cnt_params.type == DPA_STATS_CNT_CLASSIF_TBL)
		ret = copy_key_descriptor(
				&prm.cnt_params.classif_tbl_params.key, &key);
	if (ret != 0) {
		pr_err("Could not copy the key descriptor\n");
		return -EINVAL;
	}

	ret = dpa_stats_create_counter(prm.stats_id,
				       &prm.cnt_params, &prm.cnt_id);
	if (ret < 0)
		return ret;

	if (copy_to_user(args, &prm, sizeof(prm))) {
		pr_err("Could not copy to user the Counter ID\n");
		ret = -EINVAL;
	}

	if (prm.cnt_params.type == DPA_STATS_CNT_CLASSIF_NODE ||
	    prm.cnt_params.type == DPA_STATS_CNT_CLASSIF_TBL) {
		kfree(key.byte);
		kfree(key.mask);
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static int do_ioctl_stats_compat_create_counter(void *args)
{
	struct ioc_dpa_stats_cnt_params kprm;
	struct compat_ioc_dpa_stats_cnt_params uprm;
	long ret = 0;

	if (copy_from_user(&uprm, args, sizeof(uprm))) {
		pr_err("Could not copy Counter parameters\n");
		return -EINVAL;
	}

	memset(&kprm, 0, sizeof(struct ioc_dpa_stats_cnt_params));
	kprm.stats_id = uprm.stats_id;
	kprm.cnt_params.type = uprm.cnt_params.type;

	switch (kprm.cnt_params.type) {
	case DPA_STATS_CNT_ETH:
		memcpy(&kprm.cnt_params.eth_params,
		       &uprm.cnt_params.eth_params,
		       sizeof(struct dpa_stats_cnt_eth));
		break;
	case DPA_STATS_CNT_REASS:
		dpa_stats_reass_cnt_compatcpy(&kprm.cnt_params.reass_params,
					      &uprm.cnt_params.reass_params);
		break;
	case DPA_STATS_CNT_FRAG:
		dpa_stats_frag_cnt_compatcpy(&kprm.cnt_params.frag_params,
					     &uprm.cnt_params.frag_params);
		break;
	case DPA_STATS_CNT_POLICER:
		dpa_stats_plcr_cnt_compatcpy(&kprm.cnt_params.plcr_params,
					     &uprm.cnt_params.plcr_params);
		break;
	case DPA_STATS_CNT_CLASSIF_TBL:
		ret = dpa_stats_tbl_cnt_compatcpy(
					&kprm.cnt_params.classif_tbl_params,
					&uprm.cnt_params.classif_tbl_params);
		if (ret < 0)
			return ret;
		break;
	case DPA_STATS_CNT_CLASSIF_NODE:
		ret = dpa_stats_ccnode_cnt_compatcpy(
					&kprm.cnt_params.classif_node_params,
					&uprm.cnt_params.classif_node_params);
		if (ret < 0)
			return ret;
		break;
	case DPA_STATS_CNT_IPSEC:
		memcpy(&kprm.cnt_params.ipsec_params,
		       &uprm.cnt_params.ipsec_params,
		       sizeof(struct dpa_stats_cnt_ipsec));
		break;
	case DPA_STATS_CNT_TRAFFIC_MNG:
		memcpy(&kprm.cnt_params.traffic_mng_params,
		       &uprm.cnt_params.traffic_mng_params,
		       sizeof(struct dpa_stats_cnt_traffic_mng));
		break;
	default:
		break;
	}

	ret = dpa_stats_create_counter(kprm.stats_id,
				       &kprm.cnt_params, &kprm.cnt_id);
	if (ret < 0)
		return ret;

	uprm.cnt_id = kprm.cnt_id;

	if (copy_to_user(args, &uprm, sizeof(uprm))) {
		pr_err("Could not copy to user the Counter ID\n");
		ret = -EINVAL;
	}

	if (kprm.cnt_params.type == DPA_STATS_CNT_CLASSIF_NODE) {
		kfree(kprm.cnt_params.classif_node_params.key.byte);
		kfree(kprm.cnt_params.classif_node_params.key.mask);
	} else if (kprm.cnt_params.type == DPA_STATS_CNT_CLASSIF_TBL) {
		kfree(kprm.cnt_params.classif_tbl_params.key.byte);
		kfree(kprm.cnt_params.classif_tbl_params.key.mask);
	}

	return ret;
}
#endif

static int do_ioctl_stats_create_class_counter(void *args)
{
	struct ioc_dpa_stats_cls_cnt_params prm;
	struct dpa_stats_cls_cnt_classif_node *cnode;
	struct dpa_stats_cls_cnt_classif_tbl  *tbl;
	struct dpa_offload_lookup_key key;
	struct dpa_stats_cnt_eth_src *eth_src = NULL;
	uint32_t i = 0, eth_src_size = 0;
	void *cls_objs = NULL;
	int *sa_ids = NULL;
	long ret = 0;

	if (copy_from_user(&prm, args, sizeof(prm))) {
		pr_err("Could not copy Counter parameters\n");
		return -EINVAL;
	}

	switch (prm.cnt_params.type) {
	case DPA_STATS_CNT_ETH:
		eth_src_size = prm.cnt_params.class_members *
				sizeof(struct dpa_stats_cnt_eth_src);

		/* Allocate memory to store the sources array */
		eth_src = kmalloc(eth_src_size, GFP_KERNEL);
		if (!eth_src) {
			pr_err("No more memory for ethernet sources array\n");
			return -ENOMEM;
		}

		if (copy_from_user(eth_src,
				   prm.cnt_params.eth_params.src,
				   eth_src_size)) {
			pr_err("Could not copy array of ethernet sources\n");
			kfree(eth_src);
			return -EBUSY;
		}
		prm.cnt_params.eth_params.src = eth_src;
		break;
	case DPA_STATS_CNT_REASS:
		ret = copy_class_members(cls_objs,
					 prm.cnt_params.class_members,
					 prm.cnt_params.reass_params.reass);
		if (ret < 0) {
			pr_err("Could not copy array of reass objects\n");
			kfree(cls_objs);
			return -EBUSY;
		}
		break;
	case DPA_STATS_CNT_FRAG:
		ret = copy_class_members(cls_objs,
					 prm.cnt_params.class_members,
					 prm.cnt_params.frag_params.frag);
		if (ret < 0) {
			pr_err("Could not copy array of frag objects\n");
			kfree(cls_objs);
			return -EBUSY;
		}
		break;
	case DPA_STATS_CNT_POLICER:
		ret = copy_class_members(cls_objs,
					 prm.cnt_params.class_members,
					 prm.cnt_params.plcr_params.plcr);
		if (ret < 0) {
			pr_err("Could not copy array of policer objects\n");
			kfree(cls_objs);
			return -EBUSY;
		}
		break;
	case DPA_STATS_CNT_CLASSIF_TBL:
		tbl = &prm.cnt_params.classif_tbl_params;

		if (tbl->key_type == DPA_STATS_CLASSIF_SINGLE_KEY) {
			for (i = 0; i < prm.cnt_params.class_members; i++) {
				if (!tbl->keys[i].byte)
					continue;

				ret = copy_key_descriptor(&tbl->keys[i], &key);
				if (ret != 0) {
					pr_err("Couldn't copy the key descriptor\n");
					return -EINVAL;
				}
			}
		} else if (tbl->key_type == DPA_STATS_CLASSIF_PAIR_KEY) {
			for (i = 0; i < prm.cnt_params.class_members; i++) {
				if (!tbl->pairs[i].first_key.byte)
					continue;

				ret = copy_key_descriptor(
						&tbl->pairs[i].first_key, &key);
				if (ret != 0) {
					pr_err("Could not copy the key descriptor\n");
					return -EINVAL;
				}

				ret = copy_key_descriptor(
					&tbl->pairs[i].second_key, &key);
				if (ret != 0) {
					pr_err("Could not copy the key descriptor\n");
					return -EINVAL;
				}
			}
		}
		break;
	case DPA_STATS_CNT_CLASSIF_NODE:
		cnode = &prm.cnt_params.classif_node_params;

		for (i = 0; i < prm.cnt_params.class_members; i++) {
			ret = copy_key_descriptor(&cnode->keys[i], &key);
			if (ret != 0) {
				pr_err("Could not copy the key descriptor\n");
				return -EINVAL;
			}
		}
		break;
	case DPA_STATS_CNT_IPSEC:
		/* Allocate memory to store the sa ids array */
		sa_ids = kmalloc(prm.cnt_params.class_members, GFP_KERNEL);
		if (!sa_ids) {
			pr_err("No more memory for sa ids pointer\n");
			return -ENOMEM;
		}

		if (copy_from_user(sa_ids,
				prm.cnt_params.ipsec_params.sa_id,
				(prm.cnt_params.class_members * sizeof(int)))) {
			pr_err("Could not copy array of SA ids\n");
			kfree(sa_ids);
			return -EBUSY;
		}

		prm.cnt_params.ipsec_params.sa_id = sa_ids;
		break;
	default:
		break;
	}

	ret = dpa_stats_create_class_counter(prm.stats_id,
					     &prm.cnt_params, &prm.cnt_id);
	if (ret < 0)
		return ret;

	if (copy_to_user(args, &prm, sizeof(prm))) {
		pr_err("Could not copy to user the Counter ID\n");
		ret = -EINVAL;
	}

	switch (prm.cnt_params.type) {
	case DPA_STATS_CNT_ETH:
		kfree(eth_src);
		break;
	case DPA_STATS_CNT_REASS:
	case DPA_STATS_CNT_FRAG:
	case DPA_STATS_CNT_POLICER:
		kfree(cls_objs);
		break;
	case DPA_STATS_CNT_CLASSIF_TBL:
		tbl = &prm.cnt_params.classif_tbl_params;

		for (i = 0; i < prm.cnt_params.class_members; i++) {
			if (tbl->key_type == DPA_STATS_CLASSIF_SINGLE_KEY) {
				kfree(tbl->keys[i].byte);
				kfree(tbl->keys[i].mask);
			}

			if (tbl->key_type == DPA_STATS_CLASSIF_PAIR_KEY) {
				kfree(tbl->pairs[i].first_key.byte);
				kfree(tbl->pairs[i].first_key.mask);
				kfree(tbl->pairs[i].second_key.byte);
				kfree(tbl->pairs[i].second_key.mask);
			}
		}
		break;
	case DPA_STATS_CNT_CLASSIF_NODE:
		for (i = 0; i < prm.cnt_params.class_members; i++) {
			kfree(prm.cnt_params.classif_node_params.keys[i].byte);
			kfree(prm.cnt_params.classif_node_params.keys[i].mask);
		}
		break;
	case DPA_STATS_CNT_IPSEC:
		kfree(sa_ids);
		break;

	default:
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static int do_ioctl_stats_compat_create_class_counter(void *args)
{
	struct ioc_dpa_stats_cls_cnt_params kprm;
	struct compat_ioc_dpa_stats_cls_cnt_params uprm;
	struct dpa_stats_cls_cnt_params *kprm_cls = &kprm.cnt_params;
	struct dpa_stats_compat_cls_cnt_params *uprm_cls = &uprm.cnt_params;
	long ret = 0;
	uint32_t i = 0;

	if (copy_from_user(&uprm, args, sizeof(uprm))) {
		pr_err("Could not copy Counter parameters\n");
		return -EINVAL;
	}

	memset(&kprm, 0, sizeof(struct ioc_dpa_stats_cls_cnt_params));
	kprm.stats_id = uprm.stats_id;
	kprm_cls->type = uprm_cls->type;
	kprm_cls->class_members = uprm_cls->class_members;

	switch (kprm.cnt_params.type) {
	case DPA_STATS_CNT_ETH:
		ret = dpa_stats_eth_cls_compatcpy(&kprm_cls->eth_params,
			&uprm_cls->eth_params, kprm_cls->class_members);
		if (ret < 0)
			return ret;
		break;
	case DPA_STATS_CNT_REASS:
		ret = dpa_stats_reass_cls_compatcpy(&kprm_cls->reass_params,
			&uprm_cls->reass_params, kprm_cls->class_members);
		if (ret < 0)
			return ret;
		break;
	case DPA_STATS_CNT_FRAG:
		ret = dpa_stats_frag_cls_compatcpy(&kprm_cls->frag_params,
			&uprm_cls->frag_params, kprm_cls->class_members);
		if (ret < 0)
			return ret;
		break;
	case DPA_STATS_CNT_POLICER:
		ret = dpa_stats_plcr_cls_compatcpy(&kprm_cls->plcr_params,
			&uprm_cls->plcr_params, kprm_cls->class_members);
		if (ret < 0)
			return ret;
		break;
	case DPA_STATS_CNT_CLASSIF_TBL:
		ret = dpa_stats_tbl_cls_compatcpy(&kprm_cls->classif_tbl_params,
			&uprm_cls->classif_tbl_params, kprm_cls->class_members);
		if (!ret)
			break;

		for (i = 0; i < kprm_cls->class_members; i++) {
			struct dpa_stats_cls_cnt_classif_tbl *tbl =
						&kprm_cls->classif_tbl_params;

			if (tbl->key_type == DPA_STATS_CLASSIF_SINGLE_KEY) {
				kfree(tbl->keys[i].byte);
				kfree(tbl->keys[i].mask);
			}

			if (tbl->key_type == DPA_STATS_CLASSIF_PAIR_KEY) {
				kfree(tbl->pairs[i].first_key.byte);
				kfree(tbl->pairs[i].first_key.mask);
				kfree(tbl->pairs[i].second_key.byte);
				kfree(tbl->pairs[i].second_key.mask);
			}
		}
		return ret;
	case DPA_STATS_CNT_CLASSIF_NODE:
		ret = dpa_stats_ccnode_cls_compatcpy(
					&kprm_cls->classif_node_params,
					&uprm_cls->ccnode_params,
					kprm_cls->class_members);
		if (!ret)
			break;
		for (i = 0; i < kprm_cls->class_members; i++) {
			kfree(kprm_cls->classif_node_params.keys[i].byte);
			kfree(kprm_cls->classif_node_params.keys[i].mask);
		}
		return ret;
	case DPA_STATS_CNT_IPSEC:
		ret = dpa_stats_ipsec_cls_compatcpy(&kprm_cls->ipsec_params,
			&uprm_cls->ipsec_params, kprm_cls->class_members);
		if (ret < 0)
			return ret;
		break;
	default:
		break;
	}

	ret = dpa_stats_create_class_counter(
			kprm.stats_id, kprm_cls, &kprm.cnt_id);
	if (ret < 0)
		return ret;

	uprm.cnt_id = kprm.cnt_id;

	if (copy_to_user(args, &uprm, sizeof(uprm))) {
		pr_err("Could not copy to user the Counter ID\n");
		ret = -EINVAL;
	}

	switch (uprm.cnt_params.type) {
	case DPA_STATS_CNT_ETH:
		kfree(kprm_cls->eth_params.src);
		break;
	case DPA_STATS_CNT_REASS:
		kfree(kprm_cls->reass_params.reass);
		break;
	case DPA_STATS_CNT_FRAG:
		kfree(kprm_cls->frag_params.frag);
		break;
	case DPA_STATS_CNT_POLICER:
		kfree(kprm_cls->plcr_params.plcr);
		break;
	case DPA_STATS_CNT_CLASSIF_TBL:
	{
		struct dpa_stats_cls_cnt_classif_tbl *tbl =
				&kprm_cls->classif_tbl_params;

		for (i = 0; i < kprm_cls->class_members; i++) {
			if (tbl->key_type == DPA_STATS_CLASSIF_SINGLE_KEY) {
				kfree(tbl->keys[i].byte);
				kfree(tbl->keys[i].mask);
			}

			if (tbl->key_type == DPA_STATS_CLASSIF_PAIR_KEY) {
				kfree(tbl->pairs[i].first_key.byte);
				kfree(tbl->pairs[i].first_key.mask);
				kfree(tbl->pairs[i].second_key.byte);
				kfree(tbl->pairs[i].second_key.mask);
			}
		}
		break;
	}
	case DPA_STATS_CNT_CLASSIF_NODE:
		for (i = 0; i < kprm_cls->class_members; i++) {
			kfree(kprm_cls->classif_node_params.keys[i].byte);
			kfree(kprm_cls->classif_node_params.keys[i].mask);
		}
		break;
	case DPA_STATS_CNT_IPSEC:
		kfree(kprm_cls->ipsec_params.sa_id);
		break;

	default:
		break;
	}

	return ret;
}
#endif

static int do_ioctl_stats_modify_class_counter(void *args)
{
	struct ioc_dpa_stats_cls_member_params prm;
	struct dpa_offload_lookup_key key;
	int ret;

	if (copy_from_user(&prm, args, sizeof(prm))) {
		pr_err("Could not copy user parameters\n");
		return -EINVAL;
	}

	switch (prm.params.type) {
	case DPA_STATS_CLS_MEMBER_SINGLE_KEY:
		if (prm.params.key.byte) {
			ret = copy_key_descriptor(&prm.params.key, &key);
			if (ret != 0) {
				pr_err("Couldn't copy the key descriptor\n");
				return -EINVAL;
			}
		}
		break;
	case DPA_STATS_CLS_MEMBER_PAIR_KEY:
		if (prm.params.pair.first_key.byte &&
				prm.params.pair.first_key.mask) {
			ret = copy_key_descriptor(
					&prm.params.pair.first_key, &key);
			if (ret != 0) {
				pr_err("Could not copy the key descriptor\n");
				return -EINVAL;
			}

			ret = copy_key_descriptor(
					&prm.params.pair.second_key, &key);
			if (ret != 0) {
				pr_err("Could not copy the key descriptor\n");
				return -EINVAL;
			}
		}
		break;
	case DPA_STATS_CLS_MEMBER_SA_ID:
		break;
	default:
		pr_err("Invalid class member type\n");
		return -EINVAL;
	}

	ret = dpa_stats_modify_class_counter(prm.cnt_id,
					     &prm.params, prm.member_index);
	if (ret < 0)
		return ret;

	switch (prm.params.type) {
	case DPA_STATS_CLS_MEMBER_SINGLE_KEY:
		kfree(prm.params.key.byte);
		kfree(prm.params.key.mask);
		break;
	case DPA_STATS_CLS_MEMBER_PAIR_KEY:
		kfree(prm.params.pair.first_key.byte);
		kfree(prm.params.pair.first_key.mask);
		kfree(prm.params.pair.second_key.byte);
		kfree(prm.params.pair.second_key.mask);
		break;
	case DPA_STATS_CLS_MEMBER_SA_ID:
		break;
	default:
		pr_err("Invalid class member type\n");
		break;
	}

	if (copy_to_user(args, &prm, sizeof(prm))) {
		pr_err("Could not write "
		       "dpa_stats_modify_class_counter result\n");
		return -EBUSY;
	}

	return 0;
}

#ifdef CONFIG_COMPAT
static int do_ioctl_stats_compat_modify_class_counter(void *args)
{
	struct ioc_dpa_stats_cls_member_params kprm;
	struct compat_ioc_dpa_stats_cls_member_params uprm;
	int ret;

	if (copy_from_user(&uprm, args, sizeof(uprm))) {
		pr_err("Could not copy user parameters\n");
		return -EINVAL;
	}

	memset(&kprm, 0, sizeof(struct ioc_dpa_stats_cls_member_params));
	kprm.cnt_id = uprm.cnt_id;
	kprm.member_index = uprm.member_index;
	kprm.params.type = uprm.params.type;

	switch (kprm.params.type) {
	case DPA_STATS_CLS_MEMBER_SINGLE_KEY:
		if (compat_ptr(uprm.params.key.byte)) {
			ret = copy_key_descriptor_compatcpy(
					&kprm.params.key,
					&uprm.params.key);
			if (ret < 0)
				return ret;
		}
		break;
	case DPA_STATS_CLS_MEMBER_PAIR_KEY:
		if (compat_ptr(uprm.params.pair.first_key.byte)) {
			ret = copy_key_descriptor_compatcpy(
					&kprm.params.pair.first_key,
					&uprm.params.pair.first_key);
			if (ret < 0)
				return ret;

			ret = copy_key_descriptor_compatcpy(
					&kprm.params.pair.second_key,
					&uprm.params.pair.second_key);
			if (ret != 0) {
				pr_err("Could not copy the key descriptor\n");
				return -EINVAL;
			}
		}
		break;
	case DPA_STATS_CLS_MEMBER_SA_ID:
		kprm.params.sa_id = uprm.params.sa_id;
		break;
	default:
		pr_err("Invalid class member type\n");
		return -EINVAL;
	}

	ret = dpa_stats_modify_class_counter(kprm.cnt_id,
			&kprm.params, kprm.member_index);
	if (ret < 0)
		return ret;

	uprm.cnt_id = kprm.cnt_id;

	switch (kprm.params.type) {
	case DPA_STATS_CLS_MEMBER_SINGLE_KEY:
		kfree(kprm.params.key.byte);
		kfree(kprm.params.key.mask);
		break;
	case DPA_STATS_CLS_MEMBER_PAIR_KEY:
		kfree(kprm.params.pair.first_key.byte);
		kfree(kprm.params.pair.first_key.mask);
		kfree(kprm.params.pair.second_key.byte);
		kfree(kprm.params.pair.second_key.mask);
		break;
	case DPA_STATS_CLS_MEMBER_SA_ID:
		break;
	default:
		pr_err("Invalid class member type\n");
		break;
	}

	if (copy_to_user(args, &uprm, sizeof(uprm))) {
		pr_err("Could not write "
				"dpa_stats_modify_class_counter result\n");
		return -EBUSY;
	}

	return 0;
}
#endif

static int do_ioctl_stats_get_counters(void *args)
{
	struct ioc_dpa_stats_cnt_request_params prm;
	int *cnts_ids;
	long ret = 0;

	if (copy_from_user(&prm, args, sizeof(prm))) {
		pr_err("Could not copy Request parameters\n");
		return -EINVAL;
	}

	/* Allocate kernel-space memory area to copy the counters ids */
	cnts_ids = kzalloc(prm.req_params.cnts_ids_len *
			   sizeof(int), GFP_KERNEL);
	if (!cnts_ids) {
		pr_err("Could not allocate requested counters array\n");
		return -ENOMEM;
	}

	/* Copy the user provided counter ids */
	if (copy_from_user(cnts_ids,
			   prm.req_params.cnts_ids,
			   (prm.req_params.cnts_ids_len * sizeof(int)))) {
		pr_err("Could not copy requested counters ids\n");
		kfree(prm.req_params.cnts_ids);
		return -EINVAL;
	}

	prm.req_params.cnts_ids = cnts_ids;

	/* If counters request is asynchronous */
	if (prm.request_done) {
		ret = store_get_cnts_async_params(&prm);
		if (ret < 0)
			return ret;
	}

	ret = dpa_stats_get_counters(prm.req_params,
				    &prm.cnts_len, prm.request_done);
	if (ret < 0) {
		kfree(prm.req_params.cnts_ids);
		return ret;
	}

	/* Request was sent, release the array of counter ids */
	kfree(prm.req_params.cnts_ids);

	/* If request is synchronous copy counters length to user space */
	if (!prm.request_done) {
		if (wrp_dpa_stats.k_mem)
			if (copy_to_user((wrp_dpa_stats.us_mem +
					  prm.req_params.storage_area_offset),
					  (wrp_dpa_stats.k_mem +
					  prm.req_params.storage_area_offset),
					  prm.cnts_len)) {
				pr_err("Couldn't copy counters values to storage area\n");
				return -EINVAL;
			}

		if (copy_to_user(args, &prm, sizeof(prm))) {
			pr_err("Could not copy to user the counters length\n");
			ret = -EINVAL;
		}
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static int do_ioctl_stats_compat_get_counters(void *args)
{
	struct ioc_dpa_stats_cnt_request_params kprm;
	struct compat_ioc_dpa_stats_cnt_request_params uprm;
	long ret = 0;

	if (copy_from_user(&uprm, args, sizeof(uprm))) {
		pr_err("Could not copy Request parameters\n");
		return -EINVAL;
	}

	memset(&kprm, 0, sizeof(struct ioc_dpa_stats_cnt_request_params));
	kprm.cnts_len = uprm.cnts_len;
	kprm.request_done = (dpa_stats_request_cb)
			((compat_ptr)(uprm.request_done));
	kprm.req_params.cnts_ids_len = uprm.req_params.cnts_ids_len;
	kprm.req_params.reset_cnts = uprm.req_params.reset_cnts;
	kprm.req_params.storage_area_offset =
			uprm.req_params.storage_area_offset;

	/* Allocate kernel-space memory area to copy the counters ids */
	kprm.req_params.cnts_ids = kzalloc(kprm.req_params.cnts_ids_len *
					   sizeof(int), GFP_KERNEL);
	if (!kprm.req_params.cnts_ids) {
		pr_err("Could not allocate requested counters array\n");
		return -ENOMEM;
	}

	/* Copy the user provided counter ids */
	if (copy_from_user(kprm.req_params.cnts_ids,
			   (compat_ptr)(uprm.req_params.cnts_ids),
			   (kprm.req_params.cnts_ids_len * sizeof(int)))) {
		pr_err("Could not copy requested counters ids\n");
		kfree(kprm.req_params.cnts_ids);
		return -EINVAL;
	}

	/* If counters request is asynchronous */
	if (kprm.request_done) {
		ret = store_get_cnts_async_params(&kprm);
		if (ret < 0)
			return ret;
	}

	ret = dpa_stats_get_counters(kprm.req_params,
				     &kprm.cnts_len, kprm.request_done);
	if (ret < 0) {
		kfree(kprm.req_params.cnts_ids);
		return ret;
	}

	/* Request was sent, release the array of counter ids */
	kfree(kprm.req_params.cnts_ids);

	/* If request is synchronous copy counters length to user space */
	if (!kprm.request_done) {
		if (wrp_dpa_stats.k_mem)
			if (copy_to_user((wrp_dpa_stats.us_mem +
					kprm.req_params.storage_area_offset),
					(wrp_dpa_stats.k_mem +
					kprm.req_params.storage_area_offset),
					kprm.cnts_len)) {
				pr_err("Couldn't copy counters values to storage area\n");
				return -EINVAL;
			}

		uprm.cnts_len = kprm.cnts_len;

		if (copy_to_user(args, &uprm, sizeof(uprm))) {
			pr_err("Could not copy to user the counters length\n");
			ret = -EINVAL;
		}
	}

	return ret;
}
#endif

static int do_ioctl_stats_reset_counters(void *args)
{
	struct ioc_dpa_stats_cnts_reset_params prm;
	int *cnt_ids;
	long ret = 0;

	if (copy_from_user(&prm, args, sizeof(prm))) {
		pr_err("Could not copy Counters Reset parameters\n");
		return -EINVAL;
	}

	/* Allocate kernel-space memory area to copy the counters ids */
	cnt_ids = kzalloc(prm.cnts_ids_len * sizeof(int), GFP_KERNEL);
	if (!cnt_ids) {
		pr_err("Could not allocate counters ids array\n");
		return -ENOMEM;
	}

	/* Copy the user provided counter ids */
	if (copy_from_user(cnt_ids,
			prm.cnts_ids,
			(prm.cnts_ids_len * sizeof(int)))) {
		pr_err("Could not copy requested counters ids\n");
		kfree(cnt_ids);
		return -EINVAL;
	}
	prm.cnts_ids = cnt_ids;

	ret = dpa_stats_reset_counters(prm.cnts_ids, prm.cnts_ids_len);
	if (ret < 0) {
		kfree(prm.cnts_ids);
		return ret;
	}

	kfree(cnt_ids);

	if (copy_to_user(args, &prm, sizeof(prm))) {
		pr_err("Could not copy the result to user space\n");
		return -EINVAL;
	}

	return 0;
}

#ifdef CONFIG_COMPAT
static int do_ioctl_stats_compat_reset_counters(void *args)
{
	struct ioc_dpa_stats_cnts_reset_params kprm;
	struct compat_ioc_dpa_stats_cnts_reset_params uprm;
	long ret = 0;

	if (copy_from_user(&uprm, args, sizeof(uprm))) {
		pr_err("Could not copy Counters Reset parameters\n");
		return -EINVAL;
	}

	memset(&kprm, 0, sizeof(struct ioc_dpa_stats_cnts_reset_params));
	kprm.cnts_ids_len = uprm.cnts_ids_len;

	/* Allocate kernel-space memory area to copy the counters ids */
	kprm.cnts_ids = kzalloc(kprm.cnts_ids_len * sizeof(int), GFP_KERNEL);
	if (!kprm.cnts_ids) {
		pr_err("Could not allocate counters ids array\n");
		return -ENOMEM;
	}

	/* Copy the user provided counter ids */
	if (copy_from_user(kprm.cnts_ids,
			(compat_ptr)(uprm.cnts_ids),
			(kprm.cnts_ids_len * sizeof(int)))) {
		pr_err("Could not copy requested counters ids\n");
		kfree(kprm.cnts_ids);
		return -EINVAL;
	}

	ret = dpa_stats_reset_counters(kprm.cnts_ids, kprm.cnts_ids_len);
	if (ret < 0) {
		kfree(kprm.cnts_ids);
		return ret;
	}

	kfree(kprm.cnts_ids);

	if (copy_to_user(args, &uprm, sizeof(uprm))) {
		pr_err("Could not copy the result to user space\n");
		return -EINVAL;
	}

	return 0;
}
#endif

static long wrp_dpa_stats_do_ioctl(struct file *filp,
				   unsigned int cmd, unsigned long args)
{
	long ret = 0;

	switch (cmd) {
	case DPA_STATS_IOC_INIT:
	{
		struct ioc_dpa_stats_params kparam;

		/* Copy parameters from user-space */
		if (copy_from_user(&kparam, (void *)args, sizeof(kparam))) {
			pr_err("Could not read dpa_stats_init user space args\n");
			return -EBUSY;
		}

		ret = do_ioctl_stats_init(&kparam);
		if (ret < 0)
			return ret;

		/* Copy paramters to user-space */
		if (copy_to_user((void *)args, &kparam, sizeof(kparam))) {
			pr_err("Could not write dpa_stats_init result\n");
			return -EBUSY;
		}
		break;
	}
	case DPA_STATS_IOC_FREE:
		ret = do_ioctl_stats_free((void *)args);
		if (ret < 0)
			return ret;
		break;
	case DPA_STATS_IOC_CREATE_COUNTER:
		ret = do_ioctl_stats_create_counter((void *)args);
		if (ret < 0)
			return ret;
		break;
	case DPA_STATS_IOC_CREATE_CLASS_COUNTER:
		ret = do_ioctl_stats_create_class_counter((void *)args);
		if (ret < 0)
			return ret;
		break;
	case DPA_STATS_IOC_MODIFY_CLASS_COUNTER:
		ret = do_ioctl_stats_modify_class_counter((void *)args);
		if (ret < 0)
			return ret;
		break;
	case DPA_STATS_IOC_REMOVE_COUNTER:{
		int dpa_stats_cnt_id;
		if (copy_from_user(&dpa_stats_cnt_id, (int *)args,
				    sizeof(int))) {
			pr_err("Could not copy parameters\n");
			return -EINVAL;
		}

		ret = dpa_stats_remove_counter(dpa_stats_cnt_id);
		if (ret < 0)
			return ret;

		break;
	}
	case DPA_STATS_IOC_GET_COUNTERS:
		ret = do_ioctl_stats_get_counters((void *)args);
		if (ret < 0)
			return ret;
		break;
	case DPA_STATS_IOC_RESET_COUNTERS:
		ret = do_ioctl_stats_reset_counters((void *)args);
		if (ret < 0)
			return ret;
		break;
	default:
		pr_err("invalid ioctl: cmd:0x%08x(type:0x%02x, nr:0x%02x.\n",
				cmd, _IOC_TYPE(cmd), _IOC_NR(cmd));
		break;
	}
	return ret;
}

#ifdef CONFIG_COMPAT
static long wrp_dpa_stats_do_compat_ioctl(struct file *filp,
					  unsigned int cmd,
					  unsigned long args)
{
	long ret = 0;

	switch (cmd) {
	case DPA_STATS_IOC_COMPAT_INIT:
	{
		struct ioc_dpa_stats_params kparam;
		struct compat_ioc_dpa_stats_params uparam;

		/* Copy parameters from user space */
		if (copy_from_user(&uparam, (void *)args, sizeof(uparam))) {
			pr_err("Could not read dpa_stats_init user space args\n");
			return -EBUSY;
		}
		dpa_stats_init_compatcpy(&kparam, &uparam);

		ret = do_ioctl_stats_init(&kparam);
		if (ret < 0)
			return ret;

		/* Copy result to user-space */
		uparam.dpa_stats_id = kparam.dpa_stats_id;
		if (copy_to_user((void *)args, &uparam, sizeof(uparam))) {
			pr_err("Could not write dpa_stats_init result\n");
			return -EBUSY;
		}
		break;
	}
	case DPA_STATS_IOC_FREE:
		ret = do_ioctl_stats_free((void *)args);
		if (ret < 0)
			return ret;
		break;
	case DPA_STATS_IOC_COMPAT_CREATE_COUNTER:
		ret = do_ioctl_stats_compat_create_counter((void *)args);
		if (ret < 0)
			return ret;
		break;
	case DPA_STATS_IOC_COMPAT_CREATE_CLASS_COUNTER:
		ret = do_ioctl_stats_compat_create_class_counter((void *)args);
		if (ret < 0)
			return ret;
		break;
	case DPA_STATS_IOC_COMPAT_MODIFY_CLASS_COUNTER:
		ret = do_ioctl_stats_compat_modify_class_counter((void *)args);
		if (ret < 0)
			return ret;
		break;
	case DPA_STATS_IOC_REMOVE_COUNTER:{
		int dpa_stats_cnt_id;

		if (copy_from_user(&dpa_stats_cnt_id, (int *)args,
				    sizeof(int))) {
			pr_err("Could not copy parameters\n");
			return -EINVAL;
		}

		ret = dpa_stats_remove_counter(dpa_stats_cnt_id);
		if (ret < 0)
			return ret;
		break;
	}
	case DPA_STATS_IOC_COMPAT_GET_COUNTERS:
		ret = do_ioctl_stats_compat_get_counters((void *)args);
		if (ret < 0)
			return ret;
		break;
	case DPA_STATS_IOC_COMPAT_RESET_COUNTERS:
		ret = do_ioctl_stats_compat_reset_counters((void *)args);
		if (ret < 0)
			return ret;
		break;
	default:
		pr_err("invalid ioctl: cmd:0x%08x(type:0x%02x, nr:0x%02x.\n",
				cmd, _IOC_TYPE(cmd), _IOC_NR(cmd));
		break;
	}
	return ret;
}
#endif

static long store_get_cnts_async_params(
		struct ioc_dpa_stats_cnt_request_params *kprm)
{
	struct dpa_stats_async_req_ev *async_req_ev;
	struct list_head *async_req_grp;
	uint8_t grp_idx = 0;

	mutex_lock(&wrp_dpa_stats.async_req_lock);
	if (list_empty(&wrp_dpa_stats.async_req_pool)) {
		pr_err("Reached maximum supported number "
			"of simultaneous asynchronous requests\n");
		kfree(kprm->req_params.cnts_ids);
		mutex_unlock(&wrp_dpa_stats.async_req_lock);
		return -EDOM;
	}
	/* Add in the associated group the request event */
	grp_idx = crc8(crc8_table,
			(uint8_t *)&kprm->req_params.storage_area_offset,
			sizeof(unsigned int),
			0);
	async_req_grp = &wrp_dpa_stats.async_req_group[grp_idx];

	/* Obtain a free request event and add in the group list */
	async_req_ev = list_entry(wrp_dpa_stats.async_req_pool.next,
			struct dpa_stats_async_req_ev, node);
	list_del(&async_req_ev->node);
	async_req_ev->request_done = kprm->request_done;
	async_req_ev->storage_area_offset =
			kprm->req_params.storage_area_offset;
	list_add_tail(&async_req_ev->node, async_req_grp);
	mutex_unlock(&wrp_dpa_stats.async_req_lock);

	/* Replace the application callback with wrapper function */
	kprm->request_done = do_ioctl_req_done_cb;
	return 0;
}

static int copy_key_descriptor(struct dpa_offload_lookup_key *src,
			       struct dpa_offload_lookup_key *tmp)
{
	if (!src->byte) {
		pr_err("Key byte pointer can't be NULL\n");
		return -EINVAL;
	}

	/* Allocate memory to store the key byte array */
	tmp->byte = kmalloc(src->size, GFP_KERNEL);
	if (!tmp->byte) {
		pr_err("No more memory for key pointer\n");
		return -ENOMEM;
	}

	if (copy_from_user(tmp->byte, src->byte, src->size)) {
		pr_err("Could not copy key byte\n");
		kfree(tmp->byte);
		return -EBUSY;
	}
	src->byte = tmp->byte;

	if (src->mask) {
		/* Allocate memory to store the key mask array */
		tmp->mask = kmalloc(src->size, GFP_KERNEL);
		if (!tmp->mask) {
			pr_err("No more memory for mask pointer\n");
			kfree(tmp->byte);
			return -ENOMEM;
		}

		if (copy_from_user(tmp->mask, src->mask, src->size)) {
			pr_err("Could not copy key mask\n");
			kfree(tmp->byte);
			kfree(tmp->mask);
			return -EBUSY;
		}
		src->mask = tmp->mask;
	}
	return 0;
}

#ifdef CONFIG_COMPAT
static int copy_key_descriptor_compatcpy(
		struct dpa_offload_lookup_key *kparam,
		const struct compat_ioc_dpa_offld_lookup_key *uparam)
{
	BUG_ON(uparam->size <= 0);

	kparam->size = uparam->size;

	/* Allocate memory to store the key byte array */
	kparam->byte = kmalloc(kparam->size, GFP_KERNEL);
	if (!kparam->byte) {
		pr_err("No more memory for key pointer\n");
		return -ENOMEM;
	}

	if (copy_from_user(kparam->byte, compat_ptr(uparam->byte),
		uparam->size)) {
		pr_err("Could not copy key byte\n");
		return -EBUSY;
	}
	if (compat_ptr(uparam->mask)) {
		/* Allocate memory to store the key mask array */
		kparam->mask = kmalloc(kparam->size, GFP_KERNEL);
		if (!kparam->mask) {
			pr_err("No more memory for mask pointer\n");
			kfree(kparam->byte);
			return -ENOMEM;
		}

		if (copy_from_user(kparam->mask, compat_ptr(uparam->mask),
			uparam->size)) {
			pr_err("Could not copy key mask\n");
			return -EBUSY;
		}
	} else
		kparam->mask = NULL;

	return 0;
}
#endif

static int copy_class_members(void *objs, unsigned int size, void *dst)
{
	/* Allocate memory to store the array of reass objects */
	objs = kmalloc(size * sizeof(void *), GFP_KERNEL);
	if (!objs) {
		pr_err("No more memory for class members pointers\n");
		return -ENOMEM;
	}

	if (copy_from_user(objs, dst, (size * sizeof(void *)))) {
		pr_err("Could not copy array of objects\n");
		kfree(objs);
		return -EBUSY;
	}
	dst = objs;

	return 0;
}

#ifdef CONFIG_COMPAT
static void dpa_stats_init_compatcpy(struct ioc_dpa_stats_params *kparam,
				     struct compat_ioc_dpa_stats_params *uparam)
{
	kparam->dpa_stats_id = uparam->dpa_stats_id;
	kparam->max_counters = uparam->stats_params.max_counters;
	kparam->storage_area_len = uparam->stats_params.storage_area_len;
	kparam->virt_stg_area = compat_ptr(uparam->stats_params.virt_stg_area);
	kparam->phys_stg_area = uparam->stats_params.phys_stg_area;
	kparam->stg_area_mapped = uparam->stats_params.stg_area_mapped;
}

static void dpa_stats_reass_cnt_compatcpy(struct dpa_stats_cnt_reass *kprm,
				struct dpa_stats_compat_cnt_reass *uprm)
{
	kprm->reass = compat_get_id2ptr(uprm->reass, FM_MAP_TYPE_PCD_NODE);
	kprm->cnt_sel = uprm->cnt_sel;
}

static void dpa_stats_frag_cnt_compatcpy(struct dpa_stats_cnt_frag *kprm,
					 struct dpa_stats_compat_cnt_frag *uprm)
{
	kprm->frag = compat_get_id2ptr(uprm->frag, FM_MAP_TYPE_PCD_NODE);
	kprm->cnt_sel = uprm->cnt_sel;
}

static void dpa_stats_plcr_cnt_compatcpy(struct dpa_stats_cnt_plcr *kprm,
					 struct dpa_stats_compat_cnt_plcr *uprm)
{
	kprm->plcr = compat_get_id2ptr(uprm->plcr, FM_MAP_TYPE_PCD_NODE);
	kprm->cnt_sel = uprm->cnt_sel;
}

static long dpa_stats_tbl_cnt_compatcpy(struct dpa_stats_cnt_classif_tbl *kprm,
				struct dpa_stats_compat_cnt_classif_tbl *uprm)
{
	kprm->td = uprm->td;
	kprm->cnt_sel = uprm->cnt_sel;
	return copy_key_descriptor_compatcpy(&kprm->key, &uprm->key);
}

static long dpa_stats_ccnode_cnt_compatcpy(
		struct dpa_stats_cnt_classif_node *kprm,
		struct dpa_stats_compat_cnt_classif_node *uprm)
{
	kprm->cnt_sel = uprm->cnt_sel;
	kprm->ccnode_type = uprm->ccnode_type;
	kprm->cc_node = compat_get_id2ptr(uprm->cc_node, FM_MAP_TYPE_PCD_NODE);
	return copy_key_descriptor_compatcpy(&kprm->key, &uprm->key);
}

static long dpa_stats_eth_cls_compatcpy(struct dpa_stats_cls_cnt_eth *kprm,
		struct dpa_stats_compat_cls_cnt_eth *uprm, uint32_t cls_members)
{
	uint32_t size = 0;

	size = cls_members * sizeof(struct dpa_stats_cnt_eth_src);

	/* Allocate memory to store the sources array */
	kprm->src = kzalloc(size, GFP_KERNEL);
	if (!kprm->src) {
		pr_err("No more memory for ethernet sources array\n");
		return -ENOMEM;
	}

	if (copy_from_user(kprm->src, compat_ptr(uprm->src), size)) {
		pr_err("Could not copy array of ethernet sources\n");
		kfree(kprm->src);
		return -EBUSY;
	}
	kprm->cnt_sel = uprm->cnt_sel;
	return 0;
}

static long dpa_stats_reass_cls_compatcpy(struct dpa_stats_cls_cnt_reass *kprm,
		struct dpa_stats_compat_cnt_reass *uprm, uint32_t cls_members)
{
	compat_uptr_t *reass;
	uint32_t i = 0;

	/* Allocate memory to store the array of user-space reass objects */
	reass = kzalloc(sizeof(compat_uptr_t) * cls_members, GFP_KERNEL);
	if (!reass) {
		pr_err("No more memory for class members pointers\n");
		return -ENOMEM;
	}

	if (copy_from_user(reass, compat_ptr(uprm->reass),
			(sizeof(compat_uptr_t) * cls_members))) {
		pr_err("Could not copy array of objects\n");
		return -EBUSY;
	}

	/* Allocate memory to store the array of kernel space reass objects */
	kprm->reass = kzalloc((sizeof(void *) * cls_members), GFP_KERNEL);
	if (!kprm->reass) {
		pr_err("No more memory for class members pointers\n");
		return -ENOMEM;
	}

	for (i = 0; i < cls_members; i++)
		kprm->reass[i] = compat_get_id2ptr(
				reass[i], FM_MAP_TYPE_PCD_NODE);

	kprm->cnt_sel = uprm->cnt_sel;
	return 0;
}

static long dpa_stats_frag_cls_compatcpy(struct dpa_stats_cls_cnt_frag *kprm,
					 struct dpa_stats_compat_cnt_frag *uprm,
					 uint32_t cls_members)
{
	compat_uptr_t *ufrag;
	uint32_t i = 0;

	/* Allocate memory to store the array of user-space frag objects */
	ufrag = kzalloc(sizeof(compat_uptr_t) * cls_members, GFP_KERNEL);
	if (!ufrag) {
		pr_err("No more memory for class members pointers\n");
		return -ENOMEM;
	}

	if (copy_from_user(ufrag, compat_ptr(uprm->frag),
			(sizeof(compat_uptr_t) * cls_members))) {
		pr_err("Could not copy array of objects\n");
		return -EBUSY;
	}

	/* Allocate memory to store the array of kernel space frag objects */
	kprm->frag = kzalloc((sizeof(void *) * cls_members), GFP_KERNEL);
	if (!kprm->frag) {
		pr_err("No more memory for class members pointers\n");
		return -ENOMEM;
	}

	for (i = 0; i < cls_members; i++)
		kprm->frag[i] = compat_get_id2ptr(
				ufrag[i], FM_MAP_TYPE_PCD_NODE);

	kprm->cnt_sel = uprm->cnt_sel;
	return 0;
}

static long dpa_stats_plcr_cls_compatcpy(struct dpa_stats_cls_cnt_plcr *kprm,
					 struct dpa_stats_compat_cnt_plcr *uprm,
					 uint32_t cls_members)
{
	compat_uptr_t *uplcr;
	uint32_t i = 0;

	/* Allocate memory to store the array of user-space policer objects */
	uplcr = kzalloc(sizeof(compat_uptr_t) * cls_members, GFP_KERNEL);
	if (!uplcr) {
		pr_err("No more memory for class members pointers\n");
		return -ENOMEM;
	}

	if (copy_from_user(uplcr, compat_ptr(uprm->plcr),
			(sizeof(compat_uptr_t) * cls_members))) {
		pr_err("Could not copy array of objects\n");
		return -EBUSY;
	}

	/* Allocate memory to store the array of kernel space policer objects */
	kprm->plcr = kzalloc((sizeof(void *) * cls_members), GFP_KERNEL);
	if (!kprm->plcr) {
		pr_err("No more memory for class members pointers\n");
		return -ENOMEM;
	}

	for (i = 0; i < cls_members; i++)
		kprm->plcr[i] = compat_get_id2ptr(
				uplcr[i], FM_MAP_TYPE_PCD_NODE);

	kprm->cnt_sel = uprm->cnt_sel;
	return 0;
}

static long dpa_stats_tbl_cls_compatcpy(
		struct dpa_stats_cls_cnt_classif_tbl *kprm,
		struct dpa_stats_compat_cls_cnt_classif_tbl *uprm,
		uint32_t cls_members)
{
	struct compat_ioc_dpa_offld_lookup_key *keys;
	struct compat_ioc_dpa_offld_lookup_key_pair *pairs;
	uint32_t size = 0, i;
	long ret;

	kprm->cnt_sel = uprm->cnt_sel;
	kprm->td = uprm->td;
	kprm->key_type = uprm->key_type;

	if (kprm->key_type == DPA_STATS_CLASSIF_SINGLE_KEY) {
		size = sizeof(struct dpa_offload_lookup_key) * cls_members;
		kprm->keys = kzalloc(size, GFP_KERNEL);
		if (!kprm->keys) {
			pr_err("No more memory for class members pointers\n");
			return -ENOMEM;
		}

		size = sizeof(struct compat_ioc_dpa_offld_lookup_key) *
				cls_members;
		keys = kzalloc(size, GFP_KERNEL);
		if (!keys) {
			pr_err("No more memory for class members pointers\n");
			return -ENOMEM;
		}

		if (copy_from_user(keys, (compat_ptr)(uprm->keys), size)) {
			pr_err("Could not copy array of objects\n");
			kfree(keys);
			return -EBUSY;
		}

		for (i = 0; i < cls_members; i++) {
			if (!compat_ptr(keys[i].byte))
				continue;

			ret = copy_key_descriptor_compatcpy(&kprm->keys[i],
					&keys[i]);
			if (ret != 0) {
				pr_err("Couldn't copy the key descriptor\n");
				kfree(keys);
				return -EINVAL;
			}
		}
		kfree(keys);
	} else if (kprm->key_type == DPA_STATS_CLASSIF_PAIR_KEY) {
		size = sizeof(struct dpa_offload_lookup_key_pair) * cls_members;
		kprm->pairs = kzalloc(size, GFP_KERNEL);
		if (!kprm->pairs) {
			pr_err("No more memory for class members pointers\n");
			return -ENOMEM;
		}

		size = sizeof(struct compat_ioc_dpa_offld_lookup_key_pair) *
				cls_members;
		pairs = kzalloc(size, GFP_KERNEL);
		if (!pairs) {
			pr_err("No more memory for class members pointers\n");
			return -ENOMEM;
		}

		if (copy_from_user(pairs, (compat_ptr)(uprm->pairs), size)) {
			pr_err("Could not copy array of objects\n");
			kfree(pairs);
			return -EBUSY;
		}

		for (i = 0; i < cls_members; i++) {
			if (!compat_ptr(pairs[i].first_key.byte))
				continue;

			ret = copy_key_descriptor_compatcpy(
					&kprm->pairs[i].first_key,
					&pairs[i].first_key);
			if (ret != 0) {
				pr_err("Couldn't copy the key descriptor\n");
				kfree(pairs);
				return -EINVAL;
			}

			ret = copy_key_descriptor_compatcpy(
					&kprm->pairs[i].second_key,
					&pairs[i].second_key);
			if (ret != 0) {
				pr_err("Couldn't copy the key descriptor\n");
				kfree(pairs);
				return -EINVAL;
			}
		}
		kfree(pairs);
	}
	return 0;
}

static long dpa_stats_ccnode_cls_compatcpy(
		struct dpa_stats_cls_cnt_classif_node *kprm,
		struct dpa_stats_compat_cls_cnt_classif_node *uprm,
		uint32_t cls_members)
{
	struct compat_ioc_dpa_offld_lookup_key *keys;
	uint32_t size, i;
	long ret = 0;

	kprm->cc_node = compat_get_id2ptr(uprm->cc_node, FM_MAP_TYPE_PCD_NODE);
	kprm->cnt_sel = uprm->cnt_sel;
	kprm->ccnode_type = uprm->ccnode_type;

	size = sizeof(struct dpa_offload_lookup_key) * cls_members;
	kprm->keys = kzalloc(size, GFP_KERNEL);
	if (!kprm->keys) {
		pr_err("No more memory for class members pointers\n");
		return -ENOMEM;
	}

	size = sizeof(struct compat_ioc_dpa_offld_lookup_key) * cls_members;
	keys = kzalloc(size, GFP_KERNEL);
	if (!keys) {
		pr_err("No more memory for class members pointers\n");
		return -ENOMEM;
	}

	if (copy_from_user(keys, (compat_ptr)(uprm->keys), size)) {
		pr_err("Could not copy array of objects\n");
		kfree(keys);
		return -EBUSY;
	}

	for (i = 0; i < cls_members; i++) {
		ret = copy_key_descriptor_compatcpy(&kprm->keys[i], &keys[i]);
		if (ret != 0) {
			pr_err("Couldn't copy the key descriptor\n");
			kfree(keys);
			return -EINVAL;
		}
	}
	kfree(keys);
	return ret;
}

static long dpa_stats_ipsec_cls_compatcpy(struct dpa_stats_cls_cnt_ipsec *kprm,
		struct dpa_stats_compat_cls_cnt_ipsec *uprm,
		uint32_t cls_members)
{
	kprm->cnt_sel = uprm->cnt_sel;

	/* Allocate memory to store the sa ids array */
	kprm->sa_id = kmalloc(cls_members * sizeof(int), GFP_KERNEL);
	if (!kprm->sa_id) {
		pr_err("No more memory for sa ids pointer\n");
		return -ENOMEM;
	}

	if (copy_from_user(kprm->sa_id,
			(compat_ptr)(uprm->sa_id),
			(cls_members * sizeof(int)))) {
		pr_err("Could not copy array of SA ids\n");
		kfree(kprm->sa_id);
		return -EBUSY;
	}
	return 0;
}
#endif
