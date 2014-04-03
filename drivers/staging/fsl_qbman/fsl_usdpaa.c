/* Copyright (C) 2008-2012 Freescale Semiconductor, Inc.
 * Authors: Andy Fleming <afleming@freescale.com>
 *	    Timur Tabi <timur@freescale.com>
 *	    Geoff Thorpe <Geoff.Thorpe@freescale.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/fsl_usdpaa.h>
#include "bman_low.h"
#include "qman_low.h"

#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/memblock.h>
#include <linux/slab.h>
#include <linux/mman.h>

/* Physical address range of the memory reservation, exported for mm/mem.c */
static u64 phys_start;
static u64 phys_size;
/* PFN versions of the above */
static unsigned long pfn_start;
static unsigned long pfn_size;

/* Memory reservations are manipulated under this spinlock (which is why 'refs'
 * isn't atomic_t). */
static DEFINE_SPINLOCK(mem_lock);

/* The range of TLB1 indices */
static unsigned int first_tlb;
static unsigned int num_tlb;
static unsigned int current_tlb; /* loops around for fault handling */

/* Memory reservation is represented as a list of 'mem_fragment's, some of which
 * may be mapped. Unmapped fragments are always merged where possible. */
static LIST_HEAD(mem_list);

struct mem_mapping;

/* Memory fragments are in 'mem_list'. */
struct mem_fragment {
	u64 base;
	u64 len;
	unsigned long pfn_base; /* PFN version of 'base' */
	unsigned long pfn_len; /* PFN version of 'len' */
	unsigned int refs; /* zero if unmapped */
	struct list_head list;
	/* if mapped, flags+name captured at creation time */
	u32 flags;
	char name[USDPAA_DMA_NAME_MAX];
	/* support multi-process locks per-memory-fragment. */
	int has_locking;
	wait_queue_head_t wq;
	struct mem_mapping *owner;
};

/* Mappings of memory fragments in 'struct ctx'. These are created from
 * ioctl(USDPAA_IOCTL_DMA_MAP), though the actual mapping then happens via a
 * mmap(). */
struct mem_mapping {
	struct mem_fragment *frag;
	struct list_head list;
};

struct portal_mapping {
	struct usdpaa_ioctl_portal_map user;
	union {
		struct qm_portal_config *qportal;
		struct bm_portal_config *bportal;
	};
	/* Declare space for the portals in case the process
	   exits unexpectedly and needs to be cleaned by the kernel */
	union {
		struct qm_portal qman_portal_low;
		struct bm_portal bman_portal_low;
	};
	struct list_head list;
	struct resource *phys;
};

/* Track the DPAA resources the process is using */
struct active_resource {
	struct list_head list;
	u32 id;
	u32 num;
	unsigned int refcount;
};

/* Per-FD state (which should also be per-process but we don't enforce that) */
struct ctx {
	/* Lock to protect the context */
	spinlock_t lock;
	/* Allocated resources get put here for accounting */
	struct list_head resources[usdpaa_id_max];
	/* list of DMA maps */
	struct list_head maps;
	/* list of portal maps */
	struct list_head portals;
};

/* Different resource classes */
static const struct alloc_backend {
	enum usdpaa_id_type id_type;
	int (*alloc)(u32 *, u32, u32, int);
	void (*release)(u32 base, unsigned int count);
	int (*reserve)(u32 base, unsigned int count);
	const char *acronym;
} alloc_backends[] = {
	{
		.id_type = usdpaa_id_fqid,
		.alloc = qman_alloc_fqid_range,
		.release = qman_release_fqid_range,
		.reserve = qman_reserve_fqid_range,
		.acronym = "FQID"
	},
	{
		.id_type = usdpaa_id_bpid,
		.alloc = bman_alloc_bpid_range,
		.release = bman_release_bpid_range,
		.acronym = "BPID"
	},
	{
		.id_type = usdpaa_id_qpool,
		.alloc = qman_alloc_pool_range,
		.release = qman_release_pool_range,
		.acronym = "QPOOL"
	},
	{
		.id_type = usdpaa_id_cgrid,
		.alloc = qman_alloc_cgrid_range,
		.release = qman_release_cgrid_range,
		.acronym = "CGRID"
	},
	{
		.id_type = usdpaa_id_ceetm0_lfqid,
		.alloc = qman_alloc_ceetm0_lfqid_range,
		.release = qman_release_ceetm0_lfqid_range,
		.acronym = "CEETM0_LFQID"
	},
	{
		.id_type = usdpaa_id_ceetm0_channelid,
		.alloc = qman_alloc_ceetm0_channel_range,
		.release = qman_release_ceetm0_channel_range,
		.acronym = "CEETM0_LFQID"
	},
	{
		.id_type = usdpaa_id_ceetm1_lfqid,
		.alloc = qman_alloc_ceetm1_lfqid_range,
		.release = qman_release_ceetm1_lfqid_range,
		.acronym = "CEETM1_LFQID"
	},
	{
		.id_type = usdpaa_id_ceetm1_channelid,
		.alloc = qman_alloc_ceetm1_channel_range,
		.release = qman_release_ceetm1_channel_range,
		.acronym = "CEETM1_LFQID"
	},
	{
		/* This terminates the array */
		.id_type = usdpaa_id_max
	}
};

/* Helper for ioctl_dma_map() when we have a larger fragment than we need. This
 * splits the fragment into 4 and returns the upper-most. (The caller can loop
 * until it has a suitable fragment size.) */
static struct mem_fragment *split_frag(struct mem_fragment *frag)
{
	struct mem_fragment *x[3];
	x[0] = kmalloc(sizeof(struct mem_fragment), GFP_KERNEL);
	x[1] = kmalloc(sizeof(struct mem_fragment), GFP_KERNEL);
	x[2] = kmalloc(sizeof(struct mem_fragment), GFP_KERNEL);
	if (!x[0] || !x[1] || !x[2]) {
		kfree(x[0]);
		kfree(x[1]);
		kfree(x[2]);
		return NULL;
	}
	BUG_ON(frag->refs);
	frag->len >>= 2;
	frag->pfn_len >>= 2;
	x[0]->base = frag->base + frag->len;
	x[1]->base = x[0]->base + frag->len;
	x[2]->base = x[1]->base + frag->len;
	x[0]->len = x[1]->len = x[2]->len = frag->len;
	x[0]->pfn_base = frag->pfn_base + frag->pfn_len;
	x[1]->pfn_base = x[0]->pfn_base + frag->pfn_len;
	x[2]->pfn_base = x[1]->pfn_base + frag->pfn_len;
	x[0]->pfn_len = x[1]->pfn_len = x[2]->pfn_len = frag->pfn_len;
	x[0]->refs = x[1]->refs = x[2]->refs = 0;
	list_add(&x[0]->list, &frag->list);
	list_add(&x[1]->list, &x[0]->list);
	list_add(&x[2]->list, &x[1]->list);
	return x[2];
}

/* Conversely, when a fragment is released we look to see whether its
 * similarly-split siblings are free to be reassembled. */
static struct mem_fragment *merge_frag(struct mem_fragment *frag)
{
	/* If this fragment can be merged with its siblings, it will have
	 * newbase and newlen as its geometry. */
	uint64_t newlen = frag->len << 2;
	uint64_t newbase = frag->base & ~(newlen - 1);
	struct mem_fragment *tmp, *leftmost = frag, *rightmost = frag;
	/* Scan left until we find the start */
	tmp = list_entry(frag->list.prev, struct mem_fragment, list);
	while ((&tmp->list != &mem_list) && (tmp->base >= newbase)) {
		if (tmp->refs)
			return NULL;
		if (tmp->len != tmp->len)
			return NULL;
		leftmost = tmp;
		tmp = list_entry(tmp->list.prev, struct mem_fragment, list);
	}
	/* Scan right until we find the end */
	tmp = list_entry(frag->list.next, struct mem_fragment, list);
	while ((&tmp->list != &mem_list) && (tmp->base < (newbase + newlen))) {
		if (tmp->refs)
			return NULL;
		if (tmp->len != tmp->len)
			return NULL;
		rightmost = tmp;
		tmp = list_entry(tmp->list.next, struct mem_fragment, list);
	}
	if (leftmost == rightmost)
		return NULL;
	/* OK, we can merge */
	frag = leftmost;
	frag->len = newlen;
	frag->pfn_len = newlen >> PAGE_SHIFT;
	while (1) {
		int lastone;
		tmp = list_entry(frag->list.next, struct mem_fragment, list);
		lastone = (tmp == rightmost);
		if (&tmp->list == &mem_list)
			break;
		list_del(&tmp->list);
		kfree(tmp);
		if (lastone)
			break;
	}
	return frag;
}

/* Helper to verify that 'sz' is (4096 * 4^x) for some x. */
static int is_good_size(u64 sz)
{
	int log = ilog2(phys_size);
	if ((phys_size & (phys_size - 1)) || (log < 12) || (log & 1))
		return 0;
	return 1;
}

/* Hook from arch/powerpc/mm/mem.c */
int usdpaa_test_fault(unsigned long pfn, u64 *phys_addr, u64 *size)
{
	struct mem_fragment *frag;
	int idx = -1;
	if ((pfn < pfn_start) || (pfn >= (pfn_start + pfn_size)))
		return -1;
	/* It's in-range, we need to find the fragment */
	spin_lock(&mem_lock);
	list_for_each_entry(frag, &mem_list, list) {
		if ((pfn >= frag->pfn_base) && (pfn < (frag->pfn_base +
						       frag->pfn_len))) {
			*phys_addr = frag->base;
			*size = frag->len;
			idx = current_tlb++;
			if (current_tlb >= (first_tlb + num_tlb))
				current_tlb = first_tlb;
			break;
		}
	}
	spin_unlock(&mem_lock);
	return idx;
}

static int usdpaa_open(struct inode *inode, struct file *filp)
{
	const struct alloc_backend *backend = &alloc_backends[0];
	struct ctx *ctx = kmalloc(sizeof(struct ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	filp->private_data = ctx;

	while (backend->id_type != usdpaa_id_max) {
		INIT_LIST_HEAD(&ctx->resources[backend->id_type]);
		backend++;
	}

	INIT_LIST_HEAD(&ctx->maps);
	INIT_LIST_HEAD(&ctx->portals);
	spin_lock_init(&ctx->lock);

	filp->f_mapping->backing_dev_info = &directly_mappable_cdev_bdi;

	return 0;
}


#define DQRR_MAXFILL 15

/* Reset a QMan portal to its default state */
static int init_qm_portal(struct qm_portal_config *config,
			  struct qm_portal *portal)
{
	portal->addr.addr_ce = config->addr_virt[DPA_PORTAL_CE];
	portal->addr.addr_ci = config->addr_virt[DPA_PORTAL_CI];

	/* Initialize the DQRR.  This will stop any dequeue
	   commands that are in progress */
	if (qm_dqrr_init(portal, config, qm_dqrr_dpush, qm_dqrr_pvb,
			 qm_dqrr_cdc, DQRR_MAXFILL)) {
		pr_err("qm_dqrr_init() failed when trying to"
		       " recover portal, portal will be leaked\n");
		return 1;
	}
	/* Consume any items in the dequeue ring */
	qm_dqrr_cdc_consume_n(portal, 0xffff);

	/* Initialize the EQCR */
	if (qm_eqcr_init(portal, qm_eqcr_pvb, qm_eqcr_cce)) {
		pr_err("Qman EQCR initialisation failed\n");
		return 1;
	}
	/* initialize the MR */
	if (qm_mr_init(portal, qm_mr_pvb, qm_mr_cci)) {
		pr_err("Qman MR initialisation failed\n");
		return 1;
	}
	qm_mr_pvb_update(portal);
	while (qm_mr_current(portal)) {
		qm_mr_next(portal);
		qm_mr_cci_consume_to_current(portal);
		qm_mr_pvb_update(portal);
	}

	if (qm_mc_init(portal)) {
		pr_err("Qman MC initialisation failed\n");
		return 1;
	}
	return 0;
}

static int init_bm_portal(struct bm_portal_config *config,
			  struct bm_portal *portal)
{
	portal->addr.addr_ce = config->addr_virt[DPA_PORTAL_CE];
	portal->addr.addr_ci = config->addr_virt[DPA_PORTAL_CI];

	if (bm_rcr_init(portal, bm_rcr_pvb, bm_rcr_cce)) {
		pr_err("Bman RCR initialisation failed\n");
	return 1;
	}
	if (bm_mc_init(portal)) {
		pr_err("Bman MC initialisation failed\n");
		return 1;
	}
	return 0;
}

/* Function that will scan all FQ's in the system.  For each FQ that is not
   OOS it will call the check_channel helper to determine if the FQ should
   be torn down.  If the check_channel helper returns true the FQ will be
   transitioned to the OOS state */
static int qm_check_and_destroy_fqs(struct qm_portal *portal, void *ctx,
				    bool (*check_channel)
				    (void *ctx, u32 channel))
{
	u32 fq_id = 0;
	while (1) {
		struct qm_mc_command *mcc;
		struct qm_mc_result *mcr;
		u8 state;
		u32 channel;

		/* Determine the channel for the FQID */
		mcc = qm_mc_start(portal);
		mcc->queryfq.fqid = fq_id;
		qm_mc_commit(portal, QM_MCC_VERB_QUERYFQ);
		while (!(mcr = qm_mc_result(portal)))
			cpu_relax();
		DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK)
			   == QM_MCR_VERB_QUERYFQ);
		if (mcr->result != QM_MCR_RESULT_OK)
			break; /* End of valid FQIDs */

		channel = mcr->queryfq.fqd.dest.channel;
		/* Determine the state of the FQID */
		mcc = qm_mc_start(portal);
		mcc->queryfq_np.fqid = fq_id;
		qm_mc_commit(portal, QM_MCC_VERB_QUERYFQ_NP);
		while (!(mcr = qm_mc_result(portal)))
			cpu_relax();
		DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK)
			   == QM_MCR_VERB_QUERYFQ_NP);
		state = mcr->queryfq_np.state & QM_MCR_NP_STATE_MASK;
		if (state == QM_MCR_NP_STATE_OOS)
			/* Already OOS, no need to do anymore checks */
			goto next;

		if (check_channel(ctx, channel))
			qm_shutdown_fq(portal, fq_id);
 next:
		++fq_id;
	}
	return 0;

}

static bool check_channel_device(void *_ctx, u32 channel)
{
	struct ctx *ctx = _ctx;
	struct portal_mapping *portal, *tmpportal;
	struct active_resource *res;

	/* See if the FQ is destined for one of the portals we're cleaning up */
	list_for_each_entry_safe(portal, tmpportal, &ctx->portals, list) {
		if (portal->user.type == usdpaa_portal_qman) {
			if (portal->qportal->public_cfg.channel == channel) {
				/* This FQs destination is a portal
				   we're cleaning, send a retire */
				return true;
			}
		}
	}

	/* Check the pool channels that will be released as well */
	list_for_each_entry(res, &ctx->resources[usdpaa_id_qpool], list) {
		if ((res->id >= channel) &&
		    ((res->id + res->num - 1) <= channel))
			return true;
	}
	return false;
}



static int usdpaa_release(struct inode *inode, struct file *filp)
{
	struct ctx *ctx = filp->private_data;
	struct mem_mapping *map, *tmpmap;
	struct portal_mapping *portal, *tmpportal;
	const struct alloc_backend *backend = &alloc_backends[0];
	struct active_resource *res;
	struct qm_portal *qm_cleanup_portal = NULL;
	struct bm_portal *bm_cleanup_portal = NULL;
	struct qm_portal_config *qm_alloced_portal = NULL;
	struct bm_portal_config *bm_alloced_portal = NULL;

	/* The following logic is used to recover resources that were not
	   correctly released by the process that is closing the FD.
	   Step 1: syncronize the HW with the qm_portal/bm_portal structures
	   in the kernel
	*/

	list_for_each_entry_safe(portal, tmpportal, &ctx->portals, list) {
		/* Try to recover any portals that weren't shut down */
		if (portal->user.type == usdpaa_portal_qman) {
			init_qm_portal(portal->qportal,
				       &portal->qman_portal_low);
			if (!qm_cleanup_portal)
				qm_cleanup_portal = &portal->qman_portal_low;
		} else {
			/* BMAN */
			init_bm_portal(portal->bportal,
				       &portal->bman_portal_low);
			if (!bm_cleanup_portal)
				bm_cleanup_portal = &portal->bman_portal_low;
		}
	}
	/* If no portal was found, allocate one for cleanup */
	if (!qm_cleanup_portal) {
		qm_alloced_portal = qm_get_unused_portal();
		if (!qm_alloced_portal) {
			pr_crit("No QMan portal avalaible for cleanup\n");
			return -1;
		}
		qm_cleanup_portal = kmalloc(sizeof(struct qm_portal),
					    GFP_KERNEL);
		if (!qm_cleanup_portal)
			return -ENOMEM;
		init_qm_portal(qm_alloced_portal, qm_cleanup_portal);

	}
	if (!bm_cleanup_portal) {
		bm_alloced_portal = bm_get_unused_portal();
		if (!bm_alloced_portal) {
			pr_crit("No BMan portal avalaible for cleanup\n");
			return -1;
		}
		bm_cleanup_portal = kmalloc(sizeof(struct bm_portal),
					    GFP_KERNEL);
		if (!bm_cleanup_portal)
			return -ENOMEM;
		init_bm_portal(bm_alloced_portal, bm_cleanup_portal);
	}

	/* OOS the FQs associated with this process */
	qm_check_and_destroy_fqs(qm_cleanup_portal, ctx, check_channel_device);

	while (backend->id_type != usdpaa_id_max) {
		int leaks = 0;
		list_for_each_entry(res, &ctx->resources[backend->id_type],
				    list) {
			leaks += res->num;
			backend->release(res->id, res->num);
		}
		if (leaks)
			pr_crit("USDPAA process leaking %d %s%s\n", leaks,
				backend->acronym, (leaks > 1) ? "s" : "");
		backend++;
	}
	/* Release any DMA regions */
	spin_lock(&mem_lock);
	list_for_each_entry_safe(map, tmpmap, &ctx->maps, list) {
		if (map->frag->has_locking && (map->frag->owner == map)) {
			map->frag->owner = NULL;
			wake_up(&map->frag->wq);
		}
		if (!--map->frag->refs) {
			struct mem_fragment *frag = map->frag;
			do {
				frag = merge_frag(frag);
			} while (frag);
		}
		list_del(&map->list);
		kfree(map);
	}
	spin_unlock(&mem_lock);

	/* Return portals */
	list_for_each_entry_safe(portal, tmpportal, &ctx->portals, list) {
		if (portal->user.type == usdpaa_portal_qman) {
			/* Give the portal back to the allocator */
			qm_put_unused_portal(portal->qportal);
		} else {
			bm_put_unused_portal(portal->bportal);
		}
		list_del(&portal->list);
		kfree(portal);
	}
	if (qm_alloced_portal) {
		qm_put_unused_portal(qm_alloced_portal);
		kfree(qm_cleanup_portal);
	}
	if (bm_alloced_portal) {
		bm_put_unused_portal(bm_alloced_portal);
		kfree(bm_cleanup_portal);
	}

	kfree(ctx);
	return 0;
}

static int check_mmap_dma(struct ctx *ctx, struct vm_area_struct *vma,
			  int *match, unsigned long *pfn)
{
	struct mem_mapping *map;

	list_for_each_entry(map, &ctx->maps, list) {
		if (map->frag->pfn_base == vma->vm_pgoff) {
			*match = 1;
			if (map->frag->len != (vma->vm_end - vma->vm_start))
				return -EINVAL;
			*pfn = map->frag->pfn_base;
			return 0;
		}
	}
	*match = 0;
	return 0;
}

static int check_mmap_resource(struct resource *res, struct vm_area_struct *vma,
			       int *match, unsigned long *pfn)
{
	*pfn = res->start >> PAGE_SHIFT;
	if (*pfn == vma->vm_pgoff) {
		*match = 1;
		if ((vma->vm_end - vma->vm_start) != resource_size(res))
			return -EINVAL;
	} else
		*match = 0;
	return 0;
}

static int check_mmap_portal(struct ctx *ctx, struct vm_area_struct *vma,
			      int *match, unsigned long *pfn)
{
	struct portal_mapping *portal;
	int ret;

	list_for_each_entry(portal, &ctx->portals, list) {
		ret = check_mmap_resource(&portal->phys[DPA_PORTAL_CE], vma,
					  match, pfn);
		if (*match) {
			vma->vm_page_prot =
				pgprot_cached_noncoherent(vma->vm_page_prot);
			return ret;
		}
		ret = check_mmap_resource(&portal->phys[DPA_PORTAL_CI], vma,
					  match, pfn);
		if (*match) {
			vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
			return ret;
		}
	}
	*match = 0;
	return 0;
}

static int usdpaa_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct ctx *ctx = filp->private_data;
	unsigned long pfn;
	int match, ret;

	spin_lock(&mem_lock);
	ret = check_mmap_dma(ctx, vma, &match, &pfn);
	if (!match)
		ret = check_mmap_portal(ctx, vma, &match, &pfn);
	spin_unlock(&mem_lock);
	if (!match)
		return -EINVAL;
	if (!ret)
		ret = remap_pfn_range(vma, vma->vm_start, pfn,
				      vma->vm_end - vma->vm_start,
				      vma->vm_page_prot);
	return ret;
}

/* Return the nearest rounded-up address >= 'addr' that is 'sz'-aligned. 'sz'
 * must be a power of 2, but both 'addr' and 'sz' can be expressions. */
#define USDPAA_MEM_ROUNDUP(addr, sz) \
	({ \
		unsigned long foo_align = (sz) - 1; \
		((addr) + foo_align) & ~foo_align; \
	})
/* Searching for a size-aligned virtual address range starting from 'addr' */
static unsigned long usdpaa_get_unmapped_area(struct file *file,
					      unsigned long addr,
					      unsigned long len,
					      unsigned long pgoff,
					      unsigned long flags)
{
	struct vm_area_struct *vma;

	if (!is_good_size(len))
		return -EINVAL;

	addr = USDPAA_MEM_ROUNDUP(addr, len);
	vma = find_vma(current->mm, addr);
	/* Keep searching until we reach the end of currently-used virtual
	 * address-space or we find a big enough gap. */
	while (vma) {
		if ((addr + len) < vma->vm_start)
			return addr;
		addr = USDPAA_MEM_ROUNDUP(vma->vm_end, len);
		vma = vma->vm_next;
	}
	if ((TASK_SIZE - len) < addr)
		return -ENOMEM;
	return addr;
}

static long ioctl_id_alloc(struct ctx *ctx, void __user *arg)
{
	struct usdpaa_ioctl_id_alloc i;
	const struct alloc_backend *backend;
	struct active_resource *res;
	int ret = copy_from_user(&i, arg, sizeof(i));
	if (ret)
		return ret;
	if ((i.id_type >= usdpaa_id_max) || !i.num)
		return -EINVAL;
	backend = &alloc_backends[i.id_type];
	/* Allocate the required resource type */
	ret = backend->alloc(&i.base, i.num, i.align, i.partial);
	if (ret < 0)
		return ret;
	i.num = ret;
	/* Copy the result to user-space */
	ret = copy_to_user(arg, &i, sizeof(i));
	if (ret) {
		backend->release(i.base, i.num);
		return ret;
	}
	/* Assign the allocated range to the FD accounting */
	res = kmalloc(sizeof(*res), GFP_KERNEL);
	if (!res) {
		backend->release(i.base, i.num);
		return -ENOMEM;
	}
	spin_lock(&ctx->lock);
	res->id = i.base;
	res->num = i.num;
	res->refcount = 1;
	list_add(&res->list, &ctx->resources[i.id_type]);
	spin_unlock(&ctx->lock);
	return 0;
}

static long ioctl_id_release(struct ctx *ctx, void __user *arg)
{
	struct usdpaa_ioctl_id_release i;
	const struct alloc_backend *backend;
	struct active_resource *tmp, *pos;

	int ret = copy_from_user(&i, arg, sizeof(i));
	if (ret)
		return ret;
	if ((i.id_type >= usdpaa_id_max) || !i.num)
		return -EINVAL;
	backend = &alloc_backends[i.id_type];
	/* Pull the range out of the FD accounting - the range is valid iff this
	 * succeeds. */
	spin_lock(&ctx->lock);
	list_for_each_entry_safe(pos, tmp, &ctx->resources[i.id_type], list) {
		if (pos->id == i.base && pos->num == i.num) {
			pos->refcount--;
			if (pos->refcount) {
				spin_unlock(&ctx->lock);
				return 0; /* Still being used */
			}
			list_del(&pos->list);
			kfree(pos);
			spin_unlock(&ctx->lock);
			goto found;
		}
	}
	/* Failed to find the resource */
	spin_unlock(&ctx->lock);
	return -EINVAL;
found:
	/* Release the resource to the backend */
	backend->release(i.base, i.num);
	return 0;
}

static long ioctl_id_reserve(struct ctx *ctx, void __user *arg)
{
	struct usdpaa_ioctl_id_reserve i;
	const struct alloc_backend *backend;
	struct active_resource *tmp, *pos;

	int ret = copy_from_user(&i, arg, sizeof(i));
	if (ret)
		return ret;
	if ((i.id_type >= usdpaa_id_max) || !i.num)
		return -EINVAL;
	backend = &alloc_backends[i.id_type];
	if (!backend->reserve)
		return -EINVAL;
	/* Pull the range out of the FD accounting - the range is valid iff this
	 * succeeds. */
	spin_lock(&ctx->lock);
	list_for_each_entry_safe(pos, tmp, &ctx->resources[i.id_type], list) {
		if (pos->id == i.base && pos->num == i.num) {
			pos->refcount++;
			spin_unlock(&ctx->lock);
			return 0;
		}
	}

	/* Failed to find the resource */
	spin_unlock(&ctx->lock);

	/* Reserve the resource in the backend */
	ret = backend->reserve(i.base, i.num);
	if (ret)
		return ret;
	/* Assign the reserved range to the FD accounting */
	pos = kmalloc(sizeof(*pos), GFP_KERNEL);
	if (!pos) {
		backend->release(i.base, i.num);
		return -ENOMEM;
	}
	spin_lock(&ctx->lock);
	pos->id = i.base;
	pos->num = i.num;
	pos->refcount = 1;
	list_add(&pos->list, &ctx->resources[i.id_type]);
	spin_unlock(&ctx->lock);
	return 0;
}

static long ioctl_dma_map(struct file *fp, struct ctx *ctx,
			  struct usdpaa_ioctl_dma_map *i)
{
	struct mem_fragment *frag;
	struct mem_mapping *map, *tmp;
	u64 search_size;
	int ret = 0;
	if (i->len && !is_good_size(i->len))
		return -EINVAL;
	map = kmalloc(sizeof(*map), GFP_KERNEL);
	if (!map)
		return -ENOMEM;
	spin_lock(&mem_lock);
	if (i->flags & USDPAA_DMA_FLAG_SHARE) {
		list_for_each_entry(frag, &mem_list, list) {
			if (frag->refs && (frag->flags &
					   USDPAA_DMA_FLAG_SHARE) &&
					!strncmp(i->name, frag->name,
						 USDPAA_DMA_NAME_MAX)) {
				/* Matching entry */
				if ((i->flags & USDPAA_DMA_FLAG_CREATE) &&
				    !(i->flags & USDPAA_DMA_FLAG_LAZY)) {
					ret = -EBUSY;
					goto out;
				}
				list_for_each_entry(tmp, &ctx->maps, list)
					if (tmp->frag == frag) {
						ret = -EBUSY;
						goto out;
					}
				i->has_locking = frag->has_locking;
				i->did_create = 0;
				i->len = frag->len;
				goto do_map;
			}
		}
		/* No matching entry */
		if (!(i->flags & USDPAA_DMA_FLAG_CREATE)) {
			ret = -ENOMEM;
			goto out;
		}
	}
	/* New fragment required, size must be provided. */
	if (!i->len) {
		ret = -EINVAL;
		goto out;
	}
	/* We search for the required size and if that fails, for the next
	 * biggest size, etc. */
	for (search_size = i->len; search_size <= phys_size;
	     search_size <<= 2) {
		list_for_each_entry(frag, &mem_list, list) {
			if (!frag->refs && (frag->len == search_size)) {
				while (frag->len > i->len) {
					frag = split_frag(frag);
					if (!frag) {
						ret = -ENOMEM;
						goto out;
					}
				}
				frag->flags = i->flags;
				strncpy(frag->name, i->name,
					USDPAA_DMA_NAME_MAX);
				frag->has_locking = i->has_locking;
				init_waitqueue_head(&frag->wq);
				frag->owner = NULL;
				i->did_create = 1;
				goto do_map;
			}
		}
	}
	ret = -ENOMEM;
	goto out;

do_map:
	map->frag = frag;
	frag->refs++;
	list_add(&map->list, &ctx->maps);
	i->phys_addr = frag->base;

out:
	spin_unlock(&mem_lock);
	if (!ret) {
		unsigned long longret;
		down_write(&current->mm->mmap_sem);
		longret = do_mmap_pgoff(fp, PAGE_SIZE, map->frag->len, PROT_READ |
			(i->flags & USDPAA_DMA_FLAG_RDONLY ? 0 : PROT_WRITE),
			MAP_SHARED, map->frag->pfn_base);
		up_write(&current->mm->mmap_sem);
		if (longret & ~PAGE_MASK)
			ret = (int)longret;
		else
			i->ptr = (void *)longret;
	} else
		kfree(map);
	return ret;
}

static long ioctl_dma_unmap(struct ctx *ctx, void __user *arg)
{
	struct mem_mapping *map;
	struct vm_area_struct *vma;
	int ret;

	down_write(&current->mm->mmap_sem);
	vma = find_vma(current->mm, (unsigned long)arg);
	if (!vma || (vma->vm_start > (unsigned long)arg)) {
		up_write(&current->mm->mmap_sem);
		return -EFAULT;
	}
	spin_lock(&mem_lock);
	list_for_each_entry(map, &ctx->maps, list) {
		if (map->frag->pfn_base == vma->vm_pgoff) {
			/* Drop the map lock if we hold it */
			if (map->frag->has_locking &&
					(map->frag->owner == map)) {
				map->frag->owner = NULL;
				wake_up(&map->frag->wq);
			}
			goto map_match;
		}
	}
	map = NULL;
map_match:
	spin_unlock(&mem_lock);
	if (map) {
		unsigned long base = vma->vm_start;
		size_t sz = vma->vm_end - vma->vm_start;
		do_munmap(current->mm, base, sz);
		ret = 0;
	} else
		ret = -EFAULT;
	up_write(&current->mm->mmap_sem);
	return ret;
}

static long ioctl_dma_stats(struct ctx *ctx, void __user *arg)
{
	struct mem_fragment *frag;
	struct usdpaa_ioctl_dma_used result;

	result.free_bytes = 0;
	result.total_bytes = phys_size;

	list_for_each_entry(frag, &mem_list, list) {
		result.free_bytes += frag->len;
	}

	return copy_to_user(arg, &result, sizeof(result)); }

static int test_lock(struct mem_mapping *map)
{
	int ret = 0;
	spin_lock(&mem_lock);
	if (!map->frag->owner) {
		map->frag->owner = map;
		ret = 1;
	}
	spin_unlock(&mem_lock);
	return ret;
}

static long ioctl_dma_lock(struct ctx *ctx, void __user *arg)
{
	struct mem_mapping *map;
	struct vm_area_struct *vma;

	down_read(&current->mm->mmap_sem);
	vma = find_vma(current->mm, (unsigned long)arg);
	if (!vma || (vma->vm_start > (unsigned long)arg)) {
		up_read(&current->mm->mmap_sem);
		return -EFAULT;
	}
	spin_lock(&mem_lock);
	list_for_each_entry(map, &ctx->maps, list) {
		if (map->frag->pfn_base == vma->vm_pgoff)
			goto map_match;
	}
	map = NULL;
map_match:
	spin_unlock(&mem_lock);
	up_read(&current->mm->mmap_sem);

	if (!map->frag->has_locking)
		return -ENODEV;
	return wait_event_interruptible(map->frag->wq, test_lock(map));
}

static long ioctl_dma_unlock(struct ctx *ctx, void __user *arg)
{
	struct mem_mapping *map;
	struct vm_area_struct *vma;
	int ret;

	down_read(&current->mm->mmap_sem);
	vma = find_vma(current->mm, (unsigned long)arg);
	if (!vma || (vma->vm_start > (unsigned long)arg))
		ret = -EFAULT;
	else {
		spin_lock(&mem_lock);
		list_for_each_entry(map, &ctx->maps, list) {
			if (map->frag->pfn_base == vma->vm_pgoff) {
				if (!map->frag->has_locking)
					ret = -ENODEV;
				else if (map->frag->owner == map) {
					map->frag->owner = NULL;
					wake_up(&map->frag->wq);
					ret = 0;
				} else
					ret = -EBUSY;
				goto map_match;
			}
		}
		ret = -EINVAL;
map_match:
		spin_unlock(&mem_lock);
	}
	up_read(&current->mm->mmap_sem);
	return ret;
}

static int portal_mmap(struct file *fp, struct resource *res, void **ptr)
{
	unsigned long longret = 0;

	down_write(&current->mm->mmap_sem);
	longret = do_mmap_pgoff(fp, PAGE_SIZE, resource_size(res),
				PROT_READ | PROT_WRITE, MAP_SHARED,
				res->start >> PAGE_SHIFT);
	up_write(&current->mm->mmap_sem);

	if (longret & ~PAGE_MASK)
		return (int)longret;

	*ptr = (void *) longret;
	return 0;
}

static void portal_munmap(struct resource *res, void  *ptr)
{
	down_write(&current->mm->mmap_sem);
	do_munmap(current->mm, (unsigned long)ptr, resource_size(res));
	up_write(&current->mm->mmap_sem);
}

static long ioctl_portal_map(struct file *fp, struct ctx *ctx,
			     struct usdpaa_ioctl_portal_map  *arg)
{
	struct portal_mapping *mapping = kmalloc(sizeof(*mapping), GFP_KERNEL);
	int ret;

	if (!mapping)
		return -ENOMEM;
	memcpy(&mapping->user, arg, sizeof(mapping->user));
	if (mapping->user.type == usdpaa_portal_qman) {
		mapping->qportal = qm_get_unused_portal();
		if (!mapping->qportal) {
			ret = -ENODEV;
			goto err_get_portal;
		}
		mapping->phys = &mapping->qportal->addr_phys[0];
		mapping->user.channel = mapping->qportal->public_cfg.channel;
		mapping->user.pools = mapping->qportal->public_cfg.pools;
	} else if (mapping->user.type == usdpaa_portal_bman) {
		mapping->bportal = bm_get_unused_portal();
		if (!mapping->bportal) {
			ret = -ENODEV;
			goto err_get_portal;
		}
		mapping->phys = &mapping->bportal->addr_phys[0];
	} else {
		ret = -EINVAL;
		goto err_copy_from_user;
	}
	/* Need to put pcfg in ctx's list before the mmaps because the mmap
	 * handlers look it up. */
	spin_lock(&mem_lock);
	list_add(&mapping->list, &ctx->portals);
	spin_unlock(&mem_lock);
	ret = portal_mmap(fp, &mapping->phys[DPA_PORTAL_CE],
			  &mapping->user.addr.cena);
	if (ret)
		goto err_mmap_cena;
	ret = portal_mmap(fp, &mapping->phys[DPA_PORTAL_CI],
			  &mapping->user.addr.cinh);
	if (ret)
		goto err_mmap_cinh;
	memcpy(arg, &mapping->user, sizeof(mapping->user));
	return ret;

err_mmap_cinh:
	portal_munmap(&mapping->phys[DPA_PORTAL_CE], mapping->user.addr.cena);
err_mmap_cena:
	if ((mapping->user.type == usdpaa_portal_qman) && mapping->qportal)
		qm_put_unused_portal(mapping->qportal);
	else if ((mapping->user.type == usdpaa_portal_bman) && mapping->bportal)
		bm_put_unused_portal(mapping->bportal);
	spin_lock(&mem_lock);
	list_del(&mapping->list);
	spin_unlock(&mem_lock);
err_get_portal:
err_copy_from_user:
	kfree(mapping);
	return ret;
}

static bool check_portal_channel(void *ctx, u32 channel)
{
	u32 portal_channel = *(u32 *)ctx;
	if (portal_channel == channel) {
		/* This FQs destination is a portal
		   we're cleaning, send a retire */
		return true;
	}
	return false;
}

static long ioctl_portal_unmap(struct ctx *ctx, struct usdpaa_portal_map *i)
{
	struct portal_mapping *mapping;
	struct vm_area_struct *vma;
	unsigned long pfn;
	u32 channel;

	/* Get the PFN corresponding to one of the virt addresses */
	down_read(&current->mm->mmap_sem);
	vma = find_vma(current->mm, (unsigned long)i->cinh);
	if (!vma || (vma->vm_start > (unsigned long)i->cinh)) {
		up_read(&current->mm->mmap_sem);
		return -EFAULT;
	}
	pfn = vma->vm_pgoff;
	up_read(&current->mm->mmap_sem);

	/* Find the corresponding portal */
	spin_lock(&mem_lock);
	list_for_each_entry(mapping, &ctx->portals, list) {
		if (pfn == (mapping->phys[DPA_PORTAL_CI].start >> PAGE_SHIFT))
			goto found;
	}
	mapping = NULL;
found:
	if (mapping)
		list_del(&mapping->list);
	spin_unlock(&mem_lock);
	if (!mapping)
		return -ENODEV;
	portal_munmap(&mapping->phys[DPA_PORTAL_CI], mapping->user.addr.cinh);
	portal_munmap(&mapping->phys[DPA_PORTAL_CE], mapping->user.addr.cena);
	if (mapping->user.type == usdpaa_portal_qman) {
		init_qm_portal(mapping->qportal,
				       &mapping->qman_portal_low);

		/* Tear down any FQs this portal is referencing */
		channel = mapping->qportal->public_cfg.channel;
		qm_check_and_destroy_fqs(&mapping->qman_portal_low, &channel,
					 check_portal_channel);
		qm_put_unused_portal(mapping->qportal);
	} else if (mapping->user.type == usdpaa_portal_bman) {
		init_bm_portal(mapping->bportal,
			       &mapping->bman_portal_low);
		bm_put_unused_portal(mapping->bportal);
	}
	kfree(mapping);
	return 0;
}

static long usdpaa_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct ctx *ctx = fp->private_data;
	void __user *a = (void __user *)arg;
	switch (cmd) {
	case USDPAA_IOCTL_ID_ALLOC:
		return ioctl_id_alloc(ctx, a);
	case USDPAA_IOCTL_ID_RELEASE:
		return ioctl_id_release(ctx, a);
	case USDPAA_IOCTL_ID_RESERVE:
		return ioctl_id_reserve(ctx, a);
	case USDPAA_IOCTL_DMA_MAP:
	{
		struct usdpaa_ioctl_dma_map input;
		int ret;
		if (copy_from_user(&input, a, sizeof(input)))
			return -EFAULT;
		ret = ioctl_dma_map(fp, ctx, &input);
		if (copy_to_user(a, &input, sizeof(input)))
			return -EFAULT;
		return ret;
	}
	case USDPAA_IOCTL_DMA_UNMAP:
		return ioctl_dma_unmap(ctx, a);
	case USDPAA_IOCTL_DMA_LOCK:
		return ioctl_dma_lock(ctx, a);
	case USDPAA_IOCTL_DMA_UNLOCK:
		return ioctl_dma_unlock(ctx, a);
	case USDPAA_IOCTL_PORTAL_MAP:
	{
		struct usdpaa_ioctl_portal_map input;
		int ret;
		if (copy_from_user(&input, a, sizeof(input)))
			return -EFAULT;
		ret =  ioctl_portal_map(fp, ctx, &input);
		if (copy_to_user(a, &input, sizeof(input)))
			return -EFAULT;
		return ret;
	}
	case USDPAA_IOCTL_PORTAL_UNMAP:
	{
		struct usdpaa_portal_map input;
		if (copy_from_user(&input, a, sizeof(input)))
			return -EFAULT;
		return ioctl_portal_unmap(ctx, &input);
	}
	case USDPAA_IOCTL_DMA_USED:
		return ioctl_dma_stats(ctx, a);
	}
	return -EINVAL;
}



static long usdpaa_ioctl_compat(struct file *fp, unsigned int cmd,
				unsigned long arg)
{
	struct ctx *ctx = fp->private_data;
	void __user *a = (void __user *)arg;
	switch (cmd) {
#ifdef CONFIG_COMPAT
	case USDPAA_IOCTL_DMA_MAP_COMPAT:
	{
		int ret;
		struct usdpaa_ioctl_dma_map_compat input;
		struct usdpaa_ioctl_dma_map converted;

		if (copy_from_user(&input, a, sizeof(input)))
			return -EFAULT;

		converted.ptr = compat_ptr(input.ptr);
		converted.phys_addr = input.phys_addr;
		converted.len = input.len;
		converted.flags = input.flags;
		strncpy(converted.name, input.name, USDPAA_DMA_NAME_MAX);
		converted.has_locking = input.has_locking;
		converted.did_create = input.did_create;

		ret = ioctl_dma_map(fp, ctx, &converted);
		input.ptr = ptr_to_compat(converted.ptr);
		input.phys_addr = converted.phys_addr;
		strncpy(input.name, converted.name, USDPAA_DMA_NAME_MAX);
		input.has_locking = converted.has_locking;
		input.did_create = converted.did_create;
		if (copy_to_user(a, &input, sizeof(input)))
			return -EFAULT;
		return ret;
	}
	case USDPAA_IOCTL_PORTAL_MAP_COMPAT:
	{
		int ret;
		struct compat_usdpaa_ioctl_portal_map input;
		struct usdpaa_ioctl_portal_map converted;
		if (copy_from_user(&input, a, sizeof(input)))
			return -EFAULT;
		converted.type = input.type;
		ret = ioctl_portal_map(fp, ctx, &converted);
		input.addr.cinh = ptr_to_compat(converted.addr.cinh);
		input.addr.cena = ptr_to_compat(converted.addr.cena);
		input.channel = converted.channel;
		input.pools = converted.pools;
		if (copy_to_user(a, &input, sizeof(input)))
			return -EFAULT;
		return ret;
	}
	case USDPAA_IOCTL_PORTAL_UNMAP_COMPAT:
	{
		struct usdpaa_portal_map_compat input;
		struct usdpaa_portal_map converted;

		if (copy_from_user(&input, a, sizeof(input)))
			return -EFAULT;
		converted.cinh = compat_ptr(input.cinh);
		converted.cena = compat_ptr(input.cena);
		return ioctl_portal_unmap(ctx, &converted);
	}
#endif
	default:
		return usdpaa_ioctl(fp, cmd, arg);
	}
	return -EINVAL;
}

struct qm_portal_config *usdpaa_get_qm_portal_config(struct file *filp,
						     void *hint)
{
	/* Walk the list of portals for filp and return the config
	   for the portal that matches the hint */

	struct ctx *context;
	struct portal_mapping *portal;

	/* First sanitize the filp */
	if (filp->f_op->open != usdpaa_open)
		return NULL;
	context = filp->private_data;
	spin_lock(&context->lock);
	list_for_each_entry(portal, &context->portals, list) {
		if (portal->user.type == usdpaa_portal_qman &&
		    portal->user.addr.cinh == hint) {
			spin_unlock(&context->lock);
			return portal->qportal;
		}
	}
	spin_unlock(&context->lock);
	return NULL;
}

struct bm_portal_config *usdpaa_get_bm_portal_config(struct file *filp,
						     void *hint)
{
	/* Walk the list of portals for filp and return the config
	   for the portal that matches the hint */

	struct ctx *context;
	struct portal_mapping *portal;

	/* First sanitize the filp */
	if (filp->f_op->open != usdpaa_open)
		return NULL;

	context = filp->private_data;

	spin_lock(&context->lock);
	list_for_each_entry(portal, &context->portals, list) {
		if (portal->user.type == usdpaa_portal_bman &&
		    portal->user.addr.cinh == hint) {
			spin_unlock(&context->lock);
			return portal->bportal;
		}
	}
	spin_unlock(&context->lock);
	return NULL;
}

static const struct file_operations usdpaa_fops = {
	.open		   = usdpaa_open,
	.release	   = usdpaa_release,
	.mmap		   = usdpaa_mmap,
	.get_unmapped_area = usdpaa_get_unmapped_area,
	.unlocked_ioctl	   = usdpaa_ioctl,
	.compat_ioctl	   = usdpaa_ioctl_compat
};

static struct miscdevice usdpaa_miscdev = {
	.name = "fsl-usdpaa",
	.fops = &usdpaa_fops,
	.minor = MISC_DYNAMIC_MINOR,
};

/* Early-boot memory allocation. The boot-arg "usdpaa_mem=<x>" is used to
 * indicate how much memory (if any) to allocate during early boot. If the
 * format "usdpaa_mem=<x>,<y>" is used, then <y> will be interpreted as the
 * number of TLB1 entries to reserve (default is 1). If there are more mappings
 * than there are TLB1 entries, fault-handling will occur. */
static __init int usdpaa_mem(char *arg)
{
	phys_size = memparse(arg, &arg);
	num_tlb = 1;
	if (*arg == ',') {
		unsigned long ul;
		int err = kstrtoul(arg + 1, 0, &ul);
		if (err < 0) {
			num_tlb = 1;
			pr_warn("ERROR, usdpaa_mem arg is invalid\n");
		} else
			num_tlb = (unsigned int)ul;
	}
	return 0;
}
early_param("usdpaa_mem", usdpaa_mem);

__init void fsl_usdpaa_init_early(void)
{
	if (!phys_size) {
		pr_info("No USDPAA memory, no 'usdpaa_mem' bootarg\n");
		return;
	}
	if (!is_good_size(phys_size)) {
		pr_err("'usdpaa_mem' bootarg must be 4096*4^x\n");
		phys_size = 0;
		return;
	}
	phys_start = memblock_alloc(phys_size, phys_size);
	if (!phys_start) {
		pr_err("Failed to reserve USDPAA region (sz:%llx)\n",
		       phys_size);
		return;
	}
	pfn_start = phys_start >> PAGE_SHIFT;
	pfn_size = phys_size >> PAGE_SHIFT;
	first_tlb = current_tlb = tlbcam_index;
	tlbcam_index += num_tlb;
	pr_info("USDPAA region at %llx:%llx(%lx:%lx), %d TLB1 entries)\n",
		phys_start, phys_size, pfn_start, pfn_size, num_tlb);
}

static int __init usdpaa_init(void)
{
	struct mem_fragment *frag;
	int ret;

	pr_info("Freescale USDPAA process driver\n");
	if (!phys_start) {
		pr_warn("fsl-usdpaa: no region found\n");
		return 0;
	}
	frag = kmalloc(sizeof(*frag), GFP_KERNEL);
	if (!frag) {
		pr_err("Failed to setup USDPAA memory accounting\n");
		return -ENOMEM;
	}
	frag->base = phys_start;
	frag->len = phys_size;
	frag->pfn_base = pfn_start;
	frag->pfn_len = pfn_size;
	frag->refs = 0;
	init_waitqueue_head(&frag->wq);
	frag->owner = NULL;
	list_add(&frag->list, &mem_list);
	ret = misc_register(&usdpaa_miscdev);
	if (ret)
		pr_err("fsl-usdpaa: failed to register misc device\n");
	return ret;
}

static void __exit usdpaa_exit(void)
{
	misc_deregister(&usdpaa_miscdev);
}

module_init(usdpaa_init);
module_exit(usdpaa_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("Freescale USDPAA process driver");
