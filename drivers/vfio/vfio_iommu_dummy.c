/*
 * VFIO: IOMMU DMA mapping support for systems without IOMMU
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 *     Author: Varun Sethi <varun.sethi@freescale.com>
 *
 * This file is derived from driver/vfio/vfio_iommu_type1.c and
 * driver/vfio/vfio_iommu_fsl_pamu.c
 * This driver is primarily targeted for providing direct device assignment
 * on platforms that don't have a hardware IOMMU. The driver primarily
 * pins the pages corresponding to the guest memory. The driver is enabled
 * via /sys/kernel/vfio_iommu_dummy/enable_iommu_dummy attribute.
 *
 */

#include <linux/compat.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/pci.h>		/* pci_bus_type */
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/vfio.h>
#include <linux/workqueue.h>
#include <linux/hugetlb.h>

struct vfio_iommu {
	struct mutex		lock;
	struct list_head	dma_list;
	struct list_head	group_list;
};

struct vfio_dma {
	struct list_head	next;
	dma_addr_t		iova;		/* Device address */
	unsigned long		vaddr;		/* Process virtual addr */
	long			npage;		/* Number of pages */
	int			prot;		/* IOMMU_READ/WRITE */
};

struct vfio_group {
	struct iommu_group	*iommu_group;
	struct list_head	next;
};

#define NPAGE_TO_SIZE(npage)	((size_t)(npage) << PAGE_SHIFT)

struct vwork {
	struct mm_struct	*mm;
	long			npage;
	struct work_struct	work;
};

static int enable_iommu_dummy;

/* delayed decrement/increment for locked_vm */
static void vfio_lock_acct_bg(struct work_struct *work)
{
	struct vwork *vwork = container_of(work, struct vwork, work);
	struct mm_struct *mm;

	mm = vwork->mm;
	down_write(&mm->mmap_sem);
	mm->locked_vm += vwork->npage;
	up_write(&mm->mmap_sem);
	mmput(mm);
	kfree(vwork);
}

static void vfio_lock_acct(long npage)
{
	struct vwork *vwork;
	struct mm_struct *mm;

	if (!current->mm)
		return; /* process exited */

	if (down_write_trylock(&current->mm->mmap_sem)) {
		current->mm->locked_vm += npage;
		up_write(&current->mm->mmap_sem);
		return;
	}

	/*
	 * Couldn't get mmap_sem lock, so must setup to update
	 * mm->locked_vm later. If locked_vm were atomic, we
	 * wouldn't need this silliness
	 */
	vwork = kmalloc(sizeof(struct vwork), GFP_KERNEL);
	if (!vwork)
		return;
	mm = get_task_mm(current);
	if (!mm) {
		kfree(vwork);
		return;
	}
	INIT_WORK(&vwork->work, vfio_lock_acct_bg);
	vwork->mm = mm;
	vwork->npage = npage;
	schedule_work(&vwork->work);
}

/*
 * Some mappings aren't backed by a struct page, for example an mmap'd
 * MMIO range for our own or another device.  These use a different
 * pfn conversion and shouldn't be tracked as locked pages.
 */
static bool is_invalid_reserved_pfn(unsigned long pfn)
{
	if (pfn_valid(pfn)) {
		bool reserved;
		struct page *tail = pfn_to_page(pfn);
		struct page *head = compound_trans_head(tail);
		reserved = !!(PageReserved(head));
		if (head != tail) {
			/*
			 * "head" is not a dangling pointer
			 * (compound_trans_head takes care of that)
			 * but the hugepage may have been split
			 * from under us (and we may not hold a
			 * reference count on the head page so it can
			 * be reused before we run PageReferenced), so
			 * we've to check PageTail before returning
			 * what we just read.
			 */
			smp_rmb();
			if (PageTail(tail))
				return reserved;
		}
		return PageReserved(tail);
	}

	return true;
}

static int put_pfn(unsigned long pfn, int prot)
{
	if (!is_invalid_reserved_pfn(pfn)) {
		struct page *page = pfn_to_page(pfn);
		if (prot & IOMMU_WRITE)
			SetPageDirty(page);
		put_page(page);
		return 1;
	}
	return 0;
}

static int vaddr_get_pfn(unsigned long vaddr, int prot, unsigned int nr_pages)
{
	struct page **pages;
	int i, ret;
	long locked = 0;

	pages = kzalloc(sizeof(*pages) * nr_pages, GFP_KERNEL | __GFP_ZERO);
	ret = get_user_pages_fast(vaddr, nr_pages, !!(prot & IOMMU_WRITE), pages);
	if (ret != nr_pages)
		goto error;

	/* All Pages should be contiguous */
	for (i = 1; i < nr_pages; i++) {
		if (page_to_pfn(pages[i]) != page_to_pfn(pages[i - 1]) + 1)
			goto error;
		if (!is_invalid_reserved_pfn(page_to_pfn(pages[i])))
			locked++;
	}

	vfio_lock_acct(locked);

	kfree(pages);
	return 0;
error:
	for (i = 0; i < nr_pages; i++)
		if (pages[i])
			put_page(pages[i]);
	kfree(pages);
	return -EFAULT;
}

/* Unmap DMA region */
static long __vfio_dma_do_unmap(struct vfio_iommu *iommu, dma_addr_t iova,
			     long npage, int prot)
{
	long unlocked = 0;
	unsigned long size;
	unsigned long pfn;
	int i;

	/* currently we support at most one huge page mapping */
	size = npage << PAGE_SHIFT;
	/* Release the pinned pages */
	/* one is to one mapping iova = host physical*/
	pfn = iova >> PAGE_SHIFT;
	if (pfn) {
		for (i = 0; i < npage; i++, pfn++)
			unlocked += put_pfn(pfn, prot);
	}


	return unlocked;
}

static void vfio_dma_unmap(struct vfio_iommu *iommu, dma_addr_t iova,
			   long npage, int prot)
{
	long unlocked;

	unlocked = __vfio_dma_do_unmap(iommu, iova, npage, prot);
	vfio_lock_acct(-unlocked);
}

static inline bool ranges_overlap(dma_addr_t start1, size_t size1,
				  dma_addr_t start2, size_t size2)
{
	if (start1 < start2)
		return (start2 - start1 < size1);
	else if (start2 < start1)
		return (start1 - start2 < size2);
	return (size1 > 0 && size2 > 0);
}

static struct vfio_dma *vfio_find_dma(struct vfio_iommu *iommu,
				      dma_addr_t start, size_t size)
{
	struct vfio_dma *dma;

	list_for_each_entry(dma, &iommu->dma_list, next) {
		if (ranges_overlap(dma->iova, NPAGE_TO_SIZE(dma->npage),
				   start, size))
			return dma;
	}
	return NULL;
}

static long vfio_remove_dma_overlap(struct vfio_iommu *iommu, dma_addr_t start,
				    size_t size, struct vfio_dma *dma)
{
	struct vfio_dma *split;
	long npage_lo, npage_hi;

	/* Existing dma region is completely covered, unmap all */
	if (start <= dma->iova &&
	    start + size >= dma->iova + NPAGE_TO_SIZE(dma->npage)) {
		vfio_dma_unmap(iommu, dma->iova, dma->npage, dma->prot);
		list_del(&dma->next);
		npage_lo = dma->npage;
		kfree(dma);
		return npage_lo;
	}

	/* Overlap low address of existing range */
	if (start <= dma->iova) {
		size_t overlap;

		overlap = start + size - dma->iova;
		npage_lo = overlap >> PAGE_SHIFT;

		vfio_dma_unmap(iommu, dma->iova, npage_lo, dma->prot);
		dma->iova += overlap;
		dma->vaddr += overlap;
		dma->npage -= npage_lo;
		return npage_lo;
	}

	/* Overlap high address of existing range */
	if (start + size >= dma->iova + NPAGE_TO_SIZE(dma->npage)) {
		size_t overlap;

		overlap = dma->iova + NPAGE_TO_SIZE(dma->npage) - start;
		npage_hi = overlap >> PAGE_SHIFT;

		vfio_dma_unmap(iommu, start, npage_hi, dma->prot);
		dma->npage -= npage_hi;
		return npage_hi;
	}

	/* Split existing */
	npage_lo = (start - dma->iova) >> PAGE_SHIFT;
	npage_hi = dma->npage - (size >> PAGE_SHIFT) - npage_lo;

	split = kzalloc(sizeof(*split), GFP_KERNEL);
	if (!split)
		return -ENOMEM;

	vfio_dma_unmap(iommu, start, size >> PAGE_SHIFT, dma->prot);

	dma->npage = npage_lo;

	split->npage = npage_hi;
	split->iova = start + size;
	split->vaddr = dma->vaddr + NPAGE_TO_SIZE(npage_lo) + size;
	split->prot = dma->prot;
	list_add(&split->next, &iommu->dma_list);
	return size >> PAGE_SHIFT;
}

/* Map DMA region */
static int __vfio_dma_map(struct vfio_iommu *iommu, dma_addr_t iova,
			  unsigned long vaddr, long npage, int prot)
{
	int ret;

	/*
	 * XXX We break mappings into pages and use get_user_pages_fast to
	 * pin the pages in memory.  It's been suggested that mlock might
	 * provide a more efficient mechanism, but nothing prevents the
	 * user from munlocking the pages, which could then allow the user
	 * access to random host memory.  We also have no guarantee from the
	 * IOMMU API that the iommu driver can unmap sub-pages of previous
	 * mappings.  This means we might lose an entire range if a single
	 * page within it is unmapped.  Single page mappings are inefficient,
	 * but provide the most flexibility for now.
	 */
	ret = vaddr_get_pfn(vaddr, prot, npage);
	if (ret) {
		pr_err("%s unable to map vaddr = %lx\n",
			__func__, vaddr);
		__vfio_dma_do_unmap(iommu, iova, npage, prot);
		return ret;
	}

	return 0;
}

static int vfio_dma_do_map(struct vfio_iommu *iommu,
			   struct vfio_iommu_type1_dma_map *map)
{
	struct vfio_dma *dma, *pdma = NULL;
	dma_addr_t iova = map->iova;
	unsigned long locked, lock_limit, vaddr = map->vaddr;
	size_t size = map->size;
	int ret = 0, prot = 0;
	long npage;

	/* READ/WRITE from device perspective */
	if (map->flags & VFIO_DMA_MAP_FLAG_WRITE)
		prot |= IOMMU_WRITE;
	if (map->flags & VFIO_DMA_MAP_FLAG_READ)
		prot |= IOMMU_READ;

	if (!prot)
		return -EINVAL; /* No READ/WRITE? */

	/* Don't allow IOVA wrap */
	if (iova + size && iova + size < iova)
		return -EINVAL;

	/* Don't allow virtual address wrap */
	if (vaddr + size && vaddr + size < vaddr)
		return -EINVAL;

	npage = size >> PAGE_SHIFT;
	if (!npage)
		return -EINVAL;

	mutex_lock(&iommu->lock);

	if (vfio_find_dma(iommu, iova, size)) {
		ret = -EBUSY;
		goto out_lock;
	}

	/* account for locked pages */
	locked = current->mm->locked_vm + npage;
	lock_limit = rlimit(RLIMIT_MEMLOCK) >> PAGE_SHIFT;
	if (locked > lock_limit && !capable(CAP_IPC_LOCK)) {
		pr_warn("%s: RLIMIT_MEMLOCK (%ld) exceeded\n",
			__func__, rlimit(RLIMIT_MEMLOCK));
		ret = -ENOMEM;
		goto out_lock;
	}

	ret = __vfio_dma_map(iommu, iova, vaddr, npage, prot);
	if (ret)
		goto out_lock;

	/* Check if we about a region below - nothing below 0 */
	if (iova) {
		dma = vfio_find_dma(iommu, iova - 1, 1);
		if (dma && dma->prot == prot &&
		    dma->vaddr + NPAGE_TO_SIZE(dma->npage) == vaddr) {

			dma->npage += npage;
			iova = dma->iova;
			vaddr = dma->vaddr;
			npage = dma->npage;
			size = NPAGE_TO_SIZE(npage);

			pdma = dma;
		}
	}

	/* Check if we abut a region above - nothing above ~0 + 1 */
	if (iova + size) {
		dma = vfio_find_dma(iommu, iova + size, 1);
		if (dma && dma->prot == prot &&
		    dma->vaddr == vaddr + size) {

			dma->npage += npage;
			dma->iova = iova;
			dma->vaddr = vaddr;

			/*
			 * If merged above and below, remove previously
			 * merged entry.  New entry covers it.
			 */
			if (pdma) {
				list_del(&pdma->next);
				kfree(pdma);
			}
			pdma = dma;
		}
	}

	/* Isolated, new region */
	if (!pdma) {
		dma = kzalloc(sizeof(*dma), GFP_KERNEL);
		if (!dma) {
			ret = -ENOMEM;
			vfio_dma_unmap(iommu, iova, npage, prot);
			goto out_lock;
		}

		dma->npage = npage;
		dma->iova = iova;
		dma->vaddr = vaddr;
		dma->prot = prot;
		list_add(&dma->next, &iommu->dma_list);
	}

out_lock:
	mutex_unlock(&iommu->lock);
	return ret;
}

static int vfio_dma_do_unmap(struct vfio_iommu *iommu,
			     struct vfio_iommu_type1_dma_unmap *unmap)
{
	long ret = 0, npage = unmap->size >> PAGE_SHIFT;
	struct vfio_dma *dma, *tmp;

	mutex_lock(&iommu->lock);

	list_for_each_entry_safe(dma, tmp, &iommu->dma_list, next) {
		if (ranges_overlap(dma->iova, NPAGE_TO_SIZE(dma->npage),
				   unmap->iova, unmap->size)) {
			ret = vfio_remove_dma_overlap(iommu, unmap->iova,
						      unmap->size, dma);
			if (ret > 0)
				npage -= ret;
			if (ret < 0 || npage == 0)
				break;
		}
	}

	mutex_unlock(&iommu->lock);
	return ret > 0 ? 0 : (int)ret;
}

static void *vfio_iommu_dummy_open(unsigned long arg)
{
	struct vfio_iommu *iommu;

	if (arg != VFIO_IOMMU_DUMMY)
		return ERR_PTR(-EINVAL);

	iommu = kzalloc(sizeof(*iommu), GFP_KERNEL);
	if (!iommu)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&iommu->group_list);
	INIT_LIST_HEAD(&iommu->dma_list);
	mutex_init(&iommu->lock);

	return iommu;
}

static void vfio_iommu_dummy_release(void *iommu_data)
{
	struct vfio_iommu *iommu = iommu_data;
	struct vfio_group *group, *group_tmp;
	struct vfio_dma *dma, *dma_tmp;

	list_for_each_entry_safe(group, group_tmp, &iommu->group_list, next) {
		list_del(&group->next);
		kfree(group);
	}

	list_for_each_entry_safe(dma, dma_tmp, &iommu->dma_list, next) {
		vfio_dma_unmap(iommu, dma->iova, dma->npage, dma->prot);
		list_del(&dma->next);
		kfree(dma);
	}

	kfree(iommu);
}

static long vfio_iommu_dummy_ioctl(void *iommu_data,
				   unsigned int cmd, unsigned long arg)
{
	struct vfio_iommu *iommu = iommu_data;
	unsigned long minsz;

	if (cmd == VFIO_CHECK_EXTENSION) {
		switch (arg) {
		case VFIO_IOMMU_DUMMY:
			return 1;
		default:
			return 0;
		}
	} else if (cmd == VFIO_IOMMU_MAP_DMA) {
		struct vfio_iommu_type1_dma_map map;
		uint32_t mask = VFIO_DMA_MAP_FLAG_READ |
				VFIO_DMA_MAP_FLAG_WRITE;

		minsz = offsetofend(struct vfio_iommu_type1_dma_map, size);

		if (copy_from_user(&map, (void __user *)arg, minsz))
			return -EFAULT;

		if (map.argsz < minsz || map.flags & ~mask)
			return -EINVAL;

		return vfio_dma_do_map(iommu, &map);

	} else if (cmd == VFIO_IOMMU_UNMAP_DMA) {
		struct vfio_iommu_type1_dma_unmap unmap;

		minsz = offsetofend(struct vfio_iommu_type1_dma_unmap, size);

		if (copy_from_user(&unmap, (void __user *)arg, minsz))
			return -EFAULT;

		if (unmap.argsz < minsz || unmap.flags)
			return -EINVAL;

		return vfio_dma_do_unmap(iommu, &unmap);
	}

	return -ENOTTY;
}

static int vfio_iommu_dummy_attach_group(void *iommu_data,
					 struct iommu_group *iommu_group)
{
	struct vfio_iommu *iommu = iommu_data;
	struct vfio_group *group, *tmp;

	group = kzalloc(sizeof(*group), GFP_KERNEL);
	if (!group)
		return -ENOMEM;

	mutex_lock(&iommu->lock);

	list_for_each_entry(tmp, &iommu->group_list, next) {
		if (tmp->iommu_group == iommu_group) {
			mutex_unlock(&iommu->lock);
			kfree(group);
			return -EINVAL;
		}
	}

	group->iommu_group = iommu_group;
	list_add(&group->next, &iommu->group_list);

	mutex_unlock(&iommu->lock);

	return 0;
}

static void vfio_iommu_dummy_detach_group(void *iommu_data,
					  struct iommu_group *iommu_group)
{
	struct vfio_iommu *iommu = iommu_data;
	struct vfio_group *group;

	mutex_lock(&iommu->lock);

	list_for_each_entry(group, &iommu->group_list, next) {
		if (group->iommu_group == iommu_group) {
			list_del(&group->next);
			kfree(group);
			break;
		}
	}

	mutex_unlock(&iommu->lock);

}

static const struct vfio_iommu_driver_ops vfio_iommu_driver_ops_dummy = {
	.name		= "vfio-iommu-dummy",
	.owner		= THIS_MODULE,
	.open		= vfio_iommu_dummy_open,
	.release	= vfio_iommu_dummy_release,
	.ioctl		= vfio_iommu_dummy_ioctl,
	.attach_group	= vfio_iommu_dummy_attach_group,
	.detach_group	= vfio_iommu_dummy_detach_group,
};

static int iommu_dummy_add_device(struct device *dev)
{
	struct iommu_group *group;
	int ret;

	group = iommu_group_get(dev);
	if (!group) {
		group = iommu_group_alloc();
		if (IS_ERR(group))
			return PTR_ERR(group);
	}

	ret = iommu_group_add_device(group, dev);
	iommu_group_put(group);

	return ret;
}

static struct iommu_ops iommu_dummy_ops = {
	.add_device = iommu_dummy_add_device,
};

static ssize_t enable_iommu_dummy_show(struct kobject *kobj,
				      struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", enable_iommu_dummy);
}

static ssize_t enable_iommu_dummy_store(struct kobject *kobj,
				       struct kobj_attribute *attr,
				       const char *buf, size_t count)
{
	int val;

	sscanf(buf, "%du", &val);

	if (!enable_iommu_dummy && val) {
		pr_warn("Enabling vfio_iommu_dummy driver, can't be disabled\n");
		bus_set_iommu(&pci_bus_type, &iommu_dummy_ops);
		vfio_register_iommu_driver(&vfio_iommu_driver_ops_dummy);
		enable_iommu_dummy = 1;
	}

	return count;
}

static struct kobj_attribute enable_iommu_dummy_attribute =
	__ATTR(enable_iommu_dummy, 0666, enable_iommu_dummy_show, enable_iommu_dummy_store);

static struct attribute *attrs[] = {
	&enable_iommu_dummy_attribute.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct kobject *vfio_iommu_dummy_kobj;

static int __init vfio_iommu_dummy_init(void)
{
	int retval;

	vfio_iommu_dummy_kobj = kobject_create_and_add("vfio_iommu_dummy", kernel_kobj);
	if (!vfio_iommu_dummy_kobj)
		return -ENOMEM;

	/* Create the files associated with vfio_iommu_dummy kobject */
	retval = sysfs_create_group(vfio_iommu_dummy_kobj, &attr_group);
	if (retval)
		kobject_put(vfio_iommu_dummy_kobj);

	return retval;
}

static void __exit vfio_iommu_dummy_cleanup(void)
{
	if (enable_iommu_dummy)
		vfio_unregister_iommu_driver(&vfio_iommu_driver_ops_dummy);

	kobject_put(vfio_iommu_dummy_kobj);
}

module_init(vfio_iommu_dummy_init);
module_exit(vfio_iommu_dummy_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Varun Sethi <Varun.Sethi@freescale.com>");
MODULE_DESCRIPTION("Dummy IOMMU driver for VFIO");
