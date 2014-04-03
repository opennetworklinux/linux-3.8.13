/*
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * Author: Minghuan Lian <Minghuan.Lian@freescale.com>
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
 */

#include <linux/err.h>
#include <linux/device.h>
#include <linux/eventfd.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/uaccess.h>
#include <linux/vfio.h>

#include "fsl_pci_ep.h"

static void fsl_pci_ep_win_set_offset(struct pci_ep_win *win)
{
	win->offset = (u64)win->type << PCI_EP_WIN_TYPE_SHIFT |
		      (u64)win->idx << PCI_EP_WIN_INDEX_SHIFT;
}

static int fsl_pci_ep_win_is_enbled(struct pci_ep_win *win)
{
	return win->attr >> 31;
}

static void fsl_pci_ep_vfio_release(void *device_data)
{
	struct pci_ep_dev *ep = device_data;

	atomic_inc(&ep->refcnt);

	module_put(THIS_MODULE);
}

static int fsl_pci_ep_vfio_open(void *device_data)
{
	struct pci_ep_dev *ep = device_data;

	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	if (!atomic_dec_and_test(&ep->refcnt)) {
		pr_err("%s: failing non-exclusive open()\n",
			dev_name(&ep->dev));
		atomic_inc(&ep->refcnt);
		module_put(THIS_MODULE);
		return -EBUSY;
	}

	return 0;
}

static long fsl_pci_ep_vfio_ioctl(void *device_data,
				  unsigned int cmd, unsigned long arg)
{
	struct pci_ep_dev *ep = device_data;

	if (cmd == VFIO_DEVICE_GET_INFO) {
		struct pci_ep_info info;

		info.iw_num = ep->iw_num;
		info.ow_num = ep->ow_num;
		info.vf_iw_num = 0;
		info.vf_ow_num = 0;
		info.type = ep->type;
		info.pf_idx = ep->pf->idx;
		info.vf_idx = ep->idx;
		return copy_to_user((void __user *)arg, &info, sizeof(info));
	} else if (cmd == VFIO_DEVICE_GET_WIN_INFO) {
		struct pci_ep_win win;

		if (copy_from_user(&win, (void __user *)arg, sizeof(win)))
			return -EFAULT;

		fsl_pci_ep_get_win(ep, &win);
		fsl_pci_ep_win_set_offset(&win);
		return copy_to_user((void __user *)arg, &win, sizeof(win));
	} else if (cmd == VFIO_DEVICE_SET_WIN_INFO) {
		struct pci_ep_win win;

		if (copy_from_user(&win, (void __user *)arg, sizeof(win)))
			return -EFAULT;

		fsl_pci_ep_set_win(ep, &win);
		fsl_pci_ep_get_win(ep, &win);
		return copy_to_user((void __user *)arg, &win, sizeof(win));
	}

	return -ENOTTY;
}

static ssize_t fsl_pci_ep_config_rw(struct pci_ep_dev *ep, char __user *buf,
				    size_t count, loff_t *ppos, bool iswrite)
{
	struct pci_dev *pdev = ep->pdev;
	size_t done = 0;
	int ret = 0;
	loff_t pos = *ppos;

	pos &= PCI_EP_OFFSET_MASK;

	while (count) {
		if (count >= 4 && !(pos % 4)) {
			u32 data;
			ret = 4;
			pci_read_config_dword(pdev, pos, &data);
			if (copy_to_user(buf, &data, ret))
				return -EFAULT;
		} else if (count >= 2 && !(pos % 2)) {
			u16 data;
			ret = 2;
			pci_read_config_word(pdev, pos, &data);
			if (copy_to_user(buf, &data, ret))
				return -EFAULT;
		} else {
			u8 data;
			ret = 1;
			pci_read_config_byte(pdev, pos, &data);
			if (copy_to_user(buf, &data, ret))
				return -EFAULT;
		}

		if (ret < 0)
			return ret;

		count -= ret;
		done += ret;
		buf += ret;
		pos += ret;
	}

	*ppos += done;

	return done;
}

static ssize_t fsl_pci_ep_regs_rw(struct pci_ep_dev *ep, char __user *buf,
				  size_t count, loff_t *ppos, bool iswrite)
{
	struct pci_pf_dev *pf = ep->pf;
	u32 data;
	loff_t pos = *ppos;

	/* only PF can r/w regs */
	if (ep->type == PCI_EP_TYPE_VF)
		return -EINVAL;

	pos &= PCI_EP_OFFSET_MASK;

	if (pos > resource_size(&pf->regs_rs))
		return -EINVAL;

	if (count != 4)
		return -EINVAL;

	if (iswrite) {
		if (copy_from_user(&data, buf, count))
			return -EFAULT;
		out_be32((u32 *)((u8 *)pf->regs + pos), data);
	} else {
		data = in_be32((u32 *)((u8 *)pf->regs + pos));
		if (copy_to_user(buf, &data, count))
			return -EFAULT;
	}

	return count;
}

static ssize_t fsl_pci_ep_win_rw(struct pci_ep_dev *ep, char __user *buf,
				 size_t count, loff_t *ppos, bool iswrite)
{
	loff_t pos = *ppos & PCI_EP_OFFSET_MASK;
	struct pci_ep_win win;
	void __iomem *mem;
	int ret;

	win.idx = PCI_EP_OFFSET_TO_INDEX(*ppos);
	win.type = PCI_EP_OFFSET_TO_TYPE(*ppos);

	if (fsl_pci_ep_get_win(ep, &win))
		return -EINVAL;

	if (!fsl_pci_ep_win_is_enbled(&win)) {
		pr_err("win%d-%d is not enabled\n", win.type, win.idx);
		return -EINVAL;
	}

	if (pos > win.size)
		return -EINVAL;

	count = min(count, (size_t)(win.size - pos));

	mem = ioremap(win.cpu_addr + pos, count);

	if (iswrite)
		ret = copy_from_user(mem, buf, count);
	else
		ret = copy_to_user(buf, mem, count);

	iounmap(mem);

	if (ret)
		return -EFAULT;

	return count;
}

static ssize_t fsl_pci_ep_rw(void *device_data, char __user *buf,
			     size_t count, loff_t *ppos, bool iswrite)
{
	int type = PCI_EP_OFFSET_TO_TYPE(*ppos);
	struct pci_ep_dev *ep = device_data;

	switch (type) {
	case PCI_EP_REGION_IBWIN:
		if (iswrite)
			return -EINVAL;
		return fsl_pci_ep_win_rw(ep, buf, count, ppos, false);

	case PCI_EP_REGION_OBWIN:
		return fsl_pci_ep_win_rw(ep, buf, count, ppos, iswrite);

	case PCI_EP_REGION_REGS:
		return fsl_pci_ep_regs_rw(ep, buf, count, ppos, iswrite);

	case PCI_EP_REGION_CONFIG:
		if (iswrite)
			return -EINVAL;
		return fsl_pci_ep_config_rw(ep, buf, count, ppos, iswrite);
	}

	return -EINVAL;
}

static ssize_t fsl_pci_ep_vfio_read(void *device_data,
				    char __user *buf,
				    size_t count, loff_t *ppos)
{
	if (!count)
		return 0;

	return fsl_pci_ep_rw(device_data, buf, count, ppos, false);
}

static ssize_t fsl_pci_ep_vfio_write(void *device_data,
				     const char __user *buf,
				     size_t count, loff_t *ppos)
{
	if (!count)
		return 0;

	return fsl_pci_ep_rw(device_data, (char __user *)buf,
			     count, ppos, true);
}

static int fsl_pci_ep_vfio_mmap(void *device_data,
				struct vm_area_struct *vma)
{
	struct pci_ep_dev *ep = device_data;
	u64 req_len, pgoff, req_start;
	struct pci_ep_win win;

	win.idx = PCI_EP_OFFSET_TO_INDEX((u64)vma->vm_pgoff << PAGE_SHIFT);
	win.type = PCI_EP_OFFSET_TO_TYPE((u64)vma->vm_pgoff << PAGE_SHIFT);

	fsl_pci_ep_get_win(ep, &win);

	if ((win.size == 0) || !fsl_pci_ep_win_is_enbled(&win)) {
		pr_err("win%d-%d is not enabled\n", win.type, win.idx);
		return -EINVAL;
	}

	if (vma->vm_end < vma->vm_start)
		return -EINVAL;

	if ((vma->vm_flags & VM_SHARED) == 0)
		return -EINVAL;

	req_len = vma->vm_end - vma->vm_start;

	pgoff = vma->vm_pgoff &
		((1U << (PCI_EP_WIN_INDEX_SHIFT - PAGE_SHIFT)) - 1);

	req_start = pgoff << PAGE_SHIFT;
	if (req_start + req_len > win.size)
		return -EINVAL;

	vma->vm_private_data = ep;
	vma->vm_flags |= VM_IO | VM_DONTEXPAND | VM_DONTDUMP;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_pgoff = (win.cpu_addr >> PAGE_SHIFT) + pgoff;

	return remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			       req_len, vma->vm_page_prot);
}

static const struct vfio_device_ops vfio_pci_ops = {
	.name		= "fsl-pci-ep-vfio",
	.open		= fsl_pci_ep_vfio_open,
	.release	= fsl_pci_ep_vfio_release,
	.ioctl		= fsl_pci_ep_vfio_ioctl,
	.read		= fsl_pci_ep_vfio_read,
	.write		= fsl_pci_ep_vfio_write,
	.mmap		= fsl_pci_ep_vfio_mmap,
};

int fsl_pci_ep_vfio_init(struct pci_ep_dev *ep)
{
	struct iommu_group *group;
	int ret;

	group = iommu_group_get(&ep->dev);
	if (!group) {
		group = iommu_group_alloc();
		if (IS_ERR(group))
			return PTR_ERR(group);
	}

	ret = iommu_group_add_device(group, &ep->dev);
	if (ret)
		pr_err("failed to add device %s to iommu group\n",
			dev_name(&ep->dev));

	iommu_group_put(group);

	ret = vfio_add_group_dev(&ep->dev, &vfio_pci_ops, ep);
	if (ret) {
		pr_err("failed to add device %s to vfio system\n",
			dev_name(&ep->dev));
		iommu_group_put(group);
		kfree(ep);
	}

	atomic_set(&ep->refcnt, 1);

	return ret;
}

void fsl_pci_ep_vfio_remove(struct pci_ep_dev *ep)
{
	if (!atomic_dec_and_test(&ep->refcnt))
		return;

	vfio_del_group_dev(&ep->dev);

	return;
}
