/* Copyright (c) 2013 Freescale Semiconductor, Inc.
 * All rights reserved.
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

/* define a device that allows USPDAA processes to open a file
   decriptor and specify which IRQ it wants to montior using an ioctl()
   When an IRQ is received, the device becomes readable so that a process
   can use read() or select() type calls to monitor for IRQs */

#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/fsl_usdpaa.h>
#include <linux/module.h>
#include <linux/fdtable.h>
#include <linux/file.h>

#include "qman_low.h"
#include "bman_low.h"

struct usdpaa_irq_ctx {
	int irq_set; /* Set to true once the irq is set via ioctl */
	unsigned int irq_num;
	u32 last_irq_count; /* Last value returned from read */
	u32 irq_count; /* Number of irqs since last read */
	wait_queue_head_t wait_queue; /* Waiting processes */
	spinlock_t lock;
	void *inhibit_addr; /* inhibit register address */
	struct file *usdpaa_filp;
};


static int usdpaa_irq_open(struct inode *inode, struct file *filp)
{
	struct usdpaa_irq_ctx *ctx = kmalloc(sizeof(struct usdpaa_irq_ctx),
					     GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->irq_set = 0;
	ctx->irq_count = 0;
	ctx->last_irq_count = 0;
	init_waitqueue_head(&ctx->wait_queue);
	spin_lock_init(&ctx->lock);
	filp->private_data = ctx;
	return 0;
}

static int usdpaa_irq_release(struct inode *inode, struct file *filp)
{
	struct usdpaa_irq_ctx *ctx = filp->private_data;
	if (ctx->irq_set) {
		/* Inhibit the IRQ */
		out_be32(ctx->inhibit_addr, 0x1);
		free_irq(ctx->irq_num, ctx);
		ctx->irq_set = 0;
		fput(ctx->usdpaa_filp);
	}
	kfree(filp->private_data);
	return 0;
}


irqreturn_t usdpaa_irq_handler(int irq, void *_ctx)
{
	unsigned long flags;
	struct usdpaa_irq_ctx *ctx = _ctx;
	spin_lock_irqsave(&ctx->lock, flags);
	++ctx->irq_count;
	wake_up_all(&ctx->wait_queue);
	spin_unlock_irqrestore(&ctx->lock, flags);
	/* Set the inhibit register.  This will be reenabled
	   once the USDPAA code handles the IRQ */
	out_be32(ctx->inhibit_addr, 0x1);
	return IRQ_HANDLED;
}



static int map_irq(struct file *fp, struct usdpaa_ioctl_irq_map *irq_map)
{
	struct usdpaa_irq_ctx *ctx = fp->private_data;
	struct qm_portal_config *qportal = NULL;
	struct bm_portal_config *bportal = NULL;
	int irq, ret;
	void *inhibit_reg;
	struct file *old_filp = ctx->usdpaa_filp;

	ctx->usdpaa_filp = fget(irq_map->fd);
	if (!ctx->usdpaa_filp) {
		pr_err("fget() returned NULL for fd %d\n", irq_map->fd);
		return -EINVAL;
	}

	if (irq_map->type == usdpaa_portal_qman) {
		qportal = usdpaa_get_qm_portal_config(ctx->usdpaa_filp,
						   irq_map->portal_cinh);
		if (!qportal) {
			pr_err("Couldn't associate info to QMan Portal\n");
			fput(ctx->usdpaa_filp);
			return -EINVAL;
		}
		/* Lookup IRQ number for portal */
		irq = qportal->public_cfg.irq;
		inhibit_reg = qportal->addr_virt[1] + QM_REG_IIR;
	} else {
		bportal = usdpaa_get_bm_portal_config(ctx->usdpaa_filp,
					      irq_map->portal_cinh);
		if (!bportal) {
			pr_err("Couldn't associate info to BMan Portal\n");
			fput(ctx->usdpaa_filp);
			return -EINVAL;
		}
		/* Lookup IRQ number for portal */
		irq = bportal->public_cfg.irq;
		inhibit_reg = bportal->addr_virt[1] + BM_REG_IIR;
	}
	if (ctx->irq_set) {
		fput(old_filp);
		free_irq(ctx->irq_num, ctx);
	}

	ctx->irq_set = 1;
	ctx->irq_num = irq;
	ctx->inhibit_addr = inhibit_reg;

	ret = request_irq(irq, usdpaa_irq_handler, 0,
			  "usdpaa_irq", ctx);
	if (ret) {
		pr_err("request_irq for irq %d failed, ret= %d\n", irq, ret);
		ctx->irq_set = 0;
		fput(ctx->usdpaa_filp);
		return ret;
	}
	return 0;
};

static long usdpaa_irq_ioctl(struct file *fp, unsigned int cmd,
			     unsigned long arg)
{
	int ret;
	struct usdpaa_ioctl_irq_map irq_map;

	if (cmd != USDPAA_IOCTL_PORTAL_IRQ_MAP) {
		pr_err("Unknown command 0x%x\n", cmd);
		return -EINVAL;
	}

	ret = copy_from_user(&irq_map, (void __user *)arg,
			     sizeof(irq_map));
	if (ret)
		return ret;
	return map_irq(fp, &irq_map);
};

static ssize_t usdpaa_irq_read(struct file *filp, char __user *buff,
			       size_t count, loff_t *offp)
{
	struct usdpaa_irq_ctx *ctx = filp->private_data;
	int ret;

	if (!ctx->irq_set) {
		pr_err("Reading USDPAA IRQ before it was set\n");
		return -EINVAL;
	}

	if (count < sizeof(ctx->irq_count)) {
		pr_err("USDPAA IRQ Read too small\n");
		return -EINVAL;
	}
	if (ctx->irq_count == ctx->last_irq_count) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(ctx->wait_queue,
				       ctx->irq_count != ctx->last_irq_count);
		if (ret == -ERESTARTSYS)
			return ret;
	}

	ctx->last_irq_count = ctx->irq_count;

	if (copy_to_user(buff, &ctx->last_irq_count,
			 sizeof(ctx->last_irq_count)))
		return -EFAULT;
	return sizeof(ctx->irq_count);

}

static unsigned int usdpaa_irq_poll(struct file *filp, poll_table *wait)
{
	struct usdpaa_irq_ctx *ctx = filp->private_data;
	unsigned int ret = 0;
	unsigned long flags;

	if (!ctx->irq_set)
		return POLLHUP;

	poll_wait(filp, &ctx->wait_queue, wait);

	spin_lock_irqsave(&ctx->lock, flags);
	if (ctx->irq_count != ctx->last_irq_count)
		ret |= POLLIN | POLLRDNORM;
	spin_unlock_irqrestore(&ctx->lock, flags);
	return ret;
}

static long usdpaa_irq_ioctl_compat(struct file *fp, unsigned int cmd,
				unsigned long arg)
{
	void __user *a = (void __user *)arg;
	switch (cmd) {
#ifdef CONFIG_COMPAT
	case  USDPAA_IOCTL_PORTAL_IRQ_MAP_COMPAT:
	{
		struct compat_ioctl_irq_map input;
		struct usdpaa_ioctl_irq_map converted;
		if (copy_from_user(&input, a, sizeof(input)))
			return -EFAULT;
		converted.type = input.type;
		converted.fd = input.fd;
		converted.portal_cinh = compat_ptr(input.portal_cinh);
		return map_irq(fp, &converted);
	}
#endif
	default:
		return usdpaa_irq_ioctl(fp, cmd, arg);
	}
};

static const struct file_operations usdpaa_irq_fops = {
	.open		   = usdpaa_irq_open,
	.release	   = usdpaa_irq_release,
	.unlocked_ioctl	   = usdpaa_irq_ioctl,
	.compat_ioctl	   = usdpaa_irq_ioctl_compat,
	.read              = usdpaa_irq_read,
	.poll              = usdpaa_irq_poll
};

static struct miscdevice usdpaa_miscdev = {
	.name = "fsl-usdpaa-irq",
	.fops = &usdpaa_irq_fops,
	.minor = MISC_DYNAMIC_MINOR,
};

static int __init usdpaa_irq_init(void)
{
	int ret;

	pr_info("Freescale USDPAA process IRQ driver\n");
	ret = misc_register(&usdpaa_miscdev);
	if (ret)
		pr_err("fsl-usdpaa-irq: failed to register misc device\n");
	return ret;
}

static void __exit usdpaa_irq_exit(void)
{
	misc_deregister(&usdpaa_miscdev);
}

module_init(usdpaa_irq_init);
module_exit(usdpaa_irq_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("Freescale USDPAA process IRQ driver");
