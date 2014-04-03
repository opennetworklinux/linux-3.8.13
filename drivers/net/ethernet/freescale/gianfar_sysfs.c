/*
 * drivers/net/ethernet/freescale/gianfar_sysfs.c
 *
 * Gianfar Ethernet Driver
 * This driver is designed for the non-CPM ethernet controllers
 * on the 85xx and 83xx family of integrated processors
 * Based on 8260_io/fcc_enet.c
 *
 * Author: Andy Fleming
 * Maintainer: Kumar Gala (galak@kernel.crashing.org)
 * Modifier: Sandeep Gopalpet <sandeep.kumar@freescale.com>
 *
 * Copyright 2002-2009, 2012 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * Sysfs file creation and management
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/device.h>

#include <asm/uaccess.h>
#include <linux/module.h>

#include "gianfar.h"

static ssize_t gfar_show_bd_stash(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct gfar_private *priv = netdev_priv(to_net_dev(dev));

	return sprintf(buf, "%s\n", priv->bd_stash_en ? "on" : "off");
}

static ssize_t gfar_set_bd_stash(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct gfar_private *priv = netdev_priv(to_net_dev(dev));
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	int new_setting = 0;
	u32 temp;
	unsigned long flags;

	if (!(priv->device_flags & FSL_GIANFAR_DEV_HAS_BD_STASHING))
		return count;


	/* Find out the new setting */
	if (!strncmp("on", buf, count - 1) || !strncmp("1", buf, count - 1))
		new_setting = 1;
	else if (!strncmp("off", buf, count - 1) ||
		 !strncmp("0", buf, count - 1))
		new_setting = 0;
	else
		return count;


	local_irq_save(flags);
	lock_rx_qs(priv);

	/* Set the new stashing value */
	priv->bd_stash_en = new_setting;

	temp = gfar_read(&regs->attr);

	if (new_setting)
		temp |= ATTR_BDSTASH;
	else
		temp &= ~(ATTR_BDSTASH);

	gfar_write(&regs->attr, temp);

	unlock_rx_qs(priv);
	local_irq_restore(flags);

	return count;
}

static DEVICE_ATTR(bd_stash, 0644, gfar_show_bd_stash, gfar_set_bd_stash);

static ssize_t gfar_show_bd_l2sram(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct gfar_private *priv = netdev_priv(to_net_dev(dev));

	return sprintf(buf, "%s\n", priv->bd_l2sram_en ? "on" : "off");
}

static ssize_t gfar_set_bd_l2sram(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct net_device *ndev = to_net_dev(dev);
	struct gfar_private *priv = netdev_priv(ndev);
	int new_setting = 0;

	if (!gfar_l2sram_en)
		return count;

	/* Find out the new setting */
	if (!strncmp("on", buf, count - 1) || !strncmp("1", buf, count - 1))
		new_setting = 1;
	else if (!strncmp("off", buf, count - 1) ||
		 !strncmp("0", buf, count - 1))
		new_setting = 0;
	else
		return count;

	if (new_setting == priv->bd_l2sram_en)
		/* nothing to do */
		return count;

	if (ndev->flags & IFF_UP)
		stop_gfar(ndev);

	priv->bd_l2sram_en = new_setting;

	if (ndev->flags & IFF_UP)
		startup_gfar(ndev);

	return count;
}

static DEVICE_ATTR(bd_l2sram, 0644, gfar_show_bd_l2sram, gfar_set_bd_l2sram);

static ssize_t gfar_show_rx_stash_size(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct gfar_private *priv = netdev_priv(to_net_dev(dev));

	return sprintf(buf, "%d\n", priv->rx_stash_size);
}

static ssize_t gfar_set_rx_stash_size(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct gfar_private *priv = netdev_priv(to_net_dev(dev));
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	unsigned int length = simple_strtoul(buf, NULL, 0);
	u32 temp;
	unsigned long flags;

	if (!(priv->device_flags & FSL_GIANFAR_DEV_HAS_BUF_STASHING))
		return count;

	local_irq_save(flags);
	lock_rx_qs(priv);

	if (length > priv->rx_buffer_size)
		goto out;

	if (length == priv->rx_stash_size)
		goto out;

	priv->rx_stash_size = length;

	temp = gfar_read(&regs->attreli);
	temp &= ~ATTRELI_EL_MASK;
	temp |= ATTRELI_EL(length);
	gfar_write(&regs->attreli, temp);

	/* Turn stashing on/off as appropriate */
	temp = gfar_read(&regs->attr);

	if (length)
		temp |= ATTR_BUFSTASH;
	else
		temp &= ~(ATTR_BUFSTASH);

	gfar_write(&regs->attr, temp);

out:
	unlock_rx_qs(priv);
	local_irq_restore(flags);

	return count;
}

static DEVICE_ATTR(rx_stash_size, 0644, gfar_show_rx_stash_size,
		   gfar_set_rx_stash_size);

/* Stashing will only be enabled when rx_stash_size != 0 */
static ssize_t gfar_show_rx_stash_index(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct gfar_private *priv = netdev_priv(to_net_dev(dev));

	return sprintf(buf, "%d\n", priv->rx_stash_index);
}

static ssize_t gfar_set_rx_stash_index(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct gfar_private *priv = netdev_priv(to_net_dev(dev));
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	unsigned short index = simple_strtoul(buf, NULL, 0);
	u32 temp;
	unsigned long flags;

	if (!(priv->device_flags & FSL_GIANFAR_DEV_HAS_BUF_STASHING))
		return count;

	local_irq_save(flags);
	lock_rx_qs(priv);

	if (index > priv->rx_stash_size)
		goto out;

	if (index == priv->rx_stash_index)
		goto out;

	priv->rx_stash_index = index;

	temp = gfar_read(&regs->attreli);
	temp &= ~ATTRELI_EI_MASK;
	temp |= ATTRELI_EI(index);
	gfar_write(&regs->attreli, temp);

out:
	unlock_rx_qs(priv);
	local_irq_restore(flags);

	return count;
}

static DEVICE_ATTR(rx_stash_index, 0644, gfar_show_rx_stash_index,
		   gfar_set_rx_stash_index);

static ssize_t gfar_show_fifo_threshold(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct gfar_private *priv = netdev_priv(to_net_dev(dev));

	return sprintf(buf, "%d\n", priv->fifo_threshold);
}

static ssize_t gfar_set_fifo_threshold(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct gfar_private *priv = netdev_priv(to_net_dev(dev));
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	unsigned int length = simple_strtoul(buf, NULL, 0);
	u32 temp;
	unsigned long flags;

	if (length > GFAR_MAX_FIFO_THRESHOLD)
		return count;

	local_irq_save(flags);
	lock_tx_qs(priv);

	priv->fifo_threshold = length;

	temp = gfar_read(&regs->fifo_tx_thr);
	temp &= ~FIFO_TX_THR_MASK;
	temp |= length;
	gfar_write(&regs->fifo_tx_thr, temp);

	unlock_tx_qs(priv);
	local_irq_restore(flags);

	return count;
}

static DEVICE_ATTR(fifo_threshold, 0644, gfar_show_fifo_threshold,
		   gfar_set_fifo_threshold);

static ssize_t gfar_show_fifo_starve(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct gfar_private *priv = netdev_priv(to_net_dev(dev));

	return sprintf(buf, "%d\n", priv->fifo_starve);
}

static ssize_t gfar_set_fifo_starve(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct gfar_private *priv = netdev_priv(to_net_dev(dev));
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	unsigned int num = simple_strtoul(buf, NULL, 0);
	u32 temp;
	unsigned long flags;

	if (num > GFAR_MAX_FIFO_STARVE)
		return count;

	local_irq_save(flags);
	lock_tx_qs(priv);

	priv->fifo_starve = num;

	temp = gfar_read(&regs->fifo_tx_starve);
	temp &= ~FIFO_TX_STARVE_MASK;
	temp |= num;
	gfar_write(&regs->fifo_tx_starve, temp);

	unlock_tx_qs(priv);
	local_irq_restore(flags);

	return count;
}

static DEVICE_ATTR(fifo_starve, 0644, gfar_show_fifo_starve,
		   gfar_set_fifo_starve);

static ssize_t gfar_show_fifo_starve_off(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct gfar_private *priv = netdev_priv(to_net_dev(dev));

	return sprintf(buf, "%d\n", priv->fifo_starve_off);
}

static ssize_t gfar_set_fifo_starve_off(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct gfar_private *priv = netdev_priv(to_net_dev(dev));
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	unsigned int num = simple_strtoul(buf, NULL, 0);
	u32 temp;
	unsigned long flags;

	if (num > GFAR_MAX_FIFO_STARVE_OFF)
		return count;

	local_irq_save(flags);
	lock_tx_qs(priv);

	priv->fifo_starve_off = num;

	temp = gfar_read(&regs->fifo_tx_starve_shutoff);
	temp &= ~FIFO_TX_STARVE_OFF_MASK;
	temp |= num;
	gfar_write(&regs->fifo_tx_starve_shutoff, temp);

	unlock_tx_qs(priv);
	local_irq_restore(flags);

	return count;
}

static DEVICE_ATTR(fifo_starve_off, 0644, gfar_show_fifo_starve_off,
		   gfar_set_fifo_starve_off);

static ssize_t gfar_show_recycle(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct gfar_private *priv = netdev_priv(to_net_dev(dev));
	struct gfar_priv_recycle *rec = &priv->recycle;
	int cpu;

	for_each_possible_cpu(cpu) {
		struct gfar_priv_recycle_local *local;

		local = per_cpu_ptr(rec->local, cpu);
		pr_info("local: CPU#%d: recycled skbs %d, reused skbs %d\n",
			cpu, local->recycle_cnt, local->reuse_cnt);
	}

	pr_info("shared: recycled skbs %d, reused skbs %d\n",
		atomic_read(&rec->recycle_cnt),
		atomic_read(&rec->reuse_cnt));

	return sprintf(buf, "%s\n", priv->ndev->name);
}

static DEVICE_ATTR(recycle, 0444, gfar_show_recycle, NULL);

static ssize_t gfar_show_recycle_target(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct gfar_private *priv = netdev_priv(to_net_dev(dev));
	char *name = gfar_skb_recycling_en ? priv->recycle_ndev->name : "";

	return sprintf(buf, "%s\n", name);
}

static ssize_t gfar_set_recycle_target(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct net_device *ndev = to_net_dev(dev);
	struct gfar_private *priv = netdev_priv(ndev);
	struct gfar_private *priv_target;
	int found = 0;

	if (!gfar_skb_recycling_en)
		return count;

	list_for_each_entry(priv_target, &gfar_recycle_queues, recycle_node) {
		char *name = priv_target->ndev->name;
		if ((strlen(name) == count - 1) &&
			!strncmp(buf, name, count - 1)) {
			found = 1;
			break;
		}
	}

	if (!found) {
		pr_err("Invalid skb recycle target device!\n");
		return count;
	}

	if (priv->recycle_target == &priv_target->recycle)
		/* nothing to do */
		return count;

	if (ndev->flags & IFF_UP)
		stop_gfar(ndev);

	priv->recycle_target = &priv_target->recycle;
	priv->recycle_ndev = priv_target->ndev;

	if (ndev->flags & IFF_UP)
		startup_gfar(ndev);

	return count;
}

static DEVICE_ATTR(recycle_target, 0644, gfar_show_recycle_target,
		   gfar_set_recycle_target);


static ssize_t gfar_show_ptp_1588(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gfar_private *priv = netdev_priv(to_net_dev(dev));

	if (priv->device_flags & FSL_GIANFAR_DEV_HAS_TIMER)
		return sprintf(buf, "1\n");
	else
		return sprintf(buf, "0\n");
}

static ssize_t gfar_set_ptp_1588(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct net_device *ndev = to_net_dev(dev);
	struct gfar_private *priv = netdev_priv(ndev);
	unsigned int num;
	unsigned long flags;

	if (kstrtouint(buf, 0, &num) < 0)
		return -EINVAL;

	local_irq_save(flags);
	lock_tx_qs(priv);
	lock_rx_qs(priv);

	if (num)
		gfar_1588_start(priv);
	else
		gfar_1588_stop(priv);

	unlock_rx_qs(priv);
	unlock_tx_qs(priv);
	local_irq_restore(flags);

	return count;
}

static DEVICE_ATTR(ptp_1588, 0644, gfar_show_ptp_1588, gfar_set_ptp_1588);

void gfar_init_sysfs(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	int rc;

	/* Initialize the default values */
	priv->fifo_threshold = DEFAULT_FIFO_TX_THR;
	priv->fifo_starve = DEFAULT_FIFO_TX_STARVE;
	priv->fifo_starve_off = DEFAULT_FIFO_TX_STARVE_OFF;

	/* Create our sysfs files */
	rc = device_create_file(&dev->dev, &dev_attr_bd_stash);
	rc |= device_create_file(&dev->dev, &dev_attr_bd_l2sram);
	rc |= device_create_file(&dev->dev, &dev_attr_rx_stash_size);
	rc |= device_create_file(&dev->dev, &dev_attr_rx_stash_index);
	rc |= device_create_file(&dev->dev, &dev_attr_fifo_threshold);
	rc |= device_create_file(&dev->dev, &dev_attr_fifo_starve);
	rc |= device_create_file(&dev->dev, &dev_attr_fifo_starve_off);
	rc |= device_create_file(&dev->dev, &dev_attr_recycle);
	rc |= device_create_file(&dev->dev, &dev_attr_recycle_target);
	rc |= device_create_file(&dev->dev, &dev_attr_ptp_1588);
	if (rc)
		dev_err(&dev->dev, "Error creating gianfar sysfs files.\n");
}
