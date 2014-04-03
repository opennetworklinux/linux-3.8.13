/*
 * Copyright 2009-2012 Freescale Semiconductor, Inc.
 *
 * Driver for National Semiconductor PHYs 8384x
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/netdevice.h>

/* DP8384x phy identifier values */
#define DP83848_PHY_ID	0x20005c90
#define DP83849_PHY_ID  0x20005ca0
/* PHY Status Register */
#define MII_DP8384X_PHYSTST            16

static int ns8384x_config_init(struct phy_device *phydev)
{
	int ret = phy_read(phydev, MII_DP8384X_PHYSTST);
	if (ret < 0) {
		printk(KERN_INFO "%s MII_DP83640_ISR %x\n",
			__func__, ret);
	}

	return 0;
}

static struct phy_driver dp83848_driver = {
	.phy_id = DP83848_PHY_ID,
	.phy_id_mask = 0xfffffff0,
	.name = "NatSemi DP83848",
	.features = PHY_BASIC_FEATURES,
	.flags = PHY_HAS_INTERRUPT,
	.config_init = ns8384x_config_init,
	.config_aneg = genphy_config_aneg,
	.read_status = genphy_read_status,
	.driver = {.owner = THIS_MODULE,}
};

static struct phy_driver dp83849_driver = {
	.phy_id = DP83849_PHY_ID,
	.phy_id_mask = 0xfffffff0,
	.name = "NatSemi DP83849",
	.features = PHY_BASIC_FEATURES,
	.flags = PHY_HAS_INTERRUPT,
	.config_init = ns8384x_config_init,
	.config_aneg = genphy_config_aneg,
	.read_status = genphy_read_status,
	.driver = {.owner = THIS_MODULE,}
};

static int __init ns8384x_init(void)
{
	int ret;

	ret = phy_driver_register(&dp83848_driver);
	if (ret)
		goto err1;

	ret = phy_driver_register(&dp83849_driver);
	if (ret)
		goto err2;

	return 0;
err2:
	printk(KERN_INFO "register dp83849 PHY driver fail\n");
	phy_driver_unregister(&dp83848_driver);
err1:
	printk(KERN_INFO "register dp83848 PHY driver fail\n");
	return ret;
}

static void __exit ns8384x_exit(void)
{
	phy_driver_unregister(&dp83848_driver);
	phy_driver_unregister(&dp83849_driver);
}

MODULE_DESCRIPTION("NatSemi PHY driver");
MODULE_AUTHOR("Chenghu Wu <b16972@freescale.com>");
MODULE_LICENSE("GPL v2");

module_init(ns8384x_init);
module_exit(ns8384x_exit);
