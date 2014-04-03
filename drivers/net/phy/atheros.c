/*
 * drivers/net/phy/atheros.c
 *
 * Driver for Atheros PHYs
 *
 * Author: Michael Johnston <michael.johnston@freescale.com>
 *	   Chunhe Lan <Chunhe.Lan@freescale.com>
 *
 * Copyright (c) 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <linux/phy.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>

#define AR8035_PHYSR		0x0011
#define AR8035_PHYSR_DUPLEX	0x2000
#define AR8035_PHYSR_SPEED	0xc000
#define AR8035_INER		0x0012
#define AR8035_INER_INIT	0xec00
#define AR8035_INSR		0x0013

#define PHY_ID_AR8035		0x004dd072

MODULE_DESCRIPTION("Atheros PHY driver");
MODULE_AUTHOR("Michael Johnston");
MODULE_LICENSE("GPL");

static int ar8035_ack_interrupt(struct phy_device *phydev)
{
	int err;

	err = phy_read(phydev, AR8035_INSR);

	return (err < 0) ? err : 0;
}

static int ar8035_config_intr(struct phy_device *phydev)
{
	int err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		err = phy_write(phydev, AR8035_INER, AR8035_INER_INIT);
	else
		err = phy_write(phydev, AR8035_INER, 0);

	return err;
}

/* AR8035 */
static struct phy_driver ar8035_driver = {
	.phy_id		= PHY_ID_AR8035,
	.name		= "AR8035 Gigabit Ethernet",
	.phy_id_mask	= 0x007fffff,
	.features	= PHY_GBIT_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.config_aneg	= &genphy_config_aneg,
	.read_status	= &genphy_read_status,
	.ack_interrupt	= &ar8035_ack_interrupt,
	.config_intr	= &ar8035_config_intr,
	.driver		= {
		.owner = THIS_MODULE,
	},
};

static int __init atheros_init(void)
{
	int ret;

	ret = phy_driver_register(&ar8035_driver);
	if (ret < 0)
		pr_warn("AR8035: phy_driver_register is error\n");

	return ret;
}

static void __exit atheros_exit(void)
{
	phy_driver_unregister(&ar8035_driver);
}

module_init(atheros_init);
module_exit(atheros_exit);

static struct mdio_device_id __maybe_unused atheros_tbl[] = {
	{ PHY_ID_AR8035, 0x007fffff },
	{ }
};

MODULE_DEVICE_TABLE(mdio, atheros_tbl);
