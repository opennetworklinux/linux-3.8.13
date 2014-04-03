/*
 *
 * Copyright 2011-2012 Freescale Semiconductor, Inc.
 *
 * TDM Test Module.
 * This TDM test module is a small test module which registers with the
 * TDM framework and transfer and receive data via UCC1.
 *
 * Author: Kai Jiang <Kai.Jiang@freescale.com>
 * Modifier: Jiucheng Xu <Jiucheng.Xu@freescale.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/param.h>
#include <linux/tdm.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/tdm.h>

#define READ_LEN (72 * 32)	/* BD buffer max len */
#define WRITE_LEN 0x80
#define READ_COUNT 4		/* BD ring len */

static struct task_struct *tdm_thread_task;
static struct tdm_driver test_tdmdev_driver;
static int ucc_num = 1;
static int forward;
static u8 *buf;

module_param(ucc_num, int, S_IRUGO);
module_param(forward, int, S_IRUGO);

static int tdm_check_data(void)
{
	int i, j;
	unsigned long stamp;

	stamp = jiffies + HZ;	/* 1 sec for overtime */
	while (time_before(jiffies, stamp)) {
		memset(buf, 0xff, READ_LEN);
		tdm_read_direct(test_tdmdev_driver.adapter, buf, READ_LEN);

		for (i = 0; (i < READ_LEN); i++) {
			if (buf[i] != 0xff) {
				for (j = 0; j < WRITE_LEN; j += 4)
					pr_info("%x\t%x\t%x\t%x\n",
						buf[i+j], buf[i+j+1],
						buf[i+j+2], buf[i+j+3]);
				return 0;
			}
		}
	}

	pr_warn("ucc_tdm_test: check data overtime\n");
	return 0;
}

static int tdm_thread(void *ptr)
{
	int i;

	tdm_adap_enable(&test_tdmdev_driver);

	if (forward == 1) {
		/* data forwarding application for zarlink */
		while (!kthread_should_stop()) {
			memset(buf, 0, READ_LEN);
			tdm_read_direct(test_tdmdev_driver.adapter, buf,
					READ_LEN);
			tdm_write_direct(test_tdmdev_driver.adapter, buf,
					READ_LEN);
		}
	} else {
		/* transmit-recieve test with loopback cable */
		for (i = 0; i < WRITE_LEN; i++)
			buf[i] = i;	/* generate test data */

		tdm_write_direct(test_tdmdev_driver.adapter, buf,
				WRITE_LEN);

		tdm_check_data();
	}

	tdm_adap_disable(&test_tdmdev_driver);

	return 0;
}

static int test_attach_adapter(struct tdm_adapter *adap)
{
	buf = kmalloc(READ_LEN, GFP_KERNEL);
	if (!buf) {
		dev_err(adap->parent, "No memory for TDM\n");
		return -ENOMEM;
	}

	tdm_thread_task = kthread_run(tdm_thread, NULL, "tdm_thread");

	return 0;
}

static int test_detach_adapter(struct tdm_adapter *adap)
{
	if (forward == 1)
		kthread_stop(tdm_thread_task);

	kfree(buf);

	return 0;
}

static const struct tdm_device_id test_ucc_tdm_id[] = {
	{ "tdm_ucc_1", 0 },
	{ }
};

static struct tdm_driver test_tdmdev_driver = {
	.attach_adapter	= test_attach_adapter,
	.detach_adapter	= test_detach_adapter,
	.id_table	= test_ucc_tdm_id,
};

static int __init tdm_test_init(void)
{
	if ((ucc_num < 0) || (ucc_num > 8))
		pr_warn("ucc_tdm_test: invalid ucc_num %d(1 ~7), "
			"use the defaut ucc_num = 1\n", ucc_num);
	else
		sprintf((char *)test_ucc_tdm_id[0].name, "%s%d",
			"tdm_ucc_", ucc_num);

	test_tdmdev_driver.id = 1;

	/* create a binding with TDM driver */
	return tdm_add_driver(&test_tdmdev_driver);
}

static void __exit tdm_test_exit(void)
{
	tdm_unregister_driver(&test_tdmdev_driver);
}

module_init(tdm_test_init);
module_exit(tdm_test_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(" Kai Jiang <Kai.Jiang@freescale.com>");
MODULE_DESCRIPTION("Test Module for Freescale Platforms with TDM support");
