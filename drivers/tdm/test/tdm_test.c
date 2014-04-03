/*
 * drivers/tdm/test/tdm_test.c
 *
 * Copyright (C) 2008-2012 Freescale Semiconductor, Inc. All rights reserved.
 *
 * TDM Test Module.
 * This TDM test module is a small test module which registers with the
 * TDM framework and sets up a TDM Voice path between Port 0
 * and Port 1 for each slic.
 *
 * Author:Hemant Agrawal <hemant@freescale.com>
 *        Rajesh Gumasta <rajesh.gumasta@freescale.com>
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
#include "../device/tdm_fsl.h"

#define DRV_DESC "Test Module for Freescale Platforms with TDM support"
#define DRV_NAME "tdm_test"

#define TDM_FRAME_LENGTH NUM_SAMPLES_PER_FRAME
#define TDM_E_OK 0
#define FRAME_SIZE 0x500
#define POLL_COUNT 5000

static struct task_struct *tdm_thread_task;

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hemant Agrawal <hemant@freescale.com> and "
	"Rajesh Gumasta <rajesh.gumasta@freescale.com>");
MODULE_DESCRIPTION(DRV_DESC);

static struct tdm_driver test_tdmdev_driver;
struct tdm_port *tdmport;
int tdm_thread_state;

static int tdm_thread(void *ptr)
{
	int ret = TDM_E_OK;
	int poll = 0, poll_count = POLL_COUNT;
	void *h_port;
	void *h_channel1, *h_channel2, *h_channel3, *h_channel4;
	unsigned short p_data1[TDM_FRAME_LENGTH];
	unsigned short p_data2[TDM_FRAME_LENGTH];
	uint16_t size = TDM_FRAME_LENGTH;
	u16 ch1_id = 0, ch2_id = 1, ch3_id = 2, ch4_id = 3;
	tdm_thread_state = 1;

	/* Open port */
	ret = tdm_port_open(&test_tdmdev_driver, &h_port);
	pr_debug("%s tdm_port_open ret = %d\n", __func__, ret);
	if ((ret != TDM_E_OK) || (h_port == NULL)) {
		pr_err("Error in tdm_port_open- ret %x\n", ret);
		goto port1_failed;
	}
	/* Open Channel 1*/
	ret = tdm_channel_open(ch1_id, 1, h_port, &h_channel1);
	if ((ret != TDM_E_OK) || (h_channel1 == NULL)) {
		pr_err("Error in tdm_channel_open(%d)- ret %x\n", ch1_id, ret);
		goto ch1_failed;
	}

	/* Open Channel 2*/
	ret = tdm_channel_open(ch2_id, 1, h_port, &h_channel2);
	if ((ret != TDM_E_OK) || (h_channel2 == NULL)) {
		pr_err("Error in tdm_channel_open(%d)- ret %x\n", ch2_id, ret);
		goto ch2_failed;
	}

	/* Open Channel 3*/
	ret = tdm_channel_open(ch3_id, 1, h_port, &h_channel3);
	if ((ret != TDM_E_OK) || (h_channel3 == NULL)) {
		pr_err("Error in tdm_channel_open(%d)- ret %x\n", ch3_id, ret);
		goto ch3_failed;
	}

	/* Open Channel 4*/
	ret = tdm_channel_open(ch4_id, 1, h_port, &h_channel4);
	if ((ret != TDM_E_OK) || (h_channel4 == NULL)) {
		pr_err("Error in tdm_channel_open(%d)- ret %x\n", ch4_id, ret);
		goto ch4_failed;
	}

	while ((poll < poll_count) && !kthread_should_stop()) {

		poll++;
		while (tdm_ch_poll(h_channel1, 10) != TDM_E_OK)
				continue;

		ret = tdm_channel_read(h_port, h_channel1, p_data1, &size);
		if (ret != TDM_E_OK)
			pr_info("Error in tdm_channel_read\n");

		ret = tdm_channel_write(h_port, h_channel2, p_data1, size);
		if (ret != TDM_E_OK)
			pr_info("Error in tdm_channel_write\n");

		ret = tdm_channel_read(h_port, h_channel2, p_data1, &size);
		if (ret != TDM_E_OK)
			pr_info("Error in tdm_channel_read\n");

		ret = tdm_channel_write(h_port, h_channel1, p_data1, size);
		if (ret != TDM_E_OK)
			pr_info("Error in tdm_channel_write\n");

		ret = tdm_channel_read(h_port, h_channel3, p_data2, &size);
		if (ret != TDM_E_OK)
			pr_info("Error in tdm_channel_read\n");

		ret = tdm_channel_write(h_port, h_channel4, p_data2, size);
		if (ret != TDM_E_OK)
			pr_info("Error in tdm_channel_write\n");

		ret = tdm_channel_read(h_port, h_channel4, p_data2, &size);
		if (ret != TDM_E_OK)
			pr_info("Error in tdm_channel_read\n");

		ret = tdm_channel_write(h_port, h_channel3, p_data2, size);
		if (ret != TDM_E_OK)
			pr_info("Error in tdm_channel_write\n");
	}
	pr_info("\n CLOSING THE TDM TEST\n");
	ret = tdm_channel_close(ch4_id, 1, h_port, h_channel4);
	if (ret != TDM_E_OK) {
		pr_err("Error in tdm_channel_close(%d)- ret %x\n", ch4_id, ret);
		ret = -ENXIO;
	}

	ret = tdm_channel_close(ch3_id, 1, h_port, h_channel3);
	if (ret != TDM_E_OK) {
		pr_err("Error in tdm_channel_close(%d)- ret %x\n", ch3_id, ret);
		ret = -ENXIO;
	}

	ret = tdm_channel_close(ch2_id, 1, h_port, h_channel2);
	if (ret != TDM_E_OK) {
		pr_err("Error in tdm_channel_close(%d)- ret %x\n", ch2_id, ret);
		ret = -ENXIO;
	}

	ret = tdm_channel_close(ch1_id, 1, h_port, h_channel1);
	if (ret != TDM_E_OK) {
		pr_err("Error in tdm_channel_close(%d)- ret %x\n", ch4_id, ret);
		ret = -ENXIO;
	}

	ret = tdm_port_close(h_port);
	pr_debug("%s tdm_port_close ret = %d\n", __func__, ret);
	if (ret != TDM_E_OK) {
		pr_err("Error in tdm_port_close(%d)- ret %x\n", ch1_id, ret);
		ret = -ENXIO;
	}

	tdm_thread_state = 0;

	return ret;

ch4_failed:
	tdm_channel_close(ch4_id, 1, h_port, h_channel4);
ch3_failed:
	tdm_channel_close(ch3_id, 1, h_port, h_channel3);
ch2_failed:
	tdm_channel_close(ch2_id, 1, h_port, h_channel2);
ch1_failed:
	tdm_channel_close(ch1_id, 1, h_port, h_channel1);
port1_failed:
	tdm_port_close(h_port);
	return -ENXIO;
}

static int test_attach_adapter(struct tdm_adapter *adap)
{
	pr_debug("tdm-dev: adapter [%s] registered as minor %d\n",
		 adap->name, adap->id);
	tdm_thread_state = 0;
	tdm_thread_task = kthread_run(tdm_thread, NULL, "tdm_thread");

	return 0;
}

static int test_detach_adapter(struct tdm_adapter *adap)
{
	if (tdm_thread_state)
		kthread_stop(tdm_thread_task);

	pr_debug("tdm-dev: adapter [%s] unregistered\n", adap->name);

	return 0;
}

static const struct tdm_device_id test_id[] = {
	{ "fsl_tdm", 0 },
	{ }
};

static struct tdm_driver test_tdmdev_driver = {
	.attach_adapter	= test_attach_adapter,
	.detach_adapter	= test_detach_adapter,
	.id_table = test_id,
};

static int __init tdm_test_init(void)
{
	int ret;
	pr_info("TDM_TEST: " DRV_DESC "\n");

	test_tdmdev_driver.id = 1;
	/* create a binding with TDM driver */
	ret = tdm_add_driver(&test_tdmdev_driver);
	if (ret == 0)
		pr_info("TDM_TEST module installed\n");
	else
		pr_err("%s tdm_port_init failed\n", __func__);

	return ret;

}

static void __exit tdm_test_exit(void)
{
	tdm_unregister_driver(&test_tdmdev_driver);
	pr_info("TDM_TEST module un-installed\n");
}

module_init(tdm_test_init);
module_exit(tdm_test_exit);
