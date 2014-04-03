/*
 * drivers/tdm/line_ctrl/slic_zarlink.c
 *
 * Copyright (C) 2008-2012 Freescale Semiconductor, Inc. All rights reserved.
 *
 * SLIC Line Control Module for Zarlink SLICs.
 * This  is a slic control and initialization module.
 *
 * Author:Poonam Aggrwal<poonam.aggrwal@freescale.com>
 *        Hemant Agrawal <hemant@freescale.com>
 *        Rajesh Gumasta <rajesh.gumasta@freescale.com>
 *
 * Modified by Sandeep Kr Singh <sandeep@freescale.com>
 *    1. Changed SPI cmnds to restrict transaction length to 1 byte.
 *    2. Updated probe which now does not relies on modalias.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This driver was created solely by Freescale, without the assistance,
 * support or intellectual property of Zarlink Semiconductor.  No maintenance
 * or support will be provided by Zarlink Semiconductor regarding this driver
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

 /* Note that this is a complete rewrite of Poonam's slic code.
    But we have used so much of her original code and ideas that it seems
    only fair to recognize her as co-author -- Rajesh & Hemant */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/spi/spi.h>
#include <linux/wait.h>
#include <linux/param.h>
#include <linux/delay.h>
#include <linux/of.h>
#include "slic_zarlink.h"

#define DRV_DESC "FREESCALE DEVELOPED ZARLINK SLIC DRIVER"
#define DRV_NAME "legerity"

#define MAX_NUM_OF_SLICS 10
#define SLIC_TRANS_LEN 1

#define TESTING_PRODUCT_CODE

static struct spi_device *g_spi;
struct spi_transfer t;

struct slic_channel {
	unsigned int ch1_rx_slot, ch1_tx_slot, ch2_rx_slot, ch2_tx_slot;
};
struct slic_channel slic_ch[MAX_NUM_OF_SLICS];
static int num_slics;

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Poonam Aggrwal<poonam.aggrwal@freescale.com>");
MODULE_DESCRIPTION(DRV_DESC);

static void
slic_cmd(struct spi_device *spi, unsigned char channel, unsigned char cmd,
		unsigned char len, unsigned char *cmdData)
{
	unsigned char ecCmd = WRITE_CHANNEL_ENABLE;
	unsigned char cmdLen;

	/* Write EC command */
	spi_write(spi, &ecCmd, SLIC_TRANS_LEN);

	/* write EC value */
	spi_write(spi, &channel, SLIC_TRANS_LEN);

	/* write command */
	spi_write(spi, &cmd, SLIC_TRANS_LEN);

	/* If read command or write command */
	if (cmd & 0x01) {
		for (cmdLen = 0; cmdLen < len; cmdLen++)
			spi_read(spi, &cmdData[cmdLen], SLIC_TRANS_LEN);
		}
	else {
		for (cmdLen = 0; cmdLen < len; cmdLen++)
			spi_write(spi, &cmdData[cmdLen], SLIC_TRANS_LEN);
	}
}

static void get_slic_product_code(struct spi_device *spi)
{
	u8 tx = READ_PRODUCT_CODE;
	u8 rx = 0x00;

	spi_write(spi, &tx, SLIC_TRANS_LEN);
	spi_read(spi, &rx, SLIC_TRANS_LEN);
	printk(KERN_INFO "SLIC: product code 1 read is  %x\n", rx);

	spi_read(spi, &rx, SLIC_TRANS_LEN);
	printk(KERN_INFO "SLIC: product code 2 read is  %x\n", rx);

	tx = WRITE_CHANNEL_ENABLE;
	spi_write(spi, &tx, SLIC_TRANS_LEN);
	spi_read(spi, &rx, SLIC_TRANS_LEN);
	printk(KERN_INFO "SLIC: config read is  %x\n", rx);

	tx = READ_DEVICE_CONFIGURATION;
	spi_write(spi, &tx, SLIC_TRANS_LEN);
	spi_read(spi, &rx, SLIC_TRANS_LEN);
	printk(KERN_INFO "SLIC: config read is  %x\n", rx);

	return;
}

static int slic_init_configure(unsigned char *device_handle,
		struct spi_device *spi, int slic_id)
{
	char temp1 = 0;
	char temp2[2];
	char temp3[3];
	unsigned char cad[4];
	unsigned char len;
	unsigned char channel_id;

	temp3[0] = 0x04;
	len = 0x01;
	slic_cmd(spi, CHANNEL1, HW_RESET, len, &temp3[0]);

#ifdef TESTING_PRODUCT_CODE
	get_slic_product_code(spi);
#endif
	temp3[0] = 0x82;
	len = 0x01;
	slic_cmd(spi, CHANNEL1, WRITE_DEVICE_CFG, len, &temp3[0]);

	temp3[0] = 0x7f;
	temp3[1] = 0xff;
	len = 0x02;
	slic_cmd(spi, CHANNEL1, WRITE_INT_MASK, len, &temp3[0]);

	temp3[0] = 0xff;
	temp3[1] = 0xff;
	len = 0x02;
	slic_cmd(spi, CHANNEL1, WRITE_INT_MASK, len, &temp3[0]);

	temp3[0] = 0x40;
	len = 0x01;
	slic_cmd(spi, CHANNEL1, WRITE_TXRXCLKSLOT_TXCLKEDGE, len,
			&temp3[0]);

	temp3[0] = 0x0;
	len = 0x01;
	slic_cmd(spi, CHANNEL1, WRITE_SYSTEM_STATE, len, &temp3[0]);

	temp3[0] = 0x0;
	len = 0x01;
	slic_cmd(spi, CHANNEL2, WRITE_SYSTEM_STATE, len, &temp3[0]);

	/* Put the Switching regulators in disabled mode */
	temp3[0] = 0x0;
	len = 0x01;
	slic_cmd(spi, CHANNEL1, WRITE_SWITCH_REGULATOR_CTRL, len,
			&temp3[0]);

	temp3[0] = 0x0;
	len = 0x01;
	slic_cmd(spi, CHANNEL1, WRITE_SWITCH_REGULATOR_CTRL, len,
			&temp3[0]);

	temp3[0] = 0x3;
	len = 0x01;
	slic_cmd(spi, CHANNEL1, WRITE_SYSTEM_STATE_CFG, len,
			&temp3[0]);

	temp3[0] = 0x0;
	len = 0x01;
	slic_cmd(spi, CHANNEL1, WRITE_SYSTEM_STATE, len, &temp3[0]);

	temp3[0] = 0x3;
	len = 0x01;
	slic_cmd(spi, CHANNEL2, WRITE_SYSTEM_STATE_CFG, len,
			&temp3[0]);

	temp3[0] = 0x0;
	len = 0x01;
	slic_cmd(spi, CHANNEL2, WRITE_SYSTEM_STATE, len, &temp3[0]);

	temp3[0] = 0x2b;
	len = 0x01;
	slic_cmd(spi, CHANNEL1, WRITE_SYSTEM_STATE, len, &temp3[0]);

	temp3[0] = 0x80;
	len = 0x01;
	slic_cmd(spi, CHANNEL1, WRITE_OPERATING_FUNCTION, len,
			&temp3[0]);

	temp3[0] = 0xe0;
	len = 0x01;
	slic_cmd(spi, CHANNEL1, WRITE_OPERATING_CONDITIONS, len,
			&temp3[0]);

	temp3[0] = 0x1;
	len = 0x01;
	slic_cmd(spi, CHANNEL1, WRITE_CONVERTER_CFG, len, &temp3[0]);

	/* Set Switching Paramenters as for Le88266
	 * 1. BSI[1:0] = 00b (sense pin VBL is SWVSY, VBH is SWVSZ)
	 * 2. SWFS[1:0] = 00b (setting frequency as 384kHz in high power mode)
	 * 3. SWYV[4:0] = 00101b (setting to -25V)
	 * 4. SWZV[4:0] = 00000b (setting to 0V)
	 */
	temp3[0] = 0x00;
	temp3[1] = 0x05;
	temp3[2] = 0x00;
	len = 0x03;
	slic_cmd(spi, CHANNEL1, WRITE_SWITCH_REGULATOR_PARAMS, len,
			&temp3[0]);

	/* Put the Switching regulators in
	 * 1. Regulator Y & Z in low power state
	 * 2. Over voltage protection enabled
	 */
	temp3[0] = 0x15;
	len = 0x01;
	slic_cmd(spi, CHANNEL1, WRITE_SWITCH_REGULATOR_CTRL, len,
			&temp3[0]);

	/* Wait 20ms before switching from low power to high power */
	mdelay(20);

	temp3[0] = 0x9;
	len = 0x01;
	slic_cmd(spi, CHANNEL1, WRITE_CONVERTER_CFG, len, &temp3[0]);

	temp3[0] = 0xb;
	len = 0x01;
	slic_cmd(spi, CHANNEL1, WRITE_CONVERTER_CFG, len, &temp3[0]);

	temp3[0] = 0xb;
	len = 0x01;
	slic_cmd(spi, CHANNEL1, WRITE_CONVERTER_CFG, len, &temp3[0]);

	temp3[0] = 0x1;
	len = 0x01;
	slic_cmd(spi, CHANNEL1, WRITE_CONVERTER_CFG, len, &temp3[0]);

	temp3[0] = 0x1;
	len = 0x01;
	slic_cmd(spi, CHANNEL2, WRITE_CONVERTER_CFG, len, &temp3[0]);

	temp3[0] = 0x2;
	len = 0x01;
	slic_cmd(spi, CHANNEL1, WRITE_CONVERTER_CFG, len, &temp3[0]);

	temp3[0] = 0x2;
	len = 0x01;
	slic_cmd(spi, CHANNEL1, WRITE_CONVERTER_CFG, len, &temp3[0]);

	temp3[0] = 0x3;
	len = 0x01;
	slic_cmd(spi, CHANNEL1, WRITE_CONVERTER_CFG, len, &temp3[0]);

	temp3[0] = 0x3;
	len = 0x01;
	slic_cmd(spi, CHANNEL1, WRITE_CONVERTER_CFG, len, &temp3[0]);

	/* Set Switching Paramenters as for Le88266
	 * 1. BSI[1:0] = 00b (sense pin VBL is SWVSY, VBH is SWVSZ)
	 * 2. SWFS[1:0] = 00b (setting frequency as 384kHz in high power mode)
	 * 3. SWYV[4:0] = 00101b (setting to -25V)
	 * 4. SWZV[4:0] = 00000b (setting to 0V)
	 */
	temp3[0] = 0x00;
	temp3[1] = 0x05;
	temp3[2] = 0x00;
	len = 0x03;
	slic_cmd(spi, CHANNEL1, WRITE_SWITCH_REGULATOR_PARAMS, len,
			&temp3[0]);

	/* Put the Switching regulators in
	 * 1. Regulator Y & Z in high power state
	 * 2. Over voltage protection enabled
	 */
	temp3[0] = 0x1f;
	len = 0x01;
	slic_cmd(spi, CHANNEL1, WRITE_SWITCH_REGULATOR_CTRL, len,
			&temp3[0]);

	/* Setting the channel specific parameters */
	for (channel_id = CHANNEL1; channel_id <= CHANNEL2; channel_id++) {

		/* Set the IO direction to Output - to energise the fxo
		 * relay */
		temp3[0] = 0x1;
		len = 0x01;
		slic_cmd(spi, channel_id, WRITE_IO_DIRECTION, len,
				&temp3[0]);

		temp3[0] = 0x0;
		len = 0x01;
		slic_cmd(spi, channel_id, WRITE_IO_DATA, len,
				&temp3[0]);

		temp3[0] = 0x0;
		len = 0x01;
		slic_cmd(spi, channel_id, WRITE_OPERATING_CONDITIONS,
				len, &temp3[0]);

		len = sizeof(dataset_cadenceTimer) / sizeof(unsigned char);
		slic_cmd(spi, channel_id, WRITE_CADENCE_TIMER, len,
				&dataset_cadenceTimer[0]);

		temp3[0] = 0x2;
		len = 0x01;
		slic_cmd(spi, channel_id, WRITE_SYSTEM_STATE_CFG,
				len, &temp3[0]);

		temp3[0] = 0xc0;
		len = 0x01;
		slic_cmd(spi, channel_id, WRITE_OPERATING_CONDITIONS,
				len, &temp3[0]);

		temp3[0] = 0x0;
		temp3[1] = 0x2;
		len = 0x02;
		slic_cmd(spi, channel_id, WRITE_DC_CALIBRATION, len,
			       &temp3[0]);

		temp3[0] = 0x0;
		len = 0x01;
		slic_cmd(spi, channel_id, WRITE_SYSTEM_STATE, len,
				&temp3[0]);

		len = sizeof(dataset_writeLoopParams) / sizeof(unsigned char);
		slic_cmd(spi, channel_id,
				WRITE_LOOP_SUPERVISION_PARAMS, len,
				&dataset_writeLoopParams[0]);

		temp3[0] = 0x13;
		temp3[1] = 0x8;
		len = 0x02;
		slic_cmd(spi, channel_id, WRITE_DC_FEED_PARAMS, len,
				&temp3[0]);

		len = sizeof(dataset1_for_nooperation) / sizeof(unsigned char);
		slic_cmd(spi, channel_id, WRITE_NO_OPERATION, len,
				&dataset1_for_nooperation[0]);

		temp3[0] = 0x3f;
		len = 0x01;
		slic_cmd(spi, channel_id, WRITE_OPERATING_FUNCTION,
				len, &temp3[0]);

		len = sizeof(dataset2_for_nooperation) / sizeof(unsigned char);
		slic_cmd(spi, channel_id, WRITE_NO_OPERATION, len,
				&dataset2_for_nooperation[0]);

		temp3[0] = 0x2;
		len = 0x1;
		slic_cmd(spi, channel_id, WRITE_SYSTEM_STATE_CFG,
				len, &temp3[0]);

		len = sizeof(dataset_internalCfgReg3) / sizeof(unsigned char);
		slic_cmd(spi, channel_id, WRITE_INTERNAL_CFG_REG3 ,
				len, &dataset_internalCfgReg3[0]);

		len = sizeof(dataset3_for_nooperation) / sizeof(unsigned char);
		slic_cmd(spi, channel_id, WRITE_NO_OPERATION, len,
				&dataset3_for_nooperation[0]);

		temp3[0] = 0xbf;
		len = 0x1;
		slic_cmd(spi, channel_id, WRITE_OPERATING_FUNCTION,
				len, &temp3[0]);

		temp3[0] = 0xc0;
		len = 0x1;
		slic_cmd(spi, channel_id, WRITE_OPERATING_CONDITIONS,
				len, &temp3[0]);

		temp3[0] = 0x6;
		len = 0x1;
		slic_cmd(spi, channel_id, WRITE_SYSTEM_STATE_CFG,
				len, &temp3[0]);

		temp3[0] = 0x6;
		len = 0x1;
		slic_cmd(spi, channel_id, WRITE_SYSTEM_STATE_CFG,
				len, &temp3[0]);

		temp3[0] = 0xc0;
		len = 0x1;
		slic_cmd(spi, channel_id, WRITE_OPERATING_CONDITIONS,
				len, &temp3[0]);

		temp3[0] = 0x16;
		len = 0x01;
		slic_cmd(spi, channel_id, WRITE_SYSTEM_STATE_CFG,
				len, &temp3[0]);

		temp3[0] = 0xc0;
		len = 0x01;
		slic_cmd(spi, channel_id, WRITE_OPERATING_CONDITIONS,
				len, &temp3[0]);

		temp3[0] = 0x16;
		len = 0x01;
		slic_cmd(spi, channel_id, WRITE_SYSTEM_STATE_CFG,
				len, &temp3[0]);

		temp3[0] = 0x3f;
		if (channel_id == CHANNEL1)
			temp3[1] = 0xff;
		else
			temp3[1] = 0xbf;
		len = 0x02;
		slic_cmd(spi, channel_id, WRITE_INT_MASK, len,
				&temp3[0]);

		temp3[0] = 0x16;
		len = 0x01;
		slic_cmd(spi, channel_id, WRITE_SYSTEM_STATE_CFG,
				len, &temp3[0]);

		temp3[0] = 0xc0;
		len = 0x01;
		slic_cmd(spi, channel_id, WRITE_OPERATING_CONDITIONS,
				len, &temp3[0]);

		temp3[0] = 0x0;
		len = 0x01;
		slic_cmd(spi, channel_id, WRITE_OPERATING_CONDITIONS,
				len, &temp3[0]);

		temp3[0] = 0x0;
		temp3[1] = 0x2;
		len = 0x02;
		slic_cmd(spi, channel_id, WRITE_DC_CALIBRATION, len,
				&temp3[0]);

		temp3[0] = 0x2b;
		len = 0x01;
		slic_cmd(spi, channel_id, WRITE_SYSTEM_STATE, len,
				&temp3[0]);

	}
	/* Reading the Device Configuration register */
	len = 0x1;
	slic_cmd(spi, CHANNEL1, READ_DEVICE_CONFIGURATION, len,
			&temp1);
	printk(KERN_INFO "DEV reg is %x\n", temp1);

	/* Enabling interrupt by writing into Device Configuration Register */
	temp1 &= 0x7F;
	len = 0x01;
	slic_cmd(spi, CHANNEL1, WRITE_DEVICE_CFG, len, &temp1);

	/*  Reading the Device Configuration register */
	len = 0x1;
	slic_cmd(spi, CHANNEL1, READ_DEVICE_CONFIGURATION, len,
			&temp1);
	printk(KERN_INFO "DEV reg after is %x\n", temp1);

	/*  Reading the Mask register */
	len = 0x2;
	slic_cmd(spi, CHANNEL1, READ_INT_MASK, len, &temp2[0]);
	printk(KERN_INFO "Mask reg before setting is %x %x\n",
					 temp2[0], temp2[1]);

	/*  Writing into the mask register */
	temp2[0] = 0xF6;
	temp2[1] = 0xF6;
	len = 0x2;
	slic_cmd(spi, CHANNEL1, WRITE_INT_MASK, len, &temp2[0]);

	/*  Reading the Mask register */
	len = 0x2;
	slic_cmd(spi, CHANNEL1, READ_INT_MASK, len, &temp2[0]);
	printk(KERN_INFO "Mask reg after setting is %x %x\n",
					 temp2[0], temp2[1]);

	temp1 = slic_ch[slic_id].ch1_tx_slot;
	len = 0x1;
	slic_cmd(spi, CHANNEL1, WRITE_TX_TIME_SLOT, len, &temp1);

	len = 0x1;
	slic_cmd(spi, CHANNEL1, READ_TX_TIME_SLOT, len, &temp1);
	printk(KERN_INFO "Read Tx Timeslot for CH1 is %x\n", temp1);

	temp1 = slic_ch[slic_id].ch2_tx_slot;
	len = 0x1;
	slic_cmd(spi, CHANNEL2, WRITE_TX_TIME_SLOT, len, &temp1);

	len = 0x1;
	slic_cmd(spi, CHANNEL2, READ_TX_TIME_SLOT, len, &temp1);
	printk(KERN_INFO "Read Tx Timeslot for CH2 is %x\n", temp1);

	temp1 = slic_ch[slic_id].ch1_rx_slot;
	len = 0x1;
	slic_cmd(spi, CHANNEL1, WRITE_RX_TIME_SLOT, len, &temp1);

	len = 0x1;
	slic_cmd(spi, CHANNEL1, READ_RX_TIME_SLOT, len, &temp1);
	printk(KERN_INFO "Read Rx Timeslot for CH1 is %x\n", temp1);

	temp1 = slic_ch[slic_id].ch2_rx_slot;
	len = 0x1;
	slic_cmd(spi, CHANNEL2, WRITE_RX_TIME_SLOT, len, &temp1);

	len = 0x1;
	slic_cmd(spi, CHANNEL2, READ_RX_TIME_SLOT, len, &temp1);
	printk(KERN_INFO "Read Rx Timeslot for CH2 is %x\n", temp1);

	for (channel_id = CHANNEL1; channel_id <= CHANNEL2; channel_id++) {

		temp1 &= 0xBF;
		temp1 |= 0x80;
		len = 0x01;
		slic_cmd(spi, channel_id, WRITE_OPERATING_FUNCTION,
				len, &temp1);

		len = 0x01;
		slic_cmd(spi, channel_id, READ_OPERATING_FUNCTION,
				len, &temp1);
		printk(KERN_INFO "Operating Fun for channel %d is %x\n",
				channel_id, temp1);

		/* Install Timers */
		len = 0x04;
		slic_cmd(spi, channel_id, READ_CADENCE_TIMER,	len,
				&cad[0]);
		printk(KERN_INFO "Cadence Timer Reg for CH%d before is %x %x"
				"%x %x\n", channel_id, cad[0], cad[1], cad[2],
			       cad[3]);

		len = sizeof(set_cadenceTimer) / sizeof(unsigned char);
		slic_cmd(spi, channel_id, WRITE_CADENCE_TIMER, len,
				&set_cadenceTimer[0]);

		len = 0x04;
		slic_cmd(spi, channel_id, READ_CADENCE_TIMER , len,
				&cad[0]);
		printk(KERN_INFO "Cadence Timer Reg for CH%d after is %x %x"
				"%x %x\n", channel_id, cad[0], cad[1], cad[2],
				cad[3]);
		temp1 = 0x20;
		len = 0x01;
		slic_cmd(spi, channel_id, WRITE_SYSTEM_STATE_CFG,
				len, &temp1);

		slic_cmd(spi, channel_id, READ_SYSTEM_STATE_CFG,
				len, &temp1);
		printk(KERN_INFO "Switching control for channel %d is %x\n",
				channel_id, temp1);
	}
	return 0;
}

void configure_spi_pdata(struct spi_device *spi)
{
	struct slic_platform_data *spi_slic_pdata;
	const u32 *iprop;
	struct device_node *np = spi->dev.of_node;
	static int num_slic;

	spi_slic_pdata = kzalloc(sizeof(*spi_slic_pdata), GFP_KERNEL);
	if (spi_slic_pdata == NULL)
		return;

	spi->dev.platform_data = spi_slic_pdata;

	spi_slic_pdata->ch1_rx_slot = CH1_RX_SLOT_NUM + num_slic;
	spi_slic_pdata->ch1_tx_slot = CH1_TX_SLOT_NUM + num_slic;
	spi_slic_pdata->ch2_rx_slot = CH2_RX_SLOT_NUM + num_slic;
	spi_slic_pdata->ch2_tx_slot = CH2_TX_SLOT_NUM + num_slic;
	pr_info("SLIC config success\n");
	num_slic = num_slic + SLIC_SLOT_OFFSET;

}
static int slic_remove(struct spi_device *spi)
{

	printk(KERN_INFO "SLIC module uninstalled\n");
	return 0;
}

static int slic_probe(struct spi_device *spi)
{
	int ret = 0;
	unsigned char *device_handle;
	struct slic_platform_data *data;

	printk(KERN_INFO "SLIC probed!\n");

	g_spi = spi;
	spi->bits_per_word = 8;

	if (num_slics >= MAX_NUM_OF_SLICS) {
		printk(KERN_ERR "Exceeded the max number of slics\n");
		return ret;
	}

	/* Initialize the SLIC */
	configure_spi_pdata(spi);
	data = spi->dev.platform_data;
	slic_ch[num_slics].ch1_tx_slot = data->ch1_tx_slot;
	slic_ch[num_slics].ch1_rx_slot = data->ch1_rx_slot;
	slic_ch[num_slics].ch2_tx_slot = data->ch2_tx_slot;
	slic_ch[num_slics].ch2_rx_slot = data->ch2_rx_slot;

	device_handle = 0x0;
	ret = slic_init_configure(device_handle, spi, num_slics);
	if (ret == 0) {
		num_slics++;
		printk(KERN_INFO "SLIC %d configuration success\n",
				num_slics);
	} else {
		printk(KERN_ERR "%s slic configuration failed\n", __func__);
		return ret;
	}
	return ret;
}

static const struct of_device_id slic_match[] = {
	{
	 .compatible = "zarlink,le88266",
	 },
	{},
};

static struct spi_driver slic_driver = {
	.driver = {
		   .name = "legerity",
		   .bus = &spi_bus_type,
		   .owner = THIS_MODULE,
		   .of_match_table = slic_match,
		   },
	.probe = slic_probe,
	.remove = slic_remove,

};

static int __init slic_init(void)
{
	int ret;
	printk(KERN_INFO "SLIC: " DRV_DESC "\n");
	printk(KERN_INFO  "####################################################"
			"\n# This driver was created solely by Freescale,     #"
			"\n# without the assistance, support or intellectual  #"
			"\n# property of Zarlink Semiconductor. No            #"
			"\n# maintenance or support will be provided by       #"
			"\n# Zarlink  Semiconductor regarding this driver.    #"
			"\n####################################################"
		"\n");

	ret = spi_register_driver(&slic_driver);
	if (ret != 0)
		printk(KERN_ERR "%s spi_register_driver failed\n",
							__func__);
	return ret;
}

static void __exit slic_exit(void)
{
	spi_unregister_driver(&slic_driver);
}

module_init(slic_init);
module_exit(slic_exit);
