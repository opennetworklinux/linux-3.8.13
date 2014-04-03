/*
 * drivers/tdm/line/slic_zarlink.h
 *
 * Copyright (C) 2009-2012 Freescale Semiconductor, Inc. All rights reserved.
 *
 * This is the header file for the SLIC  Driver Module
 * drivers/tdm/line/slic_zarlink.c.
 *
 * Author: Rajesh Gumasta<rajesh.gumasta@freescale.com>
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


#ifndef SLIC_ZARLINK_H
#define SLIC_ZARLINK_H

struct slic_platform_data {
	unsigned int ch1_rx_slot;
	unsigned int ch1_tx_slot;
	unsigned int ch2_rx_slot;
	unsigned int ch2_tx_slot;
};

/* SLIC channel configuration */
#define CH1_RX_SLOT_NUM		0
#define CH1_TX_SLOT_NUM		0
#define CH2_RX_SLOT_NUM		2
#define CH2_TX_SLOT_NUM		2

#define SLIC_SLOT_OFFSET	2

/* commands to the SLIC */
#define CHANNEL1			0x01
#define CHANNEL2			0x02
#define HW_RESET			0x04
#define WRITE_NO_OPERATION		0x06
#define WRITE_TX_TIME_SLOT		0x40
#define READ_TX_TIME_SLOT		0x41
#define WRITE_RX_TIME_SLOT		0x42
#define READ_RX_TIME_SLOT		0x43
#define WRITE_TXRXCLKSLOT_TXCLKEDGE	0x44
#define WRITE_DEVICE_CFG		0x46
#define READ_DEVICE_CONFIGURATION	0x47
#define WRITE_CHANNEL_ENABLE		0X4A
#define WRITE_IO_DATA			0x52
#define WRITE_IO_DIRECTION		0x54
#define WRITE_SYSTEM_STATE		0x56
#define WRITE_OPERATING_FUNCTION	0x60
#define READ_OPERATING_FUNCTION		0x61
#define WRITE_SYSTEM_STATE_CFG		0x68
#define READ_SYSTEM_STATE_CFG		0x69
#define WRITE_INT_MASK			0x6C
#define READ_INT_MASK			0x6D
#define WRITE_OPERATING_CONDITIONS	0x70
#define READ_PRODUCT_CODE		0X73
#define WRITE_CONVERTER_CFG		0xA6
#define WRITE_LOOP_SUPERVISION_PARAMS	0xC2
#define WRITE_DC_FEED_PARAMS		0xC6
#define WRITE_CADENCE_TIMER		0xE0
#define READ_CADENCE_TIMER		0xE1
#define WRITE_SWITCH_REGULATOR_PARAMS	0xE4
#define WRITE_SWITCH_REGULATOR_CTRL	0xE6
#define WRITE_INTERNAL_CFG_REG3		0xF2
#define WRITE_DC_CALIBRATION		0xFC

/* Dataset1 for no operation command */
static unsigned char dataset1_for_nooperation[] = {
					0xca, 0xfa, 0x98, 0xca, 0xb9,
					0xa2, 0x4c, 0x2b, 0xa2, 0xa3,
					0xa2, 0xae, 0x2b, 0x9a, 0x23,
					0xca, 0x26, 0x9f, 0x1,  0x8a,
					0x1d, 0x1,  0x1,  0x11, 0x1,
					0x90, 0x1,  0x90, 0x1,  0x90,
					0x1,  0x90, 0x1,  0x90, 0x88,
					0xd8, 0x70, 0x7a, 0x87, 0x23,
					0x3f, 0x4a, 0x97, 0x5a, 0xa7,
					0x5a, 0xaf, 0x82, 0x22, 0xe0,
					0x80, 0x32, 0x10, 0x50, 0x10,
					0x86, 0xa2, 0x63, 0x23, 0xbb,
					0x2a, 0xa4, 0x29, 0x7d, 0x87,
					0x2a, 0xfa, 0x8f, 0x29, 0xf0,
					0x96, 0x2e, 0x1
};

/* Dataset2 for no operation command */
static unsigned char dataset2_for_nooperation[] = {
					0xd2, 0x0,  0x0,  0x0,  0x0,
					0x36, 0x36, 0xb9, 0x0,  0x0,
					0x0,  0x0,  0x68, 0x0
};

/* Dataset3 for no operation command */
static unsigned char dataset3_for_nooperation[] = {
					0xc2, 0x1b, 0x84, 0xb4, 0x5,
					0xc6, 0x8,  0x8
};

/* Dataset for internal configuration register 3 command */
static unsigned char dataset_internalCfgReg3[] = {
					0x10, 0x1,  0x0, 0x0
};

/* Dataset for cadence timer command */
static unsigned char dataset_cadenceTimer[] = {
					0x3f, 0xff, 0x0,  0x0
};

/* Dataset for Loop parameters command */
static unsigned char dataset_writeLoopParams[] = {
					0x1b, 0x84, 0xb3, 0x5
};

/* Dataset1 for cadence timer command */
static unsigned char set_cadenceTimer[] = {
					0x01, 0x90, 0x03, 0x20
};

#endif
