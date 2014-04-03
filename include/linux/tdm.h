/* include/linux/tdm.h
 *
 * Copyright 2012 Freescale Semiconductor, Inc.
 *
 * tdm.h - definitions for the tdm-device framework interface
 *
 * Author:Hemant Agrawal <hemant@freescale.com>
 *	Rajesh Gumasta <rajesh.gumasta@freescale.com>
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


#ifndef _LINUX_TDM_H
#define _LINUX_TDM_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/device.h>	/* for struct device */
#include <linux/sched.h>	/* for completion */
#include <linux/mutex.h>
#include <linux/interrupt.h>

#define CHANNEL_8BIT_LIN	0	/* 8 bit linear */
#define CHANNEL_8BIT_ULAW	1	/* 8 bit Mu-law */
#define CHANNEL_8BIT_ALAW	2	/* 8 bit A-law */
#define CHANNEL_16BIT_LIN	3	/* 16 bit Linear */

#define NUM_CHANNELS		16
#define NUM_SAMPLES_PER_MS	8		/* 8 samples per milli sec per
						 channel. Req for voice data */
#define NUM_MS			10
#define NUM_SAMPLES_PER_FRAME	(NUM_MS * NUM_SAMPLES_PER_MS) /* Number of
						samples for 1 client buffer */
#define NUM_OF_TDM_BUF		3

/* General options */

struct tdm_adapt_algorithm;
struct tdm_adapter;
struct tdm_port;
struct tdm_driver;

/* Align addr on a size boundary - adjust address up if needed */
/* returns min value greater than size which is multiple of alignment */
static inline int ALIGN_SIZE(u64 size, u32 alignment)
{
	return (size + alignment - 1) & (~(alignment - 1));
}

int tdm_master_send(struct tdm_adapter *adap, void **buf, int count);
int tdm_master_recv(struct tdm_adapter *adap, void **buf);
int tdm_read_direct(struct tdm_adapter *adap, u8 *buf, u32 len);
int tdm_write_direct(struct tdm_adapter *adap, u8 *buf, u32 len);

/**
 * struct tdm_driver - represent an TDM device driver
 * @class: What kind of tdm device we instantiate (for detect)
 * @id:Driver id
 * @name: Name of the driver
 * @attach_adapter: Callback for device addition (for legacy drivers)
 * @detach_adapter: Callback for device removal (for legacy drivers)
 * @probe: Callback for device binding
 * @remove: Callback for device unbinding
 * @shutdown: Callback for device shutdown
 * @suspend: Callback for device suspend
 * @resume: Callback for device resume
 * @command: Callback for sending commands to device
 * @id_table: List of TDM devices supported by this driver
 * @list: List of drivers created (for tdm-core use only)
 */
struct tdm_driver {
	unsigned int class;
	unsigned int id;
	char name[TDM_NAME_SIZE];

	int (*attach_adapter)(struct tdm_adapter *);
	int (*detach_adapter)(struct tdm_adapter *);

	/* Standard driver model interfaces */
	int (*probe)(const struct tdm_device_id *);
	int (*remove)(void);

	/* driver model interfaces that don't relate to enumeration */
	void (*shutdown)(void);
	int (*suspend)(pm_message_t mesg);
	int (*resume)(void);

	/* a ioctl like command that can be used to perform specific functions
	 * with the device.
	 */
	int (*command)(unsigned int cmd, void *arg);

	const struct tdm_device_id *id_table;

	/* The associated adapter for this driver */
	struct tdm_adapter *adapter;
	struct list_head list;
};

/* tdm per port statistics structure, used for providing and storing tdm port
 * statistics.
 */
struct tdm_port_stats {
	unsigned int rx_pkt_count;	/* Rx frame count per channel */
	unsigned int rx_pkt_drop_count;	/* Rx drop count per channel to
					 clean space for new buffer */
	unsigned int tx_pkt_count;	/* Tx frame count per channel */
	unsigned int tx_pkt_conf_count;	/* Tx frame confirmation count per
					 channel */
	unsigned int tx_pkt_drop_count;	/* Tx drop count per channel due to
					 queue full */
};


/* tdm Buffer Descriptor, used for Creating Interleaved and De-interleaved
 * FIFOs
 */
struct tdm_bd {
	unsigned char flag;		/* BD is full or empty */
	unsigned char wrap;		/* BD is last in the queue */
	unsigned short length;	/* Length of Data in BD */
	/*TODO: use dyanmic memory */
	unsigned short p_data[NUM_SAMPLES_PER_FRAME];	/* Data Pointer */
	unsigned long offset;	/* Offset of the Data Pointer to be used */
};

#define TDM_CH_RX_BD_RING_SIZE	3
#define TDM_CH_TX_BD_RING_SIZE	3

/* tdm RX-TX Channelised Data */
struct tdm_port_data {
	struct tdm_bd rx_data_fifo[TDM_CH_RX_BD_RING_SIZE]; /* Rx Channel Data
								BD Ring */
	struct tdm_bd *rx_in_data;	/* Current Channel Rx BD to be filled by
						de-interleave function */
	struct tdm_bd *rx_out_data;	/* Current Channel Rx BD to be
							read by App */
	struct tdm_bd tx_data_fifo[TDM_CH_TX_BD_RING_SIZE]; /* Tx Channel Data
								BD Ring */
	struct tdm_bd *tx_in_data;	/* Current Channel Tx BD to be
						 filled by App */
	struct tdm_bd *tx_out_data;	/* Current Channel Tx BD to be read by
						interleave function */
	spinlock_t rx_channel_lock;	/* Spin Lock for Rx Channel */
	spinlock_t tx_channel_lock;	/* Spin Lock for Tx Channel */
};

/* structure tdm_port_cfg - contains configuration params for a port */
struct tdm_port_cfg {
	unsigned short port_mode;
};

/* struct tdm_port - represent an TDM ports for a device */
struct tdm_port {
	unsigned short port_id;
	unsigned short in_use;		/* Port is enabled? */
	uint16_t rx_max_frames;		/* Received Port frames
					 before allowing Read Operation in
					 Port Mode */

	struct tdm_port_stats port_stat;/* A structure parameters defining
					 TDM port statistics. */
	struct tdm_port_data *p_port_data;	/* a structure parameters
						defining tdm channelised data */

	struct tdm_driver *driver;	/* driver for this port */
	struct tdm_adapter *adapter;	/* adapter for this port */
	struct list_head list;		/* list of ports */
	struct list_head mychannels;	/* list of channels, created on this
					 port*/
	spinlock_t ch_list_lock;	/* Spin Lock for channel_list */
	struct tdm_port_cfg port_cfg;/* A structure parameters defining
					 TDM port configuration. */
};

/* tdm RX-TX Channelised Data */
struct tdm_ch_data {
	struct tdm_bd rx_data_fifo[TDM_CH_RX_BD_RING_SIZE]; /* Rx Port Data BD
								Ring */
	struct tdm_bd *rx_in_data;	/* Current Port Rx BD to be filled by
						de-interleave function */
	struct tdm_bd *rx_out_data; /* Current Port Rx BD to be read by App */
	struct tdm_bd tx_data_fifo[TDM_CH_TX_BD_RING_SIZE]; /* Tx Port Data BD
								Ring */
	struct tdm_bd *tx_in_data;	/* Current Port Tx BD to be filled by
						App */
	struct tdm_bd *tx_out_data;	/* Current Port Tx BD to be read by
						interleave function */
	spinlock_t rx_channel_lock;	/* Spin Lock for Rx Port */
	spinlock_t tx_channel_lock;	/* Spin Lock for Tx Port */
};

/* Channel config params */
struct tdm_ch_cfg {
	unsigned short num_slots;
	unsigned short first_slot;
};

/* struct tdm_channel- represent a TDM channel for a port */
struct tdm_channel {
	u16 ch_id;			/* logical channel number */
	struct list_head list;		/* list of channels in a port*/
	struct tdm_port *port;		/* port for this channel */
	u16 in_use;			/* channel is enabled? */
	struct tdm_ch_cfg ch_cfg;	/* channel configuration */
	struct tdm_ch_data *p_ch_data;	/* data storage space for channel */
	wait_queue_head_t ch_wait_queue;/* waitQueue for RX Channel Data */
};

/* tdm_adapt_algorithm is for accessing the routines of device */
struct tdm_adapt_algorithm {
	u32 (*tdm_read)(struct tdm_adapter *, u16 **);
	u32 (*tdm_get_write_buf)(struct tdm_adapter *, u16 **);
	int (*tdm_read_simple)(struct tdm_adapter *, u8 *, u32 len);
	int (*tdm_write_simple)(struct tdm_adapter *, u8 *, u32 len);
	u32 (*tdm_write)(struct tdm_adapter *, void * , unsigned int len);
	int (*tdm_enable)(struct tdm_adapter *);
	int (*tdm_disable)(struct tdm_adapter *);
};

/* tdm_adapter_mode is to define in mode of the device */
enum tdm_adapter_mode {
	e_TDM_ADAPTER_MODE_NONE = 0x00,
	e_TDM_ADAPTER_MODE_T1 = 0x01,
	e_TDM_ADAPTER_MODE_E1 = 0x02,
	e_TDM_ADAPTER_MODE_T1_RAW = 0x10,
	e_TDM_ADAPTER_MODE_E1_RAW = 0x20,
};

/* tdm_port_mode defines the mode in which the port is configured to operate
 * It can be channelized/full/fractional.
 */
enum tdm_port_mode {
	e_TDM_PORT_CHANNELIZED = 0	/* Channelized mode */
	, e_TDM_PORT_FULL = 1		/* Full mode */
	, e_TDM_PORT_FRACTIONAL = 2	/* Fractional mode */
};

/* tdm_process_mode used for testing the tdm device in normal mode or internal
 * loopback or external loopback
 */
enum tdm_process_mode {
	e_TDM_PROCESS_NORMAL = 0	/* Normal mode */
	, e_TDM_PROCESS_INT_LPB = 1	/* Internal loop mode */
	, e_TDM_PROCESS_EXT_LPB = 2	/* External Loopback mode */
};


/* TDM configuration parameters */
struct fsl_tdm_adapt_cfg {
	u8 num_ch;		/* Number of channels in this adpater */
	u8 ch_size_type;		/* reciever/transmit channel
						size for all channels */
	u8 slot_width;		/* 1 or 2 Is defined by channel type */
	u8 frame_len;		/* Length of frame in samples */
	u32 num_frames;
	u8 loopback;			/* loopback or normal */
	u8 adap_mode;			/* 0=None, 1= T1, 2= T1-FULL, 3=E1,
						4 = E1-FULL */
	int max_num_ports;		/* Not Used: Max Number of ports that
					can be created on this adapter */
	int max_timeslots;		/* Max Number of timeslots that are
					supported on this adapter */
};

/*
 * tdm_adapter is the structure used to identify a physical tdm device along
 * with the access algorithms necessary to access it.
 */
struct tdm_adapter {
	struct module *owner;	/* owner of the adapter module */
	unsigned int id;	/* Adapter Id */
	unsigned int class;	/* classes to allow probing for */
	unsigned int drv_count;	/* Number of drivers associated with the
				 adapter */

	const struct tdm_adapt_algorithm *algo;	/* the algorithm to access the
						 adapter*/

	char name[TDM_NAME_SIZE];	/* Name of Adapter */
	struct mutex adap_lock;
	struct device *parent;		/*Not Used*/

	struct tasklet_struct tdm_data_tasklet;	/* tasklet handle to perform
						 data processing*/
	int tasklet_conf;	/* flag for tasklet configuration */
	int tdm_rx_flag;

	struct list_head myports;	/* list of ports, created on this
					 adapter */
	struct list_head list;
	spinlock_t portlist_lock;	/* Spin Lock for port_list */
	void *data;
	struct fsl_tdm_adapt_cfg adapt_cfg;
};

static inline void *tdm_get_adapdata(const struct tdm_adapter *dev)
{
	return dev->data;
}

static inline void tdm_set_adapdata(struct tdm_adapter *dev, void *data)
{
	dev->data = data;
}

/* functions exported by tdm.o */

extern int tdm_add_adapter(struct tdm_adapter *);
extern int tdm_del_adapter(struct tdm_adapter *);
extern int tdm_register_driver(struct tdm_driver *);
extern void tdm_del_driver(struct tdm_driver *);
extern void tdm_unregister_driver(struct tdm_driver *);
extern void init_config_adapter(struct tdm_adapter *);
extern int tdm_adap_enable(struct tdm_driver *drv);
extern int tdm_adap_disable(struct tdm_driver *drv);
extern unsigned int tdm_port_open(struct tdm_driver *, void **);
extern unsigned int tdm_port_close(void *);
extern unsigned int tdm_port_ioctl(void *, unsigned int, unsigned long);
extern unsigned int tdm_channel_read(void *, void *, void *, u16 *);
extern unsigned int tdm_channel_write(void *, void * , void *, u16);
extern unsigned int tdm_ch_poll(void *, unsigned int);

extern int tdm_channel_open(u16, u16, struct tdm_port *, void **);
extern int tdm_channel_close(u16, u16, struct tdm_port *,
						struct tdm_channel *);

static inline int tdm_add_driver(struct tdm_driver *driver)
{
	return tdm_register_driver(driver);
}

extern struct tdm_adapter *tdm_get_adapter(int id);
extern void tdm_put_adapter(struct tdm_adapter *adap);

#endif /* __KERNEL__ */

#define TDM_E_OK 0

#endif /* _LINUX_TDM_H */
