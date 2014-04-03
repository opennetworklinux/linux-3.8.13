/* driver/tdm/tdm-core.c
 *
 * Copyright 2012 Freescale Semiconductor, Inc.
 *
 * TDM core is the interface between TDM clients and TDM devices.
 * It is also intended to serve as an interface for line controld
 * devices later on.
 *
 * Author:Hemant Agrawal <hemant@freescale.com>
 *	Rajesh Gumasta <rajesh.gumasta@freescale.com>
 *
 * Modified by Sandeep Kr Singh <sandeep@freescale.com>
 *		Poonam Aggarwal <poonam.aggarwal@freescale.com>
 * 1. Added framework based initilization of device.
 * 2. All the init/run time configuration is now done by framework.
 * 3. Added channel level operations.
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

/* if read write debug required */
#undef TDM_CORE_DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/tdm.h>
#include <linux/init.h>
#include <linux/idr.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/hardirq.h>
#include <linux/irqflags.h>
#include <linux/list.h>
#include <linux/uaccess.h>
#include <linux/io.h>


static DEFINE_MUTEX(tdm_core_lock);
static DEFINE_IDR(tdm_adapter_idr);
/* List of TDM adapters registered with TDM framework */
LIST_HEAD(adapter_list);

/* List of TDM clients registered with TDM framework */
LIST_HEAD(driver_list);

/* In case the previous data is not fetched by the client driver, the
 * de-interleaving function will  discard the old data and rewrite the
 * new data */
static int use_latest_tdm_data = 1;

/* this tasklet is created for each adapter instance */
static void tdm_data_tasklet_fn(unsigned long);

/* tries to match client driver with the adapter */
static int tdm_device_match(struct tdm_driver *driver, struct tdm_adapter *adap)
{
	/* match on an id table if there is one */
	if (driver->id_table && driver->id_table->name[0]) {
		if (!(strcmp(driver->id_table->name, adap->name)))
			return (int)driver->id_table;
	}
	return TDM_E_OK;
}

static int tdm_attach_driver_adap(struct tdm_driver *driver,
					struct tdm_adapter *adap)
{
	int ret = TDM_E_OK;
	/* if driver is already attached to any other adapter, return*/
	if (driver->adapter && (driver->adapter != adap))
		return ret;

	driver->adapter = adap;

	if (driver->attach_adapter) {
		ret = driver->attach_adapter(adap);
		if (ret < 0) {
			pr_err("attach_adapter failed for driver [%s] err:%d\n"
				, driver->name, ret);
			return ret;
		}
	}
	adap->drv_count++;

	if (!adap->tasklet_conf) {
		tasklet_init(&adap->tdm_data_tasklet, tdm_data_tasklet_fn,
						(unsigned long)adap);
		adap->tasklet_conf = 1;
	}

	return ret;
}

/* Detach client driver and adapter */
static int tdm_detach_driver_adap(struct tdm_driver *driver,
					struct tdm_adapter *adap)
{
	int res = TDM_E_OK;

	if (!driver->adapter || (driver->adapter != adap))
		return TDM_E_OK;

	if (!driver->detach_adapter)
		return TDM_E_OK;

	adap->drv_count--;

	/* If no more driver is registed with the adapter*/
	if (!adap->drv_count && adap->tasklet_conf) {
		tasklet_disable(&adap->tdm_data_tasklet);
		tasklet_kill(&adap->tdm_data_tasklet);
		adap->tasklet_conf = 0;
	}

	if (driver->detach_adapter) {
		if (driver->detach_adapter(adap))
			pr_err("detach_adapter failed for driver [%s]\n",
				driver->name);
	}

	driver->adapter = NULL;
	return res;
}

/* TDM adapter Registration/De-registration with TDM framework */

static int tdm_register_adapter(struct tdm_adapter *adap)
{
	int res = TDM_E_OK;
	struct tdm_driver *driver, *next;

	if (!adap) {
		pr_err("%s:Invalid handle\n", __func__);
		return -EINVAL;
	}

	mutex_init(&adap->adap_lock);
	INIT_LIST_HEAD(&adap->myports);
	spin_lock_init(&adap->portlist_lock);

	adap->drv_count = 0;
	adap->tasklet_conf = 0;

	list_add_tail(&adap->list, &adapter_list);

	/* initialization of driver by framework in default configuration */
	init_config_adapter(adap);

	/* Notify drivers */
	pr_info("adapter [%s] registered\n", adap->name);
	mutex_lock(&tdm_core_lock);
	list_for_each_entry_safe(driver, next, &driver_list, list) {
		if (tdm_device_match(driver, adap)) {
			res = tdm_attach_driver_adap(driver, adap);
			if (res == TDM_E_OK) {
				pr_info("Driver(ID=%d) is "
				"attached with Adapter %s(ID = %d)\n",
				driver->id, adap->name, adap->id);
			} else {
				pr_err("Driver(ID=%d) is unable "
				"to attach with Adapter %s(ID = %d)\n",
				driver->id, adap->name, adap->id);
			}
		}
	}
	mutex_unlock(&tdm_core_lock);

	return res;
}

/*
 * tdm_add_adapter - declare tdm adapter, use dynamic device number
 * @adapter: the adapter to add
 * Context: can sleep
 *
 * This routine is used to declare a TDM adapter
 * When this returns zero, a new device number will be allocated and stored
 * in adap->id, and the specified adapter became available for the clients.
 * Otherwise, a negative errno value is returned.
 */
int tdm_add_adapter(struct tdm_adapter *adapter)
{
	int id, res = TDM_E_OK;
	if (!adapter) {
		pr_err("%s:Invalid handle\n", __func__);
		return -EINVAL;
	}

retry:
	if (idr_pre_get(&tdm_adapter_idr, GFP_KERNEL) == 0)
		return -ENOMEM;

	mutex_lock(&tdm_core_lock);
	res = idr_get_new(&tdm_adapter_idr, adapter, &id);
	mutex_unlock(&tdm_core_lock);

	if (res < 0) {
		if (res == -EAGAIN)
			goto retry;
		return res;
	}

	adapter->id = id;
	return tdm_register_adapter(adapter);
}
EXPORT_SYMBOL(tdm_add_adapter);


/**
 * tdm_del_adapter - unregister TDM adapter
 * @adap: the adapter being unregistered
 *
 * This unregisters an TDM adapter which was previously registered
 * by @tdm_add_adapter.
 */
int tdm_del_adapter(struct tdm_adapter *adap)
{
	int res = TDM_E_OK;
	struct tdm_adapter *found;
	struct tdm_driver *driver, *next;

	if (!adap) {
		pr_err("%s:Invalid handle\n", __func__);
		return -EINVAL;
	}

	/* First make sure that this adapter was ever added */
	mutex_lock(&tdm_core_lock);
	found = idr_find(&tdm_adapter_idr, adap->id);
	mutex_unlock(&tdm_core_lock);
	if (found != adap) {
		pr_err("tdm-core: attempting to delete unregistered "
			 "adapter [%s]\n", adap->name);
		return -EINVAL;
	}

	/*disable and kill the data processing tasklet */
	if (adap->tasklet_conf) {
		tasklet_disable(&adap->tdm_data_tasklet);
		tasklet_kill(&adap->tdm_data_tasklet);
		adap->tasklet_conf = 0;
	}

	/* Detach any active ports. This can't fail, thus we do not
	   checking the returned value. */
	mutex_lock(&tdm_core_lock);
	list_for_each_entry_safe(driver, next, &driver_list, list) {
		if (tdm_device_match(driver, adap)) {
			tdm_detach_driver_adap(driver, adap);
			pr_info(
			"Driver(ID=%d) is detached from Adapter %s(ID = %d)\n",
				 driver->id, adap->name, adap->id);
		}
	}
	mutex_unlock(&tdm_core_lock);

	mutex_lock(&tdm_core_lock);
	idr_remove(&tdm_adapter_idr, adap->id);
	mutex_unlock(&tdm_core_lock);

	pr_debug("adapter [%s] unregistered\n", adap->name);

	list_del(&adap->list);
	/* Clear the device structure in case this adapter is ever going to be
	   added again */
	adap->parent = NULL;

	return res;
}
EXPORT_SYMBOL(tdm_del_adapter);

/* TDM Client Drivers Registration/De-registration Functions */
int tdm_register_driver(struct tdm_driver *driver)
{
	int res = TDM_E_OK;
	struct tdm_adapter *adap, *next;

	list_add_tail(&driver->list, &driver_list);

	mutex_lock(&tdm_core_lock);
	/* Walk the adapters that are already present */
	list_for_each_entry_safe(adap, next, &adapter_list, list) {
		if (tdm_device_match(driver, adap)) {
			res = tdm_attach_driver_adap(driver, adap);
			if (res == TDM_E_OK) {
				pr_info("TDM Driver(ID=%d)is attached with "
					"Adapter%s(ID = %d) drv_count=%d",
					driver->id, adap->name, adap->id,
					adap->drv_count);
			} else {
				pr_err("TDM Driver(ID=%d) unable to attach "
					"to Adapter%s(ID = %d) drv_count=%d",
					driver->id, adap->name, adap->id,
					adap->drv_count);
			}
		break;
		}
	}
	mutex_unlock(&tdm_core_lock);

	return res;
}
EXPORT_SYMBOL(tdm_register_driver);

/*
 * tdm_unregister_driver - unregister TDM client driver from TDM framework
 * @driver: the driver being unregistered
 */
void tdm_unregister_driver(struct tdm_driver *driver)
{
	if (!driver) {
		pr_err("%s:Invalid handle\n", __func__);
		return;
	}
       /* A driver can register to only one adapter,
	* so no need to browse the list */
	mutex_lock(&tdm_core_lock);
	tdm_detach_driver_adap(driver, driver->adapter);
	mutex_unlock(&tdm_core_lock);

	list_del(&driver->list);

	pr_debug("tdm-core: driver [%s] unregistered\n", driver->name);
}
EXPORT_SYMBOL(tdm_unregister_driver);

/* TDM Framework init and exit */
static int __init tdm_init(void)
{
	pr_info("%s\n", __func__);
	return TDM_E_OK;
}

static void __exit tdm_exit(void)
{
	pr_info("%s\n", __func__);
	return;
}

/* We must initialize early, because some subsystems register tdm drivers
 * in subsys_initcall() code, but are linked (and initialized) before tdm.
 */
postcore_initcall(tdm_init);
module_exit(tdm_exit);


/* Interface to the tdm device/adapter */

/* tdm_read_direct - issue a TDM read
 * @adap: Handle to TDM device
 * @buf: Data that will be read from the TDM device
 * @len: How many bytes to read
 *
 * Returns negative errno, or else 0.
 */
int tdm_read_direct(struct tdm_adapter *adap, u8 *buf, u32 len)
{
	int res;

	if (adap->algo->tdm_read_simple)
		res = adap->algo->tdm_read_simple(adap, buf, len);
	else {
		pr_err("TDM level read not supported\n");
		return -EOPNOTSUPP;
	}
	/* If everything went ok (i.e. frame received), return #bytes
	transmitted, else error code. */

	return res;


}
EXPORT_SYMBOL(tdm_read_direct);

/* tdm_write_direct - issue a TDM write
 * @adap: Handle to TDM device
 * @buf: Data that will be written to the TDM device
 * @len: How many bytes to write
 *
 * Returns negative errno, or else 0.
 */
int tdm_write_direct(struct tdm_adapter *adap, u8 *buf, u32 len)
{
	int res;

	if (adap->algo->tdm_write_simple)
		res = adap->algo->tdm_write_simple(adap, buf, len);
	else {
		pr_err("TDM level write not supported\n");
		return -EOPNOTSUPP;
	}

	return res;
}
EXPORT_SYMBOL(tdm_write_direct);

/* tdm_adap_send - issue a TDM write
 * @adap: Handle to TDM device
 * @buf: Data that will be written to the TDM device
 * @count: How many bytes to write
 *
 * Returns negative errno, or else the number of bytes written.
 */
int tdm_adap_send(struct tdm_adapter *adap, void **buf, int count)
{
	int res;

	if ((adap == NULL) || (buf == NULL)) { /* invalid handle*/
		pr_err("%s: Invalid Handle\n", __func__);
		return -ENXIO;
	}

	if (adap->algo->tdm_write)
		res = adap->algo->tdm_write(adap, buf, count);
	else {
		pr_err("TDM level write not supported\n");
		return -EOPNOTSUPP;
	}

	/* If everything went ok (i.e. frame transmitted), return #bytes
	   transmitted, else error code. */
	return (res == 1) ? count : res;
}
EXPORT_SYMBOL(tdm_adap_send);

/**
 * tdm_adap_recv - issue a TDM read
 * @adap: Handle to TDM device
 * @buf: Where to store data read from TDM device
 *
 * Returns negative errno, or else the number of bytes read.
 */
int tdm_adap_recv(struct tdm_adapter *adap, void **buf)
{
	int res;

	if (adap->algo->tdm_read)
		res = adap->algo->tdm_read(adap, (u16 **)buf);
	else {
		pr_err("TDM level read not supported\n");
		return -EOPNOTSUPP;
	}
	/* If everything went ok (i.e. frame received), return #bytes
	   transmitted, else error code. */
	return res;
}

/**
 * tdm_adap_get_write_buf - get next write TDM device buffer
 * @adap: Handle to TDM device
 * @buf: pointer to TDM device buffer
 *
 * Returns negative errno, or else size of the write buffer.
 */
int tdm_adap_get_write_buf(struct tdm_adapter *adap, void **buf)
{
	int res;

	if (adap->algo->tdm_get_write_buf) {
		res = adap->algo->tdm_get_write_buf(adap, (u16 **)buf);
	} else {
		pr_err("TDM level write buf get not supported\n");
		return -EOPNOTSUPP;
	}
	/* If everything went ok (i.e. 1 msg received), return #bytes
	   transmitted, else error code. */
	return res;
}
EXPORT_SYMBOL(tdm_adap_get_write_buf);

int tdm_adap_enable(struct tdm_driver *drv)
{
	int res;
	struct tdm_adapter *adap;
	if (drv == NULL) { /* invalid handle*/
		pr_err("%s: Invalid Handle\n", __func__);
		return -ENXIO;
	}
	adap = drv->adapter;

	if (adap->algo->tdm_enable) {
		res = adap->algo->tdm_enable(adap);
	} else {
		pr_err("TDM level enable not supported\n");
		return -EOPNOTSUPP;
	}
	return res;
}
EXPORT_SYMBOL(tdm_adap_enable);

int tdm_adap_disable(struct tdm_driver *drv)
{
	int res;
	struct tdm_adapter *adap;
	if (drv == NULL) { /* invalid handle*/
		pr_err("%s: Invalid Handle\n", __func__);
		return -ENXIO;
	}
	adap = drv->adapter;

	if (adap->algo->tdm_disable) {
		res = adap->algo->tdm_disable(adap);
	} else {
		pr_err("TDM level enable not supported\n");
		return -EOPNOTSUPP;
	}
	return res;
}
EXPORT_SYMBOL(tdm_adap_disable);

struct tdm_adapter *tdm_get_adapter(int id)
{
	struct tdm_adapter *adapter;

	mutex_lock(&tdm_core_lock);
	adapter = idr_find(&tdm_adapter_idr, id);
	if (adapter && !try_module_get(adapter->owner))
		adapter = NULL;

	mutex_unlock(&tdm_core_lock);

	return adapter;
}
EXPORT_SYMBOL(tdm_get_adapter);

void tdm_put_adapter(struct tdm_adapter *adap)
{
	module_put(adap->owner);
}
EXPORT_SYMBOL(tdm_put_adapter);


/* Port Level APIs of TDM Framework */
unsigned int tdm_port_open(struct tdm_driver *driver, void **h_port)
{
	struct tdm_port *port;
	struct tdm_adapter *adap;
	unsigned long		flags;
	int res = TDM_E_OK;

	if (driver == NULL) {
		pr_err("driver NULL\n");
		return -ENODEV;
	}
	if (driver->adapter == NULL) {
		pr_err("adapter NULL\n");
		return -ENODEV;
	}

	adap = tdm_get_adapter(driver->adapter->id);
	if (!adap)
		return -ENODEV;

	/* This creates an anonymous tdm_port, which may later be
	 * pointed to some slot.
	 *
	 */
	port = kzalloc(sizeof(*port), GFP_KERNEL);
	if (!port) {
		res = -ENOMEM;
		goto out;
	}

	port->rx_max_frames = NUM_SAMPLES_PER_FRAME;
	port->port_cfg.port_mode = e_TDM_PORT_CHANNELIZED;

	port->in_use = 1;

	snprintf(driver->name, TDM_NAME_SIZE, "tdm-dev");
	port->driver = driver;
	port->adapter = adap;

	spin_lock_irqsave(&adap->portlist_lock, flags);
	list_add_tail(&port->list, &adap->myports);
	spin_unlock_irqrestore(&adap->portlist_lock, flags);

	INIT_LIST_HEAD(&port->mychannels);

	*h_port = port;

out:
	return res;
}
EXPORT_SYMBOL(tdm_port_open);

unsigned int tdm_port_close(void *h_port)
{
	struct tdm_adapter *adap;
	struct tdm_driver *driver;
	struct tdm_port *port;
	struct tdm_channel *temp, *channel;
	unsigned long		flags;
	int res = TDM_E_OK;
	port = (struct tdm_port *)h_port;

	if (port == NULL) { /* invalid handle*/
		pr_err("Invalid Handle");
		return -ENXIO;
	}

	driver =  port->driver;

	if (driver == NULL) {
		pr_err("driver NULL\n");
		res = -ENODEV;
		goto out;
	}
	if (driver->adapter == NULL) {
		pr_err("adapter NULL\n");
		res = -ENODEV;
		goto out;
	}

	list_for_each_entry_safe(channel, temp, &port->mychannels, list) {
	if (channel)
		if (channel->in_use) {
			pr_err("%s: Cannot close port. Channel in use\n",
								__func__);
			res = -ENXIO;
			goto out;
			}
	}
	adap = driver->adapter;

	spin_lock_irqsave(&adap->portlist_lock, flags);
	list_del(&port->list);
	spin_unlock_irqrestore(&adap->portlist_lock, flags);

	if (port->p_port_data != NULL) {
		int i;
		struct tdm_bd *ch_bd;

		/* If the tdm is in channelised mode,
		de-allocate the channelised buffer */
		ch_bd = &(port->p_port_data->rx_data_fifo[0]);
		for (i = 0; ch_bd && i < TDM_CH_RX_BD_RING_SIZE; i++) {
			ch_bd->flag = 0;
			ch_bd++;
		}
		ch_bd = &(port->p_port_data->tx_data_fifo[0]);
		for (i = 0; ch_bd && i < TDM_CH_TX_BD_RING_SIZE; i++) {
			ch_bd->flag = 0;
			ch_bd++;
		}
		kfree(port->p_port_data);
	}
	kfree(port);
	return res;
out:
	if (port)
		kfree(port->p_port_data);
	kfree(port);
	return res;
}
EXPORT_SYMBOL(tdm_port_close);

unsigned int tdm_channel_read(void *h_port, void *h_channel,
				void *p_data, u16 *size)
{
	struct tdm_port *port;
	struct tdm_channel *channel;
	struct tdm_bd *rx_bd;
	unsigned long flags;
	int i, res = TDM_E_OK;
	unsigned short *buf, *buf1;
	port = (struct tdm_port *)h_port;
	channel = (struct tdm_channel *)h_channel;

	if ((port && channel) == 0) { /* invalid handle*/
		pr_err("%s:Invalid Handle\n", __func__);
		return -ENXIO;
	}

	if (!port->in_use)
		return -EIO;
	if (!channel->p_ch_data || !channel->in_use)
		return -EIO;

	spin_lock_irqsave(&channel->p_ch_data->rx_channel_lock, flags);
	rx_bd = channel->p_ch_data->rx_out_data;

	if (rx_bd->flag) {
		*size = rx_bd->length;
		buf = (u16 *) p_data;
		buf1 = (u16 *)rx_bd->p_data;
		for (i = 0; i < NUM_SAMPLES_PER_FRAME; i++)
			buf[i] = buf1[i];
		rx_bd->flag = 0;
		rx_bd->offset = 0;
		channel->p_ch_data->rx_out_data = (rx_bd->wrap) ?
				channel->p_ch_data->rx_data_fifo : rx_bd + 1;

	} else {
		spin_unlock_irqrestore(&channel->p_ch_data->rx_channel_lock,
						flags);
		pr_info("No Data Available");
		return -EAGAIN;
	}
	spin_unlock_irqrestore(&channel->p_ch_data->rx_channel_lock, flags);

	return res;
}
EXPORT_SYMBOL(tdm_channel_read);


unsigned int tdm_channel_write(void *h_port, void *h_channel,
				void *p_data, u16 size)
{
	struct tdm_port *port;
	struct tdm_channel *channel;
	struct tdm_bd *tx_bd;
	unsigned long flags;
	int err = TDM_E_OK;
	port = (struct tdm_port *)h_port;
	channel = (struct tdm_channel *)h_channel;
#ifdef TDM_CORE_DEBUG
	bool data_flag = 0;
#endif

	if ((port && channel) == 0) { /* invalid handle*/
		pr_err("Invalid Handle");
		return -ENXIO;
	}

	if (p_data == NULL) { /* invalid data*/
		pr_err("Invalid Data");
		return -EFAULT;
	}

	if (!port->in_use)
		return -EIO;
	if (!channel->p_ch_data || !channel->in_use)
		return -EIO;

	spin_lock_irqsave(&channel->p_ch_data->tx_channel_lock, flags);
	tx_bd = channel->p_ch_data->tx_in_data;

	if (!tx_bd->flag) {
		tx_bd->length = size;
		memcpy(tx_bd->p_data, p_data,
			size * port->adapter->adapt_cfg.slot_width);
		tx_bd->flag = 1;
		tx_bd->offset = 0;
		channel->p_ch_data->tx_in_data = (tx_bd->wrap) ?
				channel->p_ch_data->tx_data_fifo : tx_bd+1;
		port->port_stat.tx_pkt_count++;
#ifdef TDM_CORE_DEBUG
		data_flag = 1;
#endif
	} else {
		spin_unlock_irqrestore(&channel->p_ch_data->tx_channel_lock,
						flags);
		port->port_stat.tx_pkt_drop_count++;
		pr_err("E_NO_MEMORY -Failed Transmit");
		return -ENOMEM;
	}
	spin_unlock_irqrestore(&channel->p_ch_data->tx_channel_lock, flags);

#ifdef	TDM_CORE_DEBUG
	if (data_flag) {
		int k;
		pr_info("\nTX port:%d - Write - Port TX-%d\n",
						port->port_id, size);
		for (k = 0; k < size; k++)
			pr_info("%x", p_data[k]);
		pr_info("\n");
	}
#endif
	return err;
}
EXPORT_SYMBOL(tdm_channel_write);

/* Driver Function for select and poll. Based on Channel, it sleeps on
 * waitqueue */
unsigned int tdm_ch_poll(void *h_channel, unsigned int wait_time)
{
	struct tdm_channel *channel;
	unsigned long timeout = msecs_to_jiffies(wait_time);
	channel = h_channel;

	if (!channel->p_ch_data || !channel->in_use)
		return -EIO;

	if (channel->p_ch_data->rx_out_data->flag) {
		pr_debug("Data Available");
		return TDM_E_OK;
	}
	if (timeout) {
		wait_event_interruptible_timeout(channel->ch_wait_queue,
					  channel->p_ch_data->rx_out_data->flag,
					  timeout);

		if (channel->p_ch_data->rx_out_data->flag) {
			pr_debug("Data Available");
			return TDM_E_OK;
		}
	}
	return -EAGAIN;
}
EXPORT_SYMBOL(tdm_ch_poll);

unsigned int tdm_port_get_stats(void *h_port, struct tdm_port_stats *portStat)
{
	struct tdm_port *port;
	int port_num;
	port = (struct tdm_port *)h_port;

	if (port == NULL || portStat == NULL) { /* invalid handle*/
		pr_err("Invalid Handle");
		return -ENXIO;
	}
	port_num =  port->port_id;

	memcpy(portStat, &port->port_stat, sizeof(struct tdm_port_stats));

	pr_info("TDM Port %d Get Stats", port_num);

	return TDM_E_OK;
}
EXPORT_SYMBOL(tdm_port_get_stats);

/* Data handling functions */

static int tdm_data_rx_deinterleave(struct tdm_adapter *adap)
{
	struct tdm_port *port, *next;
	struct tdm_channel *channel, *temp;
	struct tdm_bd	*ch_bd;

	int i, buf_size, ch_data_len;
	u16 *input_tdm_buffer;
	u16 *pcm_buffer;
	int slot_width;
	int frame_ch_data_size;
	bool ch_data;
	int bytes_in_fifo_per_frame;
	int bytes_slot_offset;

	ch_data_len = NUM_SAMPLES_PER_FRAME;
	frame_ch_data_size = NUM_SAMPLES_PER_FRAME;
	ch_data = 0;

	if (!adap) { /* invalid handle*/
		pr_err("%s: Invalid Handle\n", __func__);
		return -ENXIO;
	}

	slot_width = adap->adapt_cfg.slot_width;
	buf_size = tdm_adap_recv(adap, (void **)&input_tdm_buffer);
	if (buf_size <= 0 || !input_tdm_buffer)
		return -EINVAL;

	bytes_in_fifo_per_frame = buf_size/frame_ch_data_size;
	bytes_slot_offset = bytes_in_fifo_per_frame/slot_width;

	/* de-interleaving for all ports*/
	list_for_each_entry_safe(port, next, &adap->myports, list) {

		/* if the port is not open */
		if (!port->in_use)
			continue;

		list_for_each_entry_safe(channel, temp, &port->mychannels,
							list) {
		/* if the channel is not open */
		if (!channel->in_use || !channel->p_ch_data)
			continue;
		ch_bd = channel->p_ch_data->rx_in_data;
		spin_lock(&channel->p_ch_data->rx_channel_lock);
			/*if old data is to be discarded */
		if (use_latest_tdm_data)
			if (ch_bd->flag) {
				ch_bd->flag = 0;
				ch_bd->offset = 0;
				if (ch_bd == channel->p_ch_data->rx_out_data)
					channel->p_ch_data->rx_out_data =
						ch_bd->wrap ?
						channel->p_ch_data->rx_data_fifo
						: ch_bd+1;
					port->port_stat.rx_pkt_drop_count++;
				}
			/* if the bd is empty */
			if (!ch_bd->flag) {
				if (ch_bd->offset == 0)
					ch_bd->length = port->rx_max_frames;

				pcm_buffer = ch_bd->p_data + ch_bd->offset;
				/* De-interleaving the data */
				for (i = 0; i < ch_data_len; i++) {
					pcm_buffer[i]
					= input_tdm_buffer[i*bytes_slot_offset +
						channel->ch_id];
				}
				ch_bd->offset += ch_data_len * slot_width;

				if (ch_bd->offset >=
					(ch_bd->length - frame_ch_data_size)*
						(adap->adapt_cfg.slot_width)) {
					ch_bd->flag = 1;
					ch_bd->offset = 0;
					channel->p_ch_data->rx_in_data =
						ch_bd->wrap ?
						channel->p_ch_data->rx_data_fifo
						: ch_bd+1;
					ch_data = 1;
					wake_up_interruptible
						(&channel->ch_wait_queue);
				}
			} else {
				port->port_stat.rx_pkt_drop_count++;
			}
		spin_unlock(&channel->p_ch_data->rx_channel_lock);
		}

		if (ch_data) {
			/*	Wake up the Port Data Poll event */
#ifdef	TDM_CORE_DEBUG
			pr_info("Port RX-%d-%d\n", channel->ch_id, ch_data_len);
			for (i = 0; i < ch_data_len; i++)
				pr_info("%x", pcm_buffer[i]);
			pr_info("\n");
#endif
			port->port_stat.rx_pkt_count++;
			ch_data = 0;
		}
	}
	return TDM_E_OK;
}

static int tdm_data_tx_interleave(struct tdm_adapter *adap)
{
	struct tdm_port *port, *next;
	struct tdm_channel *channel, *temp;
	struct tdm_bd	*ch_bd;
	int i, buf_size, ch_data_len = NUM_SAMPLES_PER_FRAME;
	bool last_data = 0;
	u16 *output_tdm_buffer;
	u16 *pcm_buffer;
	int frame_ch_data_size = NUM_SAMPLES_PER_FRAME;
	int bytes_in_fifo_per_frame;
	int bytes_slot_offset;

#ifdef TDM_CORE_DEBUG
	u8	data_flag = 0;
#endif

	if (adap == NULL) { /* invalid handle*/
		pr_err("%s: Invalid Handle\n", __func__);
		return -ENXIO;
	}

	buf_size = tdm_adap_get_write_buf(adap, (void **)&output_tdm_buffer);
	if (buf_size <= 0 || !output_tdm_buffer)
		return -EINVAL;

	bytes_in_fifo_per_frame = buf_size/frame_ch_data_size;
	bytes_slot_offset = bytes_in_fifo_per_frame/adap->adapt_cfg.slot_width;


	memset(output_tdm_buffer, 0, sizeof(buf_size));

	list_for_each_entry_safe(port, next, &adap->myports, list) {

		/* check if the port is open */
		if (!port->in_use)
			continue;

		list_for_each_entry_safe(channel, temp, &port->mychannels,
								list) {
		pr_debug("TX-Tdm %d (slots-)", channel->ch_id);


		/* if the channel is open */
		if (!channel->in_use || !channel->p_ch_data)
			continue;

		spin_lock(&channel->p_ch_data->tx_channel_lock);
		if (!channel->in_use || !channel->p_ch_data)
			continue;
			ch_bd = channel->p_ch_data->tx_out_data;
			if (ch_bd->flag) {
				pcm_buffer = (u16 *)((uint8_t *)ch_bd->p_data +
						ch_bd->offset);
				/*if the buffer has less frames than required */
				if (frame_ch_data_size >=
					((ch_bd->length) - (ch_bd->offset/
						adap->adapt_cfg.slot_width))) {
					ch_data_len =
					(ch_bd->length) - (ch_bd->offset/
						adap->adapt_cfg.slot_width);
					last_data = 1;
				} else {
					ch_data_len = frame_ch_data_size;
				}
				/* Interleaving the data */
				for (i = 0; i < ch_data_len; i++) {
					/* TODO- need to be genric for any size
					   assignment*/
					output_tdm_buffer[channel->ch_id +
						bytes_slot_offset * i] =
								pcm_buffer[i];
				}
				/* If all the data of this buffer is
							transmitted */
				if (last_data) {
					ch_bd->flag = 0;
					ch_bd->offset = 0;
					channel->p_ch_data->tx_out_data =
						ch_bd->wrap ?
						channel->p_ch_data->tx_data_fifo
						: ch_bd+1;
					port->port_stat.tx_pkt_conf_count++;
				} else {
					ch_bd->offset += ch_data_len *
						(adap->adapt_cfg.slot_width);
				}
#ifdef	TDM_CORE_DEBUG
				data_flag = 1;
#endif
			}
		spin_unlock(&channel->p_ch_data->tx_channel_lock);
		}
	}

#ifdef	TDM_CORE_DEBUG
	if (data_flag) {
		pr_info("TX-TDM Interleaved Data-\n");
		for (i = 0; i < 64; i++)
			pr_info("%x", output_tdm_buffer[i]);
		pr_info("\n");
	  }
#endif
	return TDM_E_OK;
}

/* Channel Level APIs of TDM Framework */
int tdm_channel_open(u16 chanid, u16 ch_width, struct tdm_port *port,
				void **h_channel)
{
	struct tdm_channel *channel, *temp;
	unsigned long		flags;
	struct tdm_ch_data	*p_ch_data;
	int res = TDM_E_OK;

	if (!(port && h_channel)) {
		pr_err("%s: Invalid handle\n", __func__);
		return -EINVAL;
	}

	if (ch_width != 1) {
		pr_err("%s: Mode not supported\n", __func__);
		return -EINVAL;
	}

	list_for_each_entry_safe(channel, temp, &port->mychannels, list) {
		if (channel->ch_id == chanid) {
			pr_err("%s: Channel %d already open\n",
						__func__, chanid);
			return -EINVAL;
		}
	}

	channel = kzalloc(sizeof(*channel), GFP_KERNEL);
	if (!channel) {
		res = -ENOMEM;
		goto out;
	}

	init_waitqueue_head(&channel->ch_wait_queue);
	p_ch_data = kzalloc(sizeof(struct tdm_ch_data), GFP_KERNEL);
	if (!p_ch_data) {
		res = -ENOMEM;
		goto outdata;
	}

	p_ch_data->rx_data_fifo[TDM_CH_RX_BD_RING_SIZE-1].wrap = 1;
	p_ch_data->tx_data_fifo[TDM_CH_TX_BD_RING_SIZE-1].wrap = 1;

	p_ch_data->rx_in_data = p_ch_data->rx_data_fifo;
	p_ch_data->rx_out_data = p_ch_data->rx_data_fifo;
	p_ch_data->tx_in_data = p_ch_data->tx_data_fifo;
	p_ch_data->tx_out_data = p_ch_data->tx_data_fifo;
	spin_lock_init(&p_ch_data->rx_channel_lock);
	spin_lock_init(&p_ch_data->tx_channel_lock);

	channel->p_ch_data = p_ch_data;

	channel->ch_id = chanid;
	channel->ch_cfg.first_slot = chanid;
	channel->ch_cfg.num_slots = 1;	/* This is 1 for channelized mode and
						configurable for other modes */
	channel->port = port;
	channel->in_use = 1;

	spin_lock_irqsave(&port->ch_list_lock, flags);
	list_add_tail(&channel->list, &port->mychannels);
	spin_unlock_irqrestore(&port->ch_list_lock, flags);

	*h_channel = channel;

	return res;

outdata:
	kfree(channel);
out:
	return res;
}
EXPORT_SYMBOL(tdm_channel_open);

int tdm_channel_close(u16 chanid, u16 ch_width, struct tdm_port *port,
				struct tdm_channel *h_channel)
{
	struct tdm_channel *channel;
	unsigned long		flags;
	int res = TDM_E_OK;
	channel = h_channel;

	if (!(port && channel)) {
		pr_err("%s: Invalid handle\n", __func__);
		res = -EINVAL;
		goto out;
	}

	if (ch_width != 1) {
		pr_err("%s: Mode not supported\n", __func__);
		res = -EINVAL;
		goto out;
	}

	spin_lock_irqsave(&port->ch_list_lock, flags);
	list_del(&channel->list);
	spin_unlock_irqrestore(&port->ch_list_lock, flags);

out:
	if (channel)
		kfree(channel->p_ch_data);
	kfree(channel);
	return res;
}
EXPORT_SYMBOL(tdm_channel_close);

void init_config_adapter(struct tdm_adapter *adap)
{
	struct fsl_tdm_adapt_cfg default_adapt_cfg = {
		.loopback = e_TDM_PROCESS_NORMAL,
		.num_ch = NUM_CHANNELS,
		.ch_size_type = CHANNEL_16BIT_LIN,
		.frame_len = NUM_SAMPLES_PER_FRAME,
		.num_frames = NUM_SAMPLES_PER_FRAME,
		.adap_mode = e_TDM_ADAPTER_MODE_NONE
			 };

	default_adapt_cfg.slot_width = default_adapt_cfg.ch_size_type/3 + 1;

	memcpy(&adap->adapt_cfg, &default_adapt_cfg,
		sizeof(struct fsl_tdm_adapt_cfg));

	return;
}
EXPORT_SYMBOL(init_config_adapter);

static void tdm_data_tasklet_fn(unsigned long data)
{
	struct tdm_adapter *adapter;
	adapter = (struct tdm_adapter *)data;
	if (adapter != NULL) {
		tdm_data_tx_interleave(adapter);
		tdm_data_rx_deinterleave(adapter);
	}
}


MODULE_AUTHOR("Hemant Agrawal <hemant@freescale.com> and "
	"Rajesh Gumasta <rajesh.gumasta@freescale.com>");
MODULE_DESCRIPTION("TDM Driver Framework Core");
MODULE_LICENSE("GPL");
