/*
 * An hwmon driver for accton as5710_54x sfp
 *
 * Copyright (C)  Brandon Chuang <brandon_chuang@accton.com.tw>
 *
 * Based on ad7414.c
 * Copyright 2006 Stefan Roese <sr at denx.de>, DENX Software Engineering
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/slab.h>

#define NUM_OF_SFP_PORT 54
#define BIT_INDEX(i) (1ULL << (i))

#define CPLD2_I2C_SELECT_NONE  0x18
#define CPLD3_I2C_SELECT_NONE  0x1E

static ssize_t show_status(struct device *dev, struct device_attribute *da,char *buf);
static ssize_t set_tx_disable(struct device *dev, struct device_attribute *da,
			const char *buf, size_t count);
static ssize_t show_active_port(struct device *dev, struct device_attribute *da, char *buf);
static ssize_t set_active_port(struct device *dev, struct device_attribute *da,
			const char *buf, size_t count);
static ssize_t show_eeprom(struct device *dev, struct device_attribute *da, char *buf);
static int as5710_54x_sfp_read_block(struct i2c_client *client, u8 command, u8 *data,int data_len);
extern int accton_i2c_cpld_read(unsigned short cpld_addr, u8 reg);
extern int accton_i2c_cpld_write(unsigned short cpld_addr, u8 reg, u8 value);

/* Addresses scanned 
 */
static const unsigned short normal_i2c[] = { 0x50, I2C_CLIENT_END };

/* Each client has this additional data 
 */
struct as5710_54x_sfp_data {
    struct device      *hwmon_dev;
    struct mutex        update_lock;
    char                valid;           /* !=0 if registers are valid */
    unsigned long       last_updated;    /* In jiffies */
    u8                  active_port;
    char                eeprom[256];
    u64                 status[4];       /* bit0:port0, bit1:port1 and so on */
                                         /* index 0 => is_present 
                                                  1 => tx_fail 
                                                  2 => tx_disable
                                                  3 => rx_loss */
};

static struct as5710_54x_sfp_data *as5710_54x_sfp_update_device(struct device *dev);             

enum as5710_54x_sfp_sysfs_attributes {
    SFP_IS_PRESENT,
    SFP_TX_FAULT,
    SFP_TX_DISABLE,
    SFP_RX_LOSS,
    SFP_ACTIVE_PORT,
    SFP_EEPROM
};

/* sysfs attributes for hwmon 
 */
static SENSOR_DEVICE_ATTR(sfp_is_present,  S_IRUGO, show_status, NULL, SFP_IS_PRESENT);
static SENSOR_DEVICE_ATTR(sfp_tx_fault,    S_IRUGO, show_status, NULL, SFP_TX_FAULT);
static SENSOR_DEVICE_ATTR(sfp_tx_disable,  S_IWUSR | S_IRUGO, show_status, set_tx_disable, SFP_TX_DISABLE);
static SENSOR_DEVICE_ATTR(sfp_rx_loss,     S_IRUGO, show_status,NULL, SFP_RX_LOSS);
static SENSOR_DEVICE_ATTR(sfp_active_port, S_IWUSR | S_IRUGO, show_active_port, set_active_port, SFP_ACTIVE_PORT);
static SENSOR_DEVICE_ATTR(sfp_eeprom,      S_IRUGO, show_eeprom, NULL, SFP_EEPROM);

static struct attribute *as5710_54x_sfp_attributes[] = {
    &sensor_dev_attr_sfp_is_present.dev_attr.attr,
    &sensor_dev_attr_sfp_tx_fault.dev_attr.attr,
    &sensor_dev_attr_sfp_rx_loss.dev_attr.attr,
    &sensor_dev_attr_sfp_tx_disable.dev_attr.attr,
    &sensor_dev_attr_sfp_eeprom.dev_attr.attr,
    &sensor_dev_attr_sfp_active_port.dev_attr.attr,
    NULL
};

static ssize_t show_status(struct device *dev, struct device_attribute *da,
             char *buf)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct as5710_54x_sfp_data *data = as5710_54x_sfp_update_device(dev);
    u8 val;

    if (data->active_port == 0 || data->active_port > NUM_OF_SFP_PORT) {
        return sprintf(buf, "0\n");
    }

    if (attr->index == SFP_IS_PRESENT) {
        val = (data->status[attr->index] & BIT_INDEX(data->active_port-1)) ? 0 : 1;
    }
    else {
        val = (data->status[attr->index] & BIT_INDEX(data->active_port-1)) ? 1 : 0;
    }

    return sprintf(buf, "%d", val);
}

static ssize_t set_tx_disable(struct device *dev, struct device_attribute *da,
			const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct as5710_54x_sfp_data *data = i2c_get_clientdata(client);
    unsigned short cpld_addr = 0;
    u8 cpld_reg = 0, cpld_val = 0;
	long disable;
	int error;

    /* Tx disable is not supported for QSFP ports(49-54) */
    if (data->active_port == 0 || data->active_port > 48) {
        return -EINVAL;
    }

	error = kstrtol(buf, 10, &disable);
	if (error) {
        return error;
    }

    mutex_lock(&data->update_lock);
    cpld_addr = (data->active_port <= 24) ? 0x61 : 0x62;
    cpld_reg  = 0xC + ((data->active_port - 1)%24)/8;

    /* Update tx_disable status */
    if (disable) {
        data->status[SFP_TX_DISABLE] |= BIT_INDEX(data->active_port-1);
    }
    else {
        data->status[SFP_TX_DISABLE] &= ~BIT_INDEX(data->active_port-1);
    }
    
    cpld_val |= data->status[SFP_TX_DISABLE] >> (8*((data->active_port-1)/8));
    accton_i2c_cpld_write(cpld_addr, cpld_reg, cpld_val);

    mutex_unlock(&data->update_lock);

    return count;    
}

static ssize_t show_active_port(struct device *dev, struct device_attribute *da,
             char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct as5710_54x_sfp_data *data = i2c_get_clientdata(client);

    if (data->active_port == 0 || data->active_port > NUM_OF_SFP_PORT) {
        return sprintf(buf, "0");
    }

    return sprintf(buf, "%d", data->active_port);
}

static ssize_t set_active_port(struct device *dev, struct device_attribute *da,
			const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct as5710_54x_sfp_data *data = i2c_get_clientdata(client);
    unsigned short cpld_addr;
	long active_port;
	int error;
    u8 cpld_val;

	error = kstrtol(buf, 10, &active_port);
	if (error) {
        return error;
    }

    if (active_port > NUM_OF_SFP_PORT || active_port < 0) {
        return -EINVAL;
    }

    mutex_lock(&data->update_lock);
    data->valid = 0; /* invalidate the data if active port changed */
    data->active_port = active_port;

    /* Disable all sfp ports */
    accton_i2c_cpld_write(0x61, 0x2, CPLD2_I2C_SELECT_NONE);
    accton_i2c_cpld_write(0x62, 0x2, CPLD3_I2C_SELECT_NONE);

    /* Set active port to CPLD */
    if (active_port != 0) {
        cpld_addr = (active_port <= 24) ? 0x61 : 0x62;
        cpld_val  = (active_port <= 24) ? (active_port-1) : (active_port-25);
        accton_i2c_cpld_write(cpld_addr, 0x2, cpld_val);
    }
    
    mutex_unlock(&data->update_lock);

    return count;
}

static ssize_t show_eeprom(struct device *dev, struct device_attribute *da,
             char *buf)
{
    struct as5710_54x_sfp_data *data = as5710_54x_sfp_update_device(dev);

    if (data->active_port == 0 || data->active_port > NUM_OF_SFP_PORT) {
        return 0;
    }
    
    memcpy(buf, data->eeprom, sizeof(data->eeprom));    
    return sizeof(data->eeprom);
}

static const struct attribute_group as5710_54x_sfp_group = {
    .attrs = as5710_54x_sfp_attributes,
};

static int as5710_54x_sfp_probe(struct i2c_client *client,
            const struct i2c_device_id *dev_id)
{
    struct as5710_54x_sfp_data *data;
    int status;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK)) {
        status = -EIO;
        goto exit;
    }

    data = kzalloc(sizeof(struct as5710_54x_sfp_data), GFP_KERNEL);
    if (!data) {
        status = -ENOMEM;
        goto exit;
    }

    i2c_set_clientdata(client, data);
    data->valid = 0;
    data->active_port = 0;
    memset(data->eeprom, 0, sizeof(data->eeprom));
    memset(data->status, 0, sizeof(data->status));
    mutex_init(&data->update_lock);

    dev_info(&client->dev, "chip found\n");

    /* Register sysfs hooks */
    status = sysfs_create_group(&client->dev.kobj, &as5710_54x_sfp_group);
    if (status) {
        goto exit_free;
    }

    data->hwmon_dev = hwmon_device_register(&client->dev);
    if (IS_ERR(data->hwmon_dev)) {
        status = PTR_ERR(data->hwmon_dev);
        goto exit_remove;
    }

    dev_info(&client->dev, "%s: sfp '%s'\n",
         dev_name(data->hwmon_dev), client->name);
    
    return 0;

exit_remove:
    sysfs_remove_group(&client->dev.kobj, &as5710_54x_sfp_group);
exit_free:
    kfree(data);
exit:
    
    return status;
}

static int as5710_54x_sfp_remove(struct i2c_client *client)
{
    struct as5710_54x_sfp_data *data = i2c_get_clientdata(client);

    hwmon_device_unregister(data->hwmon_dev);
    sysfs_remove_group(&client->dev.kobj, &as5710_54x_sfp_group);
    kfree(data);
    
    return 0;
}

static const struct i2c_device_id as5710_54x_sfp_id[] = {
    { "acc_as5710_54x_sfp", 0 },
    {}
};
MODULE_DEVICE_TABLE(i2c, as5710_54x_sfp_id);

static struct i2c_driver as5710_54x_sfp_driver = {
    .class        = I2C_CLASS_HWMON,
    .driver = {
        .name     = "acc_as5710_54x_sfp",
    },
    .probe        = as5710_54x_sfp_probe,
    .remove       = as5710_54x_sfp_remove,
    .id_table     = as5710_54x_sfp_id,
    .address_list = normal_i2c,
};

static int as5710_54x_sfp_read_block(struct i2c_client *client, u8 command, u8 *data,
              int data_len)
{
    int result = i2c_smbus_read_i2c_block_data(client, command, data_len, data);
    
    if (unlikely(result < 0))
        goto abort;
    if (unlikely(result != data_len)) {
        result = -EIO;
        goto abort;
    }
    
    result = 0;
    
abort:
    return result;
}

static struct as5710_54x_sfp_data *as5710_54x_sfp_update_device(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct as5710_54x_sfp_data *data = i2c_get_clientdata(client);
    
    mutex_lock(&data->update_lock);

    if (time_after(jiffies, data->last_updated + HZ + HZ / 2)
        || !data->valid) {
        int status = -1;
        int i = 0, j = 0;

        //dev_dbg(&client->dev, "Starting as5710_54x sfp update\n");        
        memset(data->status, 0, sizeof(data->status));
        
        /* Read status of port 1~48(SFP port) */
        for (i = 0; i < 2; i++) {
            for (j = 0; j < 12; j++) {
                status = accton_i2c_cpld_read(0x61+i, 0x6+j);
                
                if (status < 0) {
                    dev_dbg(&client->dev, "cpld(0x%x) reg(0x%x) err %d\n", 0x61+i, 0x6+j, status);
                    continue;
                }

                data->status[j/3] |= (u64)status << ((i*24) + (j%3)*8);
            }
        }

        /* Read present status of port 49-54(QSFP port) */
        status = accton_i2c_cpld_read(0x62, 0x14);

        if (status < 0) {
            dev_dbg(&client->dev, "cpld(0x%x) reg(0x%x) err %d\n", 0x61+i, 0x6+j, status);
        }
        else {
            data->status[SFP_IS_PRESENT] |= (u64)status << 48;
        }

        /* Read eeprom data based on active port */
        memset(data->eeprom, 0, sizeof(data->eeprom));

        /* Check if the port is present */
        if ((data->status[SFP_IS_PRESENT] & BIT_INDEX(data->active_port-1)) == 0) {
            /* read eeprom */
            for (i = 0; i < sizeof(data->eeprom)/I2C_SMBUS_BLOCK_MAX; i++) {
                status = as5710_54x_sfp_read_block(client, i*I2C_SMBUS_BLOCK_MAX, 
                                                   data->eeprom+(i*I2C_SMBUS_BLOCK_MAX),
                                                   I2C_SMBUS_BLOCK_MAX);
                if (status < 0) {
                    dev_dbg(&client->dev, "unable to read eeprom from port(%d)\n", data->active_port-1);
                    break;
                }
            }
        }
        
        data->last_updated = jiffies;
        data->valid = 1;
    }

    mutex_unlock(&data->update_lock);

    return data;
}

static int __init as5710_54x_sfp_init(void)
{
    return i2c_add_driver(&as5710_54x_sfp_driver);
}

static void __exit as5710_54x_sfp_exit(void)
{
    i2c_del_driver(&as5710_54x_sfp_driver);
}

MODULE_AUTHOR("Brandon Chuang <brandon_chuang@accton.com.tw>");
MODULE_DESCRIPTION("accton as5710_54x_sfp driver");
MODULE_LICENSE("GPL");

module_init(as5710_54x_sfp_init);
module_exit(as5710_54x_sfp_exit);

