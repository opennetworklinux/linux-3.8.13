/*
 * An hwmon driver for accton as5710_54x Power Module
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

#define DEBUG_MODE 0

#if (DEBUG_MODE == 1)
    #define DEBUG_PRINT(fmt, args...)     \
        printk (KERN_INFO "%s:%s[%d]: " fmt "\r\n", __FILE__, __FUNCTION__, __LINE__, ##args)
#else
    #define DEBUG_PRINT(fmt, args...)
#endif

#define MAX_MODEL_NAME          16
#define DC12V_FAN_DIR_OFFSET    0x34
#define DC12V_FAN_DIR_LEN       3
#define IS_PRESENT(id, value)    (!((value) >> (((id) - 1) * 4) & 0x1))
#define IS_POWER_GOOD(id, value) ((value) >> (((id) - 1) * 4 + 1) & 0x1)

static ssize_t show_index(struct device *dev, struct device_attribute *da, char *buf);
static ssize_t show_status(struct device *dev, struct device_attribute *da, char *buf);
static ssize_t show_string(struct device *dev, struct device_attribute *da, char *buf);
static int as5710_54x_psu_read_block(struct i2c_client *client, u8 command, u8 *data,int data_len);
extern int accton_i2c_cpld_read(unsigned short cpld_addr, u8 reg);

/* Addresses scanned 
 */
static const unsigned short normal_i2c[] = { I2C_CLIENT_END };

/* Each client has this additional data 
 */
struct as5710_54x_psu_data {
    struct device      *hwmon_dev;
    struct mutex        update_lock;
    char                valid;           /* !=0 if registers are valid */
    unsigned long       last_updated;    /* In jiffies */
    u8  index;           /* PSU index */
    u8  status;          /* Status(present/power_good) register read from CPLD */
    char model_name[MAX_MODEL_NAME+1]; /* Model name, read from eeprom */
    char fan_dir[DC12V_FAN_DIR_LEN+1]; /* DC12V fan direction */
};

static struct as5710_54x_psu_data *as5710_54x_psu_update_device(struct device *dev);             

enum as5710_54x_psu_sysfs_attributes {
    PSU_INDEX,
    PSU_PRESENT,
    PSU_MODEL_NAME,
    PSU_POWER_GOOD,
    PSU_FAN_DIR /* For DC12V only */
};

/* sysfs attributes for hwmon 
 */
static SENSOR_DEVICE_ATTR(psu_index,      S_IRUGO, show_index,     NULL, PSU_INDEX);
static SENSOR_DEVICE_ATTR(psu_present,    S_IRUGO, show_status,    NULL, PSU_PRESENT);
static SENSOR_DEVICE_ATTR(psu_model_name, S_IRUGO, show_string,    NULL, PSU_MODEL_NAME);
static SENSOR_DEVICE_ATTR(psu_power_good, S_IRUGO, show_status,    NULL, PSU_POWER_GOOD);
static SENSOR_DEVICE_ATTR(psu_fan_dir,    S_IRUGO, show_string,    NULL, PSU_FAN_DIR);

static struct attribute *as5710_54x_psu_attributes[] = {
    &sensor_dev_attr_psu_index.dev_attr.attr,
    &sensor_dev_attr_psu_present.dev_attr.attr,
    &sensor_dev_attr_psu_model_name.dev_attr.attr,
    &sensor_dev_attr_psu_power_good.dev_attr.attr,
    &sensor_dev_attr_psu_fan_dir.dev_attr.attr,
    NULL
};

static ssize_t show_index(struct device *dev, struct device_attribute *da,
             char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct as5710_54x_psu_data *data = i2c_get_clientdata(client);
    
    return sprintf(buf, "%d\n", data->index);
}

static ssize_t show_status(struct device *dev, struct device_attribute *da,
             char *buf)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct as5710_54x_psu_data *data = as5710_54x_psu_update_device(dev);
    u8 status = 0;

    if (!data->valid) {
        return -EIO;
    }

    if (attr->index == PSU_PRESENT) {
        status = IS_PRESENT(data->index, data->status);
    }
    else { /* PSU_POWER_GOOD */
        status = IS_POWER_GOOD(data->index, data->status);
    }

    return sprintf(buf, "%d\n", status);
}

static ssize_t show_string(struct device *dev, struct device_attribute *da,
             char *buf)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct as5710_54x_psu_data *data = as5710_54x_psu_update_device(dev);
    char *ptr = NULL;

    if (!data->valid) {
        return -EIO;
    }

    if (attr->index == PSU_MODEL_NAME) {
        ptr = data->model_name;
    }
    else { /* PSU_FAN_DIR */
        ptr = data->fan_dir;
    }

    return sprintf(buf, "%s\n", ptr);
}

static const struct attribute_group as5710_54x_psu_group = {
    .attrs = as5710_54x_psu_attributes,
};

static int as5710_54x_psu_probe(struct i2c_client *client,
            const struct i2c_device_id *dev_id)
{
    struct as5710_54x_psu_data *data;
    int status;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK)) {
        status = -EIO;
        goto exit;
    }

    data = kzalloc(sizeof(struct as5710_54x_psu_data), GFP_KERNEL);
    if (!data) {
        status = -ENOMEM;
        goto exit;
    }

    i2c_set_clientdata(client, data);
    data->valid = 0;
    mutex_init(&data->update_lock);

    dev_info(&client->dev, "chip found\n");

    /* Register sysfs hooks */
    status = sysfs_create_group(&client->dev.kobj, &as5710_54x_psu_group);
    if (status) {
        goto exit_free;
    }

    data->hwmon_dev = hwmon_device_register(&client->dev);
    if (IS_ERR(data->hwmon_dev)) {
        status = PTR_ERR(data->hwmon_dev);
        goto exit_remove;
    }

    /* Update PSU index */
    if (client->addr == 0x38 || client->addr == 0x50) {
        data->index = 1;
    }
    else if (client->addr == 0x3b || client->addr == 0x53) {
        data->index = 2;
    }

    dev_info(&client->dev, "%s: psu '%s'\n",
         dev_name(data->hwmon_dev), client->name);
    
    return 0;

exit_remove:
    sysfs_remove_group(&client->dev.kobj, &as5710_54x_psu_group);
exit_free:
    kfree(data);
exit:
    
    return status;
}

static int as5710_54x_psu_remove(struct i2c_client *client)
{
    struct as5710_54x_psu_data *data = i2c_get_clientdata(client);

    hwmon_device_unregister(data->hwmon_dev);
    sysfs_remove_group(&client->dev.kobj, &as5710_54x_psu_group);
    kfree(data);
    
    return 0;
}

static const struct i2c_device_id as5710_54x_psu_id[] = {
    { "acc_as5710_54x_psu", 0 },
    {}
};
MODULE_DEVICE_TABLE(i2c, as5710_54x_psu_id);

static struct i2c_driver as5710_54x_psu_driver = {
    .class        = I2C_CLASS_HWMON,
    .driver = {
        .name     = "acc_as5710_54x_psu",
    },
    .probe        = as5710_54x_psu_probe,
    .remove       = as5710_54x_psu_remove,
    .id_table     = as5710_54x_psu_id,
    .address_list = normal_i2c,
};

static int as5710_54x_psu_read_block(struct i2c_client *client, u8 command, u8 *data,
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

enum psu_type {
    PSU_TYPE_AC_110V_F2B,
    PSU_TYPE_AC_110V_B2F,
    PSU_TYPE_DC_48V_F2B,
    PSU_TYPE_DC_48V_B2F,
    PSU_TYPE_DC_12V_NON,
};

struct model_name_info {
    enum psu_type type;
    u8 offset;
    u8 length;
    char* model_name;
};

struct model_name_info models[] = {
{PSU_TYPE_AC_110V_F2B, 0x26, 13, "CPR-4011-4M11"},
{PSU_TYPE_AC_110V_B2F, 0x26, 13, "CPR-4011-4M21"},
{PSU_TYPE_DC_48V_F2B, 0x50,  12, "um400d01-01G"},
{PSU_TYPE_DC_48V_B2F, 0x50,   9, "um400d01G"},
{PSU_TYPE_DC_12V_NON, 0x00,  11, "PSU-12V-650"},
};

static int as5710_54x_psu_model_name_get(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct as5710_54x_psu_data *data = i2c_get_clientdata(client);
    int i, status;

    for (i = 0; i < ARRAY_SIZE(models); i++) {
        memset(data->model_name, 0, sizeof(data->model_name));

        status = as5710_54x_psu_read_block(client, models[i].offset,
                                           data->model_name, models[i].length);
        if (status < 0) {
            data->model_name[0] = '\0';
            dev_dbg(&client->dev, "unable to read model name from (0x%x) offset(0x%x)\n", 
                                  client->addr, models[i].offset);
            continue;
        }
        else {
            data->model_name[models[i].length] = '\0';
        }

        /* Determine if the model name is known, if not, read next index
         */
        if (strncmp(data->model_name, models[i].model_name, models[i].length) == 0) {
            return 0;
        }
        else {
            data->model_name[0] = '\0';
        }
    }

    return -ENODATA;
}

static struct as5710_54x_psu_data *as5710_54x_psu_update_device(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct as5710_54x_psu_data *data = i2c_get_clientdata(client);
    
    mutex_lock(&data->update_lock);

    if (time_after(jiffies, data->last_updated + HZ + HZ / 2)
        || !data->valid) {
        int status = -1;

        data->valid = 0;
        dev_dbg(&client->dev, "Starting as5710_54x update\n");

        /* Read psu status */
        status = accton_i2c_cpld_read(0x60, 0x2);
        
        if (status < 0) {
            dev_dbg(&client->dev, "cpld reg 0x60 err %d\n", status);
			goto exit;
        }
        else {
            data->status = status;
        }

        if (IS_PRESENT(data->index, status)) {
            if (as5710_54x_psu_model_name_get(dev) < 0) {
				data->model_name[0] = '\0';
            }

            if (strncmp(data->model_name, 
                        models[PSU_TYPE_DC_12V_NON].model_name,
                        models[PSU_TYPE_DC_12V_NON].length) == 0) {
                /* Read dc12v fan direction */
                status = as5710_54x_psu_read_block(client, DC12V_FAN_DIR_OFFSET, 
                                                   data->fan_dir, DC12V_FAN_DIR_LEN);

                if (status < 0) {
                    data->fan_dir[0] = '\0';
                    dev_dbg(&client->dev, "unable to read fan direction from (0x%x) offset(0x%x)\n", 
                                          client->addr, DC12V_FAN_DIR_OFFSET);
                    goto exit;
                }
            }
        }
        
        data->last_updated = jiffies;
        data->valid = 1;
    }

exit:
    mutex_unlock(&data->update_lock);

    return data;
}

static int __init as5710_54x_psu_init(void)
{
    return i2c_add_driver(&as5710_54x_psu_driver);
}

static void __exit as5710_54x_psu_exit(void)
{
    i2c_del_driver(&as5710_54x_psu_driver);
}

MODULE_AUTHOR("Brandon Chuang <brandon_chuang@accton.com.tw>");
MODULE_DESCRIPTION("accton as5710_54x_psu driver");
MODULE_LICENSE("GPL");

module_init(as5710_54x_psu_init);
module_exit(as5710_54x_psu_exit);

