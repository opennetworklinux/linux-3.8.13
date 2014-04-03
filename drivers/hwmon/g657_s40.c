/*
 * An hwmon driver for the EFRP-G657-S40 Redundant Power Module
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

/* Addresses scanned 
 */
static const unsigned short normal_i2c[] = { 0x58, 0x5b, I2C_CLIENT_END };

/* Each client has this additional data 
 */
struct g657_s40_data {
    struct device      *hwmon_dev;
    struct mutex        update_lock;
    char                valid;           /* !=0 if registers are valid */
    unsigned long       last_updated;    /* In jiffies */
    u8   capability;     /* Register value */    
    u8   vout_mode;      /* Register value */    
    u16  status_word;    /* Register value */
    u8   fan_fault;      /* Register value */
    u8   over_temp;      /* Register value */
    u16  v_in;           /* Register value */
    u16  v_out;          /* Register value */
    u16  i_in;           /* Register value */
    u16  i_out;          /* Register value */
    u16  p_in;           /* Register value */
    u16  p_out;          /* Register value */
    u16  temp[2];        /* Register value */
    u16  fan_speed[2];   /* Register value */
    u8   pmbus_revision; /* Register value */
    u8   mfr_id[8];      /* Register value */
    u8   mfr_model[13];  /* Register value */
    u8   mfr_revsion[4]; /* Register value */
    u8   mfr_serial[14]; /* Register value */
    u16  mfr_vin_min;    /* Register value */
    u16  mfr_vin_max;    /* Register value */
    u16  mfr_pout_max;   /* Register value */
};

static ssize_t show_byte(struct device *dev, struct device_attribute *da,
             char *buf);
static ssize_t show_word(struct device *dev, struct device_attribute *da,
             char *buf);
static ssize_t show_linear(struct device *dev, struct device_attribute *da,
             char *buf);
static ssize_t show_fan_fault(struct device *dev, struct device_attribute *da,
             char *buf);
static ssize_t show_over_temp(struct device *dev, struct device_attribute *da,
             char *buf);
static ssize_t show_ascii(struct device *dev, struct device_attribute *da,
             char *buf);    
static ssize_t show_vout(struct device *dev, struct device_attribute *da,
             char *buf);            
static struct g657_s40_data *g657_s40_update_device(struct device *dev);             

enum g657_s40_sysfs_attributes {
    PSU_POWER_ON = 0,
    PSU_TEMP_FAULT,
    PSU_POWER_GOOD,
    PSU_FAN1_FAULT,
    PSU_FAN2_FAULT,
    PSU_OVER_TEMP,
    PSU_V_IN,
    PSU_V_OUT,
    PSU_I_IN,
    PSU_I_OUT,
    PSU_P_IN,
    PSU_P_OUT,
    PSU_TEMP1,
    PSU_TEMP2,
    PSU_FAN_SPEED1,
    PSU_FAN_SPEED2,
    PSU_PMBUS_REVISION,
    PSU_MFR_ID,
    PSU_MFR_MODEL,
    PSU_MFR_REVISION,
    PSU_MFR_SERIAL,
    PSU_MFR_VIN_MIN,
    PSU_MFR_VIN_MAX,
    PSU_MFR_POUT_MAX
};

/* sysfs attributes for hwmon 
 */
static SENSOR_DEVICE_ATTR(psu_power_on,    S_IRUGO, show_word,      NULL, PSU_POWER_ON);
static SENSOR_DEVICE_ATTR(psu_temp_fault,  S_IRUGO, show_word,      NULL, PSU_TEMP_FAULT);
static SENSOR_DEVICE_ATTR(psu_power_good,  S_IRUGO, show_word,      NULL, PSU_POWER_GOOD);
static SENSOR_DEVICE_ATTR(psu_fan1_fault,  S_IRUGO, show_fan_fault, NULL, PSU_FAN1_FAULT);
static SENSOR_DEVICE_ATTR(psu_fan2_fault,  S_IRUGO, show_fan_fault, NULL, PSU_FAN2_FAULT);
static SENSOR_DEVICE_ATTR(psu_over_temp,   S_IRUGO, show_over_temp, NULL, PSU_OVER_TEMP);
static SENSOR_DEVICE_ATTR(psu_v_in,        S_IRUGO, show_linear,    NULL, PSU_V_IN);
static SENSOR_DEVICE_ATTR(psu_v_out,       S_IRUGO, show_vout,      NULL, PSU_V_OUT);
static SENSOR_DEVICE_ATTR(psu_i_in,        S_IRUGO, show_linear,    NULL, PSU_I_IN);
static SENSOR_DEVICE_ATTR(psu_i_out,       S_IRUGO, show_linear,    NULL, PSU_I_OUT);
static SENSOR_DEVICE_ATTR(psu_p_in,        S_IRUGO, show_linear,    NULL, PSU_P_IN);
static SENSOR_DEVICE_ATTR(psu_p_out,       S_IRUGO, show_linear,    NULL, PSU_P_OUT);
static SENSOR_DEVICE_ATTR(psu_temp1,       S_IRUGO, show_linear,    NULL, PSU_TEMP1);
static SENSOR_DEVICE_ATTR(psu_temp2,       S_IRUGO, show_linear,    NULL, PSU_TEMP2);
static SENSOR_DEVICE_ATTR(psu_fan_speed1,  S_IRUGO, show_linear,    NULL, PSU_FAN_SPEED1);
static SENSOR_DEVICE_ATTR(psu_fan_speed2,  S_IRUGO, show_linear,    NULL, PSU_FAN_SPEED2);
static SENSOR_DEVICE_ATTR(psu_pmbus_revision, S_IRUGO, show_byte,   NULL, PSU_PMBUS_REVISION);
static SENSOR_DEVICE_ATTR(psu_mfr_id,         S_IRUGO, show_ascii,  NULL, PSU_MFR_ID);
static SENSOR_DEVICE_ATTR(psu_mfr_model,      S_IRUGO, show_ascii,  NULL, PSU_MFR_MODEL);
static SENSOR_DEVICE_ATTR(psu_mfr_revision,    S_IRUGO, show_ascii, NULL, PSU_MFR_REVISION);
static SENSOR_DEVICE_ATTR(psu_mfr_serial,     S_IRUGO, show_ascii,  NULL, PSU_MFR_SERIAL);
static SENSOR_DEVICE_ATTR(psu_mfr_vin_min,    S_IRUGO, show_linear, NULL, PSU_MFR_VIN_MIN);
static SENSOR_DEVICE_ATTR(psu_mfr_vin_max,    S_IRUGO, show_linear, NULL, PSU_MFR_VIN_MAX);
static SENSOR_DEVICE_ATTR(psu_mfr_pout_max,   S_IRUGO, show_linear, NULL, PSU_MFR_POUT_MAX);

static struct attribute *g657_s40_attributes[] = {
    &sensor_dev_attr_psu_power_on.dev_attr.attr,
    &sensor_dev_attr_psu_temp_fault.dev_attr.attr,
    &sensor_dev_attr_psu_power_good.dev_attr.attr,
    &sensor_dev_attr_psu_fan1_fault.dev_attr.attr,
    &sensor_dev_attr_psu_fan2_fault.dev_attr.attr,
    &sensor_dev_attr_psu_over_temp.dev_attr.attr,
    &sensor_dev_attr_psu_v_in.dev_attr.attr,
    &sensor_dev_attr_psu_v_out.dev_attr.attr,
    &sensor_dev_attr_psu_i_in.dev_attr.attr,
    &sensor_dev_attr_psu_i_out.dev_attr.attr,
    &sensor_dev_attr_psu_p_in.dev_attr.attr,
    &sensor_dev_attr_psu_p_out.dev_attr.attr,
    &sensor_dev_attr_psu_temp1.dev_attr.attr,
    &sensor_dev_attr_psu_temp2.dev_attr.attr,
    &sensor_dev_attr_psu_fan_speed1.dev_attr.attr,
    &sensor_dev_attr_psu_fan_speed2.dev_attr.attr,
    &sensor_dev_attr_psu_pmbus_revision.dev_attr.attr,
    &sensor_dev_attr_psu_mfr_id.dev_attr.attr,
    &sensor_dev_attr_psu_mfr_model.dev_attr.attr,
    &sensor_dev_attr_psu_mfr_revision.dev_attr.attr,
    &sensor_dev_attr_psu_mfr_serial.dev_attr.attr,
    &sensor_dev_attr_psu_mfr_vin_min.dev_attr.attr,
    &sensor_dev_attr_psu_mfr_vin_max.dev_attr.attr,
    &sensor_dev_attr_psu_mfr_pout_max.dev_attr.attr,
    NULL
};

static ssize_t show_byte(struct device *dev, struct device_attribute *da,
             char *buf)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct g657_s40_data *data = g657_s40_update_device(dev);
    
    return (attr->index == PSU_PMBUS_REVISION) ? sprintf(buf, "%d\n", data->pmbus_revision) :
                                 sprintf(buf, "0\n");
}

static ssize_t show_word(struct device *dev, struct device_attribute *da,
             char *buf)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct g657_s40_data *data = g657_s40_update_device(dev);
    u16 status = 0;
    
    switch (attr->index) {
    case PSU_POWER_ON: /* psu_power_on, low byte bit 6 of status_word, 0=>ON, 1=>OFF */
        status = (data->status_word & 0x40) ? 0 : 1; 
        break;
    case PSU_TEMP_FAULT: /* psu_temp_fault, low byte bit 2 of status_word, 0=>Normal, 1=>temp fault */
        status = (data->status_word & 0x4) >> 2;
        break;
    case PSU_POWER_GOOD: /* psu_power_good, high byte bit 3 of status_word, 0=>OK, 1=>FAIL */
        status = (data->status_word & 0x800) ? 0 : 1;
        break;
    }
    
    return sprintf(buf, "%d\n", status);
}

static int two_complement_to_int(u16 data, u8 valid_bit, int mask)
{
    u16  valid_data  = data & mask;
    bool is_negative = valid_data >> (valid_bit - 1);

    return is_negative ? (-(((~valid_data) & mask) + 1)) : valid_data;
}

static ssize_t show_linear(struct device *dev, struct device_attribute *da,
             char *buf)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct g657_s40_data *data = g657_s40_update_device(dev);

    u16 value = 0;
    int exponent, mantissa;
    
    switch (attr->index) {
    case PSU_V_IN:
        value = data->v_in;
        break;
    case PSU_I_IN:
        value = data->i_in;
        break;
    case PSU_I_OUT:
        value = data->i_out;
        break;
    case PSU_P_IN:
        value = data->p_in;
        break;
    case PSU_P_OUT:
        value = data->p_out;
        break;
    case PSU_TEMP1:
        value = data->temp[0];
        break;
    case PSU_TEMP2:
        value = data->temp[1];
        break;
    case PSU_FAN_SPEED1:
        value = data->fan_speed[0];
        break;
    case PSU_FAN_SPEED2:
        value = data->fan_speed[1];
        break;
    case PSU_MFR_VIN_MIN:
        value = data->mfr_vin_min;
        break;
    case PSU_MFR_VIN_MAX:
        value = data->mfr_vin_max;
        break;
    case PSU_MFR_POUT_MAX:
        value = data->mfr_pout_max;
        break;
    }
    
    exponent = two_complement_to_int(value >> 11, 5, 0x1f);
    mantissa = two_complement_to_int(value & 0x7ff, 11, 0x7ff);

    return (exponent > 0) ? sprintf(buf, "%d\n", mantissa << exponent) :
                            sprintf(buf, "%d\n", mantissa >> -exponent);
}

static ssize_t show_fan_fault(struct device *dev, struct device_attribute *da,
             char *buf)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct g657_s40_data *data = g657_s40_update_device(dev);

    u8 shift = (attr->index == PSU_FAN1_FAULT) ? 7 : 6;

    return sprintf(buf, "%d\n", data->fan_fault >> shift);
}

static ssize_t show_over_temp(struct device *dev, struct device_attribute *da,
             char *buf)
{
    struct g657_s40_data *data = g657_s40_update_device(dev);

    return sprintf(buf, "%d\n", data->over_temp >> 7);
}

static ssize_t show_ascii(struct device *dev, struct device_attribute *da,
             char *buf)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct g657_s40_data *data = g657_s40_update_device(dev);
    u8 *ptr = NULL;

    switch (attr->index) {
    case PSU_MFR_ID: /* psu_mfr_id */
        ptr = data->mfr_id;
        break;
    case PSU_MFR_MODEL: /* psu_mfr_model */
        ptr = data->mfr_model;
        break;
    case PSU_MFR_REVISION: /* psu_mfr_revision */
        ptr = data->mfr_revsion;
        break;
    case PSU_MFR_SERIAL: /* psu_mfr_serial */
        ptr = data->mfr_serial;
        break;
    default:
        return 0;
    }
    
    return sprintf(buf, "%s\n", ptr);
}
    
static ssize_t show_vout(struct device *dev, struct device_attribute *da,
             char *buf)
{
    struct g657_s40_data *data = g657_s40_update_device(dev);
    int exponent, mantissa;

    exponent = two_complement_to_int(data->vout_mode, 5, 0x1f);
    mantissa = data->v_out;

    return (exponent > 0) ? sprintf(buf, "%d\n", mantissa << exponent) :
                            sprintf(buf, "%d\n", mantissa >> -exponent);
}

static const struct attribute_group g657_s40_group = {
    .attrs = g657_s40_attributes,
};

static int g657_s40_probe(struct i2c_client *client,
            const struct i2c_device_id *dev_id)
{
    struct g657_s40_data *data;
    int status;

    if (!i2c_check_functionality(client->adapter, 
        I2C_FUNC_SMBUS_BYTE_DATA | 
        I2C_FUNC_SMBUS_WORD_DATA | 
        I2C_FUNC_SMBUS_I2C_BLOCK)) {
        status = -EIO;
        goto exit;
    }

    data = kzalloc(sizeof(struct g657_s40_data), GFP_KERNEL);
    if (!data) {
        status = -ENOMEM;
        goto exit;
    }

    i2c_set_clientdata(client, data);
    mutex_init(&data->update_lock);

    dev_info(&client->dev, "chip found\n");

    /* Register sysfs hooks */
    status = sysfs_create_group(&client->dev.kobj, &g657_s40_group);
    if (status) {
        goto exit_free;
    }

    data->hwmon_dev = hwmon_device_register(&client->dev);
    if (IS_ERR(data->hwmon_dev)) {
        status = PTR_ERR(data->hwmon_dev);
        goto exit_remove;
    }

    dev_info(&client->dev, "%s: psu '%s'\n",
         dev_name(data->hwmon_dev), client->name);
    
    return 0;

exit_remove:
    sysfs_remove_group(&client->dev.kobj, &g657_s40_group);
exit_free:
    kfree(data);
exit:
    
    return status;
}

static int g657_s40_remove(struct i2c_client *client)
{
    struct g657_s40_data *data = i2c_get_clientdata(client);

    hwmon_device_unregister(data->hwmon_dev);
    sysfs_remove_group(&client->dev.kobj, &g657_s40_group);
    kfree(data);
    
    return 0;
}

static const struct i2c_device_id g657_s40_id[] = {
    { "g657_s40", 0 },
    {}
};
MODULE_DEVICE_TABLE(i2c, g657_s40_id);

static struct i2c_driver g657_s40_driver = {
    .class        = I2C_CLASS_HWMON,
    .driver = {
        .name    = "g657_s40",
    },
    .probe      = g657_s40_probe,
    .remove      = g657_s40_remove,
    .id_table = g657_s40_id,
    .address_list = normal_i2c,
};

static int g657_s40_read_byte(struct i2c_client *client, u8 reg)
{
    return i2c_smbus_read_byte_data(client, reg);
}

static int g657_s40_read_word(struct i2c_client *client, u8 reg)
{
    return i2c_smbus_read_word_data(client, reg);
}

static int g657_s40_read_block(struct i2c_client *client, u8 command, u8 *data,
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

struct reg_data_byte {
    u8   reg;
    u8  *value;
};

struct reg_data_word {
    u8   reg;
    u16 *value;
};

static struct g657_s40_data *g657_s40_update_device(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct g657_s40_data *data = i2c_get_clientdata(client);
    
    mutex_lock(&data->update_lock);

    if (time_after(jiffies, data->last_updated + HZ + HZ / 2)
        || !data->valid) {
        int i, status;
        u8 command;
        struct reg_data_byte regs_byte[] = { {0x19, &data->capability},
                                             {0x20, &data->vout_mode},
                                             {0x7d, &data->over_temp}, 
                                             {0x81, &data->fan_fault},
                                             {0x98, &data->pmbus_revision} };
        struct reg_data_word regs_word[] = { {0x79, &data->status_word},
                                             {0x88, &data->v_in},
                                             {0x8b, &data->v_out},
                                             {0x89, &data->i_in},
                                             {0x8c, &data->i_out},
                                             {0x96, &data->p_out},
                                             {0x97, &data->p_in},
                                             {0x8d, &(data->temp[0])},
                                             {0x8e, &(data->temp[1])},
                                             {0x90, &(data->fan_speed[0])},
                                             {0x91, &(data->fan_speed[1])},
                                             {0xa0, &data->mfr_vin_min},
                                             {0xa1, &data->mfr_vin_max},
                                             {0xa7, &data->mfr_pout_max} };

        dev_dbg(&client->dev, "Starting g657_s40 update\n");

        /* Read byte data */        
        for (i = 0; i < ARRAY_SIZE(regs_byte); i++) {
            status = g657_s40_read_byte(client, regs_byte[i].reg);
            
            if (status < 0)
            {
                dev_dbg(&client->dev, "reg %d, err %d\n",
                        regs_byte[i].reg, status);
            }
            else
                *(regs_byte[i].value) = status;
        }
                    
        /* Read word data */                    
        for (i = 0; i < ARRAY_SIZE(regs_word); i++) {
            status = g657_s40_read_word(client, regs_word[i].reg);
            
            if (status < 0)
            {
                dev_dbg(&client->dev, "reg %d, err %d\n",
                        regs_word[i].reg, status);
            }
            else
                *(regs_word[i].value) = status;
        }
        
        /* Read mfr_id */
        command = 0x99;
        status = g657_s40_read_block(client, command, data->mfr_id, 
                                         ARRAY_SIZE(data->mfr_id)-1);    
        data->mfr_id[ARRAY_SIZE(data->mfr_id)-1] = '\0';
        if (status < 0)
            dev_dbg(&client->dev, "reg %d, err %d\n", command, status);
            
        /* Read mfr_model */
        command = 0x9a;
        status = g657_s40_read_block(client, command, data->mfr_model, 
                                         ARRAY_SIZE(data->mfr_model)-1);
        data->mfr_model[ARRAY_SIZE(data->mfr_model)-1] = '\0';
        if (status < 0)
            dev_dbg(&client->dev, "reg %d, err %d\n", command, status);
        
        /* Read mfr_revsion */
        command = 0x9b;
        status = g657_s40_read_block(client, command, data->mfr_revsion, 
                                         ARRAY_SIZE(data->mfr_revsion)-1);
        data->mfr_revsion[ARRAY_SIZE(data->mfr_revsion)-1] = '\0';
        if (status < 0)
            dev_dbg(&client->dev, "reg %d, err %d\n", command, status);

        /* Read mfr_serial */
        command = 0x9e;
        status = g657_s40_read_block(client, command, data->mfr_serial, 
                                         ARRAY_SIZE(data->mfr_serial)-1);
        data->mfr_serial[ARRAY_SIZE(data->mfr_serial)-1] = '\0';
        if (status < 0)
            dev_dbg(&client->dev, "reg %d, err %d\n", command, status);

        data->last_updated = jiffies;
        data->valid = 1;
    }

    mutex_unlock(&data->update_lock);

    return data;
}

static int __init g657_s40_init(void)
{
    return i2c_add_driver(&g657_s40_driver);
}

static void __exit g657_s40_exit(void)
{
    i2c_del_driver(&g657_s40_driver);
}

MODULE_AUTHOR("Brandon Chuang <brandon_chuang@accton.com.tw>");
MODULE_DESCRIPTION("EFRP_G657_S40 driver");
MODULE_LICENSE("GPL");

module_init(g657_s40_init);
module_exit(g657_s40_exit);
