/*
 * A hwmon driver for the Accton as6700 32x fan contrl
 *
 * Copyright (C) 2013 Accton Technology Corporation.
 * Brandon Chuang <brandon_chuang@accton.com.tw>
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
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/kthread.h>

#define FAN_NUMBER                       5
#define FAN_DUTY_CYCLE_MIN               0    /* 0% */
#define FAN_DUTY_CYCLE_MAX               100  /* 100% */

#define CPLD_REG_FAN_1_SPEED_OFFSET       0x0
#define CPLD_REG_FAN_1A_SPEED_OFFSET      0x1
#define CPLD_REG_FAN_2_SPEED_OFFSET       0x2
#define CPLD_REG_FAN_2A_SPEED_OFFSET      0x3
#define CPLD_REG_FAN_3_SPEED_OFFSET       0x4
#define CPLD_REG_FAN_3A_SPEED_OFFSET      0x5
#define CPLD_REG_FAN_4_SPEED_OFFSET       0x6
#define CPLD_REG_FAN_4A_SPEED_OFFSET      0x7
#define CPLD_REG_FAN_5_SPEED_OFFSET       0x8
#define CPLD_REG_FAN_5A_SPEED_OFFSET      0x9
#define CPLD_REG_FAN_DIRECTION_OFFSET     0x0A
#define CPLD_REG_FAN_PWM_CYCLE_OFFSET     0x20

#define CPLD_FAN_1_INFO_BIT_MASK       0x1
#define CPLD_FAN_2_INFO_BIT_MASK       0x2
#define CPLD_FAN_3_INFO_BIT_MASK       0x4
#define CPLD_FAN_4_INFO_BIT_MASK       0x8
#define CPLD_FAN_5_INFO_BIT_MASK       0x10

struct as6700_fan_pwm_duty_cycle_data {
    u8 reg_val;    /* Register Value of CPLD */
    float duty_cycle; /* 0-100 */
};

struct as6700_fan_pwm_duty_cycle_data as6700_pwm_data[] = {{0x00, 0}, {0x03, 37.5}, 
                                             {0x04, 50}, {0x05, 62.5}, {0x06, 75}, {0x07, 100}};

static struct accton_as6700_32x_fan  *fan_data = NULL;

struct accton_as6700_32x_fan {
    struct device   *dev;
    struct mutex     update_lock;
    char             valid;           /* != 0 if registers are valid */
    unsigned long    last_updated;    /* In jiffies */
    u8               pwm;
    u32              duty_cycle[FAN_NUMBER*2];
    u8               direction[FAN_NUMBER];    
};

/* fan related data
 */
static const u8 fan_info_mask[] = {
    CPLD_FAN_1_INFO_BIT_MASK,
    CPLD_FAN_2_INFO_BIT_MASK,
    CPLD_FAN_3_INFO_BIT_MASK,
    CPLD_FAN_4_INFO_BIT_MASK,
    CPLD_FAN_5_INFO_BIT_MASK
};

static const u8 fan_speed_reg[] = {
    CPLD_REG_FAN_1_SPEED_OFFSET,
    CPLD_REG_FAN_2_SPEED_OFFSET,
    CPLD_REG_FAN_3_SPEED_OFFSET,
    CPLD_REG_FAN_4_SPEED_OFFSET,
    CPLD_REG_FAN_5_SPEED_OFFSET
};

static const u8 fana_speed_reg[] = {
    CPLD_REG_FAN_1A_SPEED_OFFSET,
    CPLD_REG_FAN_2A_SPEED_OFFSET,
    CPLD_REG_FAN_3A_SPEED_OFFSET,
    CPLD_REG_FAN_4A_SPEED_OFFSET,
    CPLD_REG_FAN_5A_SPEED_OFFSET
};

enum sysfs_fan_attributes {
    FAN_DUTY_CYCLE,
    FAN_1_SPEED,
    FAN_1A_SPEED,
    FAN_2_SPEED,
    FAN_2A_SPEED,
    FAN_3_SPEED,
    FAN_3A_SPEED,
    FAN_4_SPEED,
    FAN_4A_SPEED,
    FAN_5_SPEED,    
    FAN_5A_SPEED,    
    FAN_1_DIRECTION,
    FAN_2_DIRECTION,
    FAN_3_DIRECTION,
    FAN_4_DIRECTION,
    FAN_5_DIRECTION,
};

static void accton_as6700_32x_fan_update_device(struct device *dev);
static int accton_as6700_32x_fan_read_value(u8 reg);
static int accton_as6700_32x_fan_write_value(u8 reg, u8 value);
                                             
static ssize_t fan_set_duty_cycle(struct device *dev, 
                    struct device_attribute *da,const char *buf, size_t count);
static ssize_t fan_show_value(struct device *dev, 
                    struct device_attribute *da, char *buf);

extern int accton_i2c_cpld_read(unsigned short cpld_addr, u8 reg);
extern int accton_i2c_cpld_write(unsigned short cpld_addr, u8 reg, u8 value);
                    
static SENSOR_DEVICE_ATTR(accton_as6700_32x_fan_duty_cycle_level, S_IWUSR | S_IRUGO, fan_show_value, fan_set_duty_cycle, FAN_DUTY_CYCLE);
static SENSOR_DEVICE_ATTR(accton_as6700_32x_fan1_speed_rpm, S_IRUGO, fan_show_value, 
                                           NULL, FAN_1_SPEED);
static SENSOR_DEVICE_ATTR(accton_as6700_32x_fan2_speed_rpm, S_IRUGO, fan_show_value, 
                                           NULL, FAN_2_SPEED);
static SENSOR_DEVICE_ATTR(accton_as6700_32x_fan3_speed_rpm,  S_IRUGO, fan_show_value, 
                                           NULL, FAN_3_SPEED);
static SENSOR_DEVICE_ATTR(accton_as6700_32x_fan4_speed_rpm,  S_IRUGO, fan_show_value, 
                                           NULL, FAN_4_SPEED);
static SENSOR_DEVICE_ATTR(accton_as6700_32x_fan5_speed_rpm, S_IRUGO, fan_show_value, 
                                           NULL, FAN_5_SPEED);
static SENSOR_DEVICE_ATTR(accton_as6700_32x_fan1a_speed_rpm, S_IRUGO, fan_show_value, 
                                           NULL, FAN_1A_SPEED);
static SENSOR_DEVICE_ATTR(accton_as6700_32x_fan2a_speed_rpm, S_IRUGO, fan_show_value, 
                                           NULL, FAN_2A_SPEED);
static SENSOR_DEVICE_ATTR(accton_as6700_32x_fan3a_speed_rpm,  S_IRUGO, fan_show_value, 
                                           NULL, FAN_3A_SPEED);
static SENSOR_DEVICE_ATTR(accton_as6700_32x_fan4a_speed_rpm,  S_IRUGO, fan_show_value, 
                                           NULL, FAN_4A_SPEED);
static SENSOR_DEVICE_ATTR(accton_as6700_32x_fan5a_speed_rpm, S_IRUGO, fan_show_value, 
                                           NULL, FAN_5A_SPEED);
static SENSOR_DEVICE_ATTR(accton_as6700_32x_fan1_direction, S_IRUGO, fan_show_value, NULL, FAN_1_DIRECTION);
static SENSOR_DEVICE_ATTR(accton_as6700_32x_fan2_direction, S_IRUGO, fan_show_value, NULL, FAN_2_DIRECTION);
static SENSOR_DEVICE_ATTR(accton_as6700_32x_fan3_direction, S_IRUGO, fan_show_value, NULL, FAN_3_DIRECTION);
static SENSOR_DEVICE_ATTR(accton_as6700_32x_fan4_direction, S_IRUGO, fan_show_value, NULL, FAN_4_DIRECTION);
static SENSOR_DEVICE_ATTR(accton_as6700_32x_fan5_direction, S_IRUGO, fan_show_value, NULL, FAN_5_DIRECTION);

static struct attribute *accton_as6700_32x_fan_attributes[] = {
    /* fan related attributes */
    &sensor_dev_attr_accton_as6700_32x_fan_duty_cycle_level.dev_attr.attr,
    &sensor_dev_attr_accton_as6700_32x_fan1_speed_rpm.dev_attr.attr,
    &sensor_dev_attr_accton_as6700_32x_fan2_speed_rpm.dev_attr.attr,
    &sensor_dev_attr_accton_as6700_32x_fan3_speed_rpm.dev_attr.attr,
    &sensor_dev_attr_accton_as6700_32x_fan4_speed_rpm.dev_attr.attr,
    &sensor_dev_attr_accton_as6700_32x_fan5_speed_rpm.dev_attr.attr,
    &sensor_dev_attr_accton_as6700_32x_fan1a_speed_rpm.dev_attr.attr,
    &sensor_dev_attr_accton_as6700_32x_fan2a_speed_rpm.dev_attr.attr,
    &sensor_dev_attr_accton_as6700_32x_fan3a_speed_rpm.dev_attr.attr,
    &sensor_dev_attr_accton_as6700_32x_fan4a_speed_rpm.dev_attr.attr,
    &sensor_dev_attr_accton_as6700_32x_fan5a_speed_rpm.dev_attr.attr,    
    &sensor_dev_attr_accton_as6700_32x_fan1_direction.dev_attr.attr,
    &sensor_dev_attr_accton_as6700_32x_fan2_direction.dev_attr.attr,
    &sensor_dev_attr_accton_as6700_32x_fan3_direction.dev_attr.attr,
    &sensor_dev_attr_accton_as6700_32x_fan4_direction.dev_attr.attr,
    &sensor_dev_attr_accton_as6700_32x_fan5_direction.dev_attr.attr,
    NULL
};

/* fan related functions
 */
static ssize_t fan_show_value(struct device *dev, struct device_attribute *da,
             char *buf)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    ssize_t ret = 0;
    
    accton_as6700_32x_fan_update_device(dev);
    
    if (fan_data->valid) {
        switch (attr->index) {
        case FAN_DUTY_CYCLE:
            ret = sprintf(buf, "%d\n", fan_data->pwm);
            break;
        case FAN_1_SPEED:
        case FAN_2_SPEED:
        case FAN_3_SPEED:
        case FAN_4_SPEED:
        case FAN_5_SPEED:
        case FAN_1A_SPEED:
        case FAN_2A_SPEED:
        case FAN_3A_SPEED:
        case FAN_4A_SPEED:
        case FAN_5A_SPEED:            
            ret = sprintf(buf, "%d\n", fan_data->duty_cycle[attr->index - FAN_1_SPEED]);
            break;
        case FAN_1_DIRECTION:
        case FAN_2_DIRECTION:
        case FAN_3_DIRECTION:
        case FAN_4_DIRECTION:
        case FAN_5_DIRECTION:
            ret = sprintf(buf, "%d\n", fan_data->direction[attr->index - FAN_1_DIRECTION]);   /* presnet, need to modify*/
            break;  
        default:
            break;
        }
    }
    return ret;
}

static ssize_t fan_set_duty_cycle(struct device *dev, struct device_attribute *da,
            const char *buf, size_t count) {
    /*struct sensor_device_attribute *attr = to_sensor_dev_attr(da);*/
    int error, value;
    
    error = kstrtoint(buf, 10, &value);
    if (error)
        return error;
        
    if ((value < 0 || value > 7)||value==1||value==2)
        return -EINVAL;

    accton_as6700_32x_fan_write_value(CPLD_REG_FAN_PWM_CYCLE_OFFSET, value);

    fan_data->valid = 0;
    
    return count;
}

static const struct attribute_group accton_as6700_32x_fan_group = {
    .attrs = accton_as6700_32x_fan_attributes,
};

static int accton_as6700_32x_fan_read_value(u8 reg)
{
    return accton_i2c_cpld_read(0x35, reg);
}

static int accton_as6700_32x_fan_write_value(u8 reg, u8 value)
{
    return accton_i2c_cpld_write(0x35, reg, value);
}

static void accton_as6700_32x_fan_update_device(struct device *dev)
{
    int ret1=0, ret2=0, i=0;
    
    mutex_lock(&fan_data->update_lock);

    if (time_after(jiffies, fan_data->last_updated + HZ + HZ / 2) || 
        !fan_data->valid) {

        dev_dbg(fan_data->dev, "Starting accton_as6700_32x_fan update\n");

        /* Update fan data
         */

         /* fan power level 
         * 0: 0%, 3:37.5%, 4:50%, 5:62.5%, 6:75%, 7:100%
         * All FAN-tray module use the same power level.
         */
        ret1 = accton_as6700_32x_fan_read_value(CPLD_REG_FAN_PWM_CYCLE_OFFSET);
        if (ret1 < 0) {
            fan_data->valid = 0;
            dev_dbg(fan_data->dev, "reg %d, err %d\n", CPLD_REG_FAN_PWM_CYCLE_OFFSET, ret1);
            mutex_unlock(&fan_data->update_lock);
            return;
        }
        
        fan_data->pwm = ret1;
        
        /* fan direction */
        ret1 = accton_as6700_32x_fan_read_value(CPLD_REG_FAN_DIRECTION_OFFSET);
        if (ret1 < 0) {
            fan_data->valid = 0;
            dev_dbg(fan_data->dev, "reg %d, err %d\n", CPLD_REG_FAN_DIRECTION_OFFSET, ret1);
            mutex_unlock(&fan_data->update_lock);
            return;
        }
        
        for (i=0; i<FAN_NUMBER; i++)
        {
            fan_data->direction[i] = (ret1 & fan_info_mask[i]) >> i;
        }
        
        /* fan speed 
         * each fan tray have two fan. if fan not present, value is 00 or FF
         */
        for (i=0; i<FAN_NUMBER; i++)
        {
            ret1 = accton_as6700_32x_fan_read_value(fan_speed_reg[i]);
            if (ret1 < 0) {
                fan_data->valid = 0;
                dev_dbg(fan_data->dev, "reg %d, err %d\n", fan_speed_reg[i], ret1);
                mutex_unlock(&fan_data->update_lock);
                return;
            }
            ret2 = accton_as6700_32x_fan_read_value(fana_speed_reg[i]);
            if (ret2 < 0) {
                fan_data->valid = 0;
                dev_dbg(fan_data->dev, "reg %d, err %d\n", fana_speed_reg[i], ret2);
                mutex_unlock(&fan_data->update_lock);
                return;
            }
            /* rpm = 707550/cpld value */
           if ( ret1 != 0x00 && ret1 != 0xff) /* CPLD will change to 0xff whcih means no speed. */
           {           
               fan_data->duty_cycle[i*2] = 707550/ret1;
           }
           else
           {
               fan_data->duty_cycle[i*2] = 0;
           }
           if ( ret2 != 0x00 && ret2 != 0xff) /* CPLD will change to 0xff whcih means no speed. */
           {            
               fan_data->duty_cycle[i*2+1] =707550/ret2;
           }
           else
           {
               fan_data->duty_cycle[i*2+1] = 0;
           }
        }
        
        fan_data->last_updated = jiffies;
        fan_data->valid = 1;
    }
    
    mutex_unlock(&fan_data->update_lock);
}

static int __init accton_as6700_32x_fan_init(void)
{
    int status = 0;

    fan_data = kzalloc(sizeof(struct accton_as6700_32x_fan), GFP_KERNEL);
    if (!fan_data) {
        status = -ENOMEM;
        goto exit;
    }
    
    mutex_init(&fan_data->update_lock);

    /* Register sysfs hooks */
    fan_data->dev = hwmon_device_register(NULL);
    if (IS_ERR(fan_data->dev)) {
        goto exit_free;
    }
    
    status = sysfs_create_group(&fan_data->dev->kobj, &accton_as6700_32x_fan_group);
    if (status) {
        goto exit_unregister;
    }

    dev_info(fan_data->dev, "accton_as6700_32x_fan\n");
    
    return 0;

exit_unregister:
    hwmon_device_unregister(fan_data->dev);
exit_free:
    kfree(fan_data);
exit:
    return status;
}

static void __exit accton_as6700_32x_fan_exit(void)
{
    sysfs_remove_group(&fan_data->dev->kobj, &accton_as6700_32x_fan_group);
    hwmon_device_unregister(fan_data->dev);
    kfree(fan_data);
}

MODULE_AUTHOR("Brandon Chuang <brandon_chuang@accton.com.tw>");
MODULE_DESCRIPTION("accton_as6700_32x_fan driver");
MODULE_LICENSE("GPL");

module_init(accton_as6700_32x_fan_init);
module_exit(accton_as6700_32x_fan_exit);
