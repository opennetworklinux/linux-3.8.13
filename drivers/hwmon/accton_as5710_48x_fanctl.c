/*
 * A hwmon driver for the Accton as5710 54x fan contrl
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
#include <linux/delay.h>
#include <linux/i2c/accton_as5710_48x_cpld.h>
#include "lm75.h"

#define FAN_STATUS_POLLING_INTERVAL 3000

static struct task_struct *fanctrl_tsk = NULL;

struct fan_speed_temp_data {
   int  duty_cycle;
   int  down_adjust_temp;
   int  up_adjust_temp;
};

enum fan_duty_cycle {
    FAN_DUTY_CYCLE_MIN = 50,
	FAN_DUTY_CYCLE_65  = 65,
	FAN_DUTY_CYCLE_80  = 80,
	FAN_DUTY_CYCLE_MAX = 100
};

struct fan_speed_temp_data  fan_speed_data[] = {
{FAN_DUTY_CYCLE_MIN,     0,  46500},
{FAN_DUTY_CYCLE_65,  42700,  50000},
{FAN_DUTY_CYCLE_80,  47700,  54700},
{FAN_DUTY_CYCLE_MAX, 52700,      0}
};

static struct accton_as5710_48x_fan  *fan_data = NULL;

struct accton_as5710_48x_fan {
    struct device   *dev;
	struct mutex     update_lock;
    char             valid;           /* != 0 if registers are valid */
    unsigned long    last_updated;    /* In jiffies */
    u8               reg_val[2];     /* Register value, 0 = fan fault status
                                                        1 = fan PWM duty cycle */
};

/* fan related data
 */
static const u8 fan_reg[] = {
    0xC,        /* fan fault */
    0xD,        /* fan PWM duty cycle */
};

enum sysfs_fan_attributes {
    FAN_FAULT = 0,
    FAN_DUTY_CYCLE
};
    
struct fan_pwm_duty_cycle_data {
    u8 reg_val;    /* Register Value of CPLD */
    u8 duty_cycle; /* 0-100 */
};

struct fan_pwm_duty_cycle_data pwm_data[] = {{0x0a, 50}, {0x0d, 65}, 
                                             {0x10, 80}, {0x14, 100}};

static void accton_as5710_48x_fan_update_device(struct device *dev);
static int accton_as5710_48x_fan_read_value(u8 reg);
static int accton_as5710_48x_fan_write_value(u8 reg, u8 value);
											 
static int fan_reg_val_to_duty_cycle(u8 reg_val);
static int fan_duty_cycle_to_reg_val(u8 duty_cycle);
static ssize_t fan_set_duty_cycle(struct device *dev, 
                    struct device_attribute *da,const char *buf, size_t count);
static ssize_t fan_show_value(struct device *dev, 
                    struct device_attribute *da, char *buf);
                    
static SENSOR_DEVICE_ATTR(fan_fault, S_IRUGO, fan_show_value, NULL, FAN_FAULT);
static SENSOR_DEVICE_ATTR(fan_duty_cycle, S_IWUSR | S_IRUGO, fan_show_value, 
                                           fan_set_duty_cycle, FAN_DUTY_CYCLE);

static struct attribute *accton_as5710_48x_fan_attributes[] = {
    /* fan related attributes */
    &sensor_dev_attr_fan_fault.dev_attr.attr,
    &sensor_dev_attr_fan_duty_cycle.dev_attr.attr,
    NULL
};

/* fan related functions
 */
static int fan_reg_val_to_duty_cycle(u8 reg_val) 
{
    int i;
    
    for (i = 0; i < ARRAY_SIZE(pwm_data); i++) {
        if (pwm_data[i].reg_val == reg_val)
            return pwm_data[i].duty_cycle;
    }
    
    return pwm_data[0].duty_cycle;
}

static int fan_duty_cycle_to_reg_val(u8 duty_cycle) 
{
    int i;
    
    for (i = 0; i < ARRAY_SIZE(pwm_data); i++) {
        if (pwm_data[i].duty_cycle == duty_cycle)
            return pwm_data[i].reg_val;
    }
    
    return pwm_data[0].reg_val;
}

static ssize_t fan_show_value(struct device *dev, struct device_attribute *da,
             char *buf)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    ssize_t ret = 0;
    
    accton_as5710_48x_fan_update_device(dev);
    
	if (fan_data->valid) {
		switch (attr->index) {
		case FAN_FAULT:
			ret = sprintf(buf, "%d\n", fan_data->reg_val[0] ? 1 : 0);
			break;
		case FAN_DUTY_CYCLE:
			ret = sprintf(buf, "%d\n", fan_reg_val_to_duty_cycle(fan_data->reg_val[1]));
			break;
		default:
			break;
		}
	}
    
    return ret;
}

static ssize_t fan_set_duty_cycle(struct device *dev, struct device_attribute *da,
            const char *buf, size_t count) {
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    int error, value;
    int idx = attr->index;
    
    error = kstrtoint(buf, 10, &value);
    if (error)
        return error;
		
    if (value < 0 || value > FAN_DUTY_CYCLE_MAX)
	    return -EINVAL;

    accton_as5710_48x_fan_write_value(fan_reg[idx], fan_duty_cycle_to_reg_val(value));
    
    return count;
}

static const struct attribute_group accton_as5710_48x_fan_group = {
    .attrs = accton_as5710_48x_fan_attributes,
};

static int accton_as5710_48x_fan_read_value(u8 reg)
{
    return accton_as5710_48x_cpld_read(0x60, reg);
}

static int accton_as5710_48x_fan_write_value(u8 reg, u8 value)
{
    return accton_as5710_48x_cpld_write(0x60, reg, value);
}

static void accton_as5710_48x_fan_update_device(struct device *dev)
{
    mutex_lock(&fan_data->update_lock);

    if (time_after(jiffies, fan_data->last_updated + HZ + HZ / 2) || 
        !fan_data->valid) {
        int i;

        dev_dbg(fan_data->dev, "Starting accton_as5710_48x_fan update\n");

        /* Update fan data
         */
        for (i = 0; i < ARRAY_SIZE(fan_data->reg_val); i++) {
            int status = accton_as5710_48x_fan_read_value(fan_reg[i]);
			
            if (status < 0) {
                fan_data->valid = 0;
                dev_dbg(fan_data->dev, "reg %d, err %d\n", fan_reg[i], status);
				return;
            }
            else
                fan_data->reg_val[i] = status;
        }
        
        fan_data->last_updated = jiffies;
        fan_data->valid = 1;
    }
	
	mutex_unlock(&fan_data->update_lock);
}

static int read_temperature(int *temp)
{
    int temp1 = 0, temp2 = 0;
	
	if (lm75_read_reg_value(0x48, 0x0, &temp1) < 0 || 
	    lm75_read_reg_value(0x49, 0x0, &temp2) < 0) {
	    return -EIO;
	}

	*temp = (temp1 + temp2) / 2;
    return 0;
}

static int is_fan_failed(void) 
{	
	if (!fan_data || !fan_data->dev)
	    return 1;
		
    accton_as5710_48x_fan_update_device(fan_data->dev);

	return fan_data->reg_val[FAN_FAULT];
}

static void set_fan_speed(int duty_cycle) 
{
	if (!fan_data)
	    return;
		
    accton_as5710_48x_fan_write_value(fan_reg[1], fan_duty_cycle_to_reg_val(duty_cycle));
}

static void set_fan_speed_by_temp(int duty_cycle, int temperature) {  
    int  i, current_duty_cycle = duty_cycle;

	for (i = 0; i < ARRAY_SIZE(fan_speed_data); i++) {
	    if (fan_speed_data[i].duty_cycle != duty_cycle)
		    continue;
			
		break;
	}

	if (i == ARRAY_SIZE(fan_speed_data)) {
        dev_dbg(fan_data->dev, "no matched duty cycle (%d)\n", duty_cycle);
		return;
	}
	
	/* Adjust new duty cycle
	 */
    if ((temperature >= fan_speed_data[i].up_adjust_temp) && 
	                   (duty_cycle != FAN_DUTY_CYCLE_MAX)) {
	    current_duty_cycle = fan_speed_data[++i].duty_cycle;
	}
	else if ((temperature <= fan_speed_data[i].down_adjust_temp) && 
	                        (duty_cycle != FAN_DUTY_CYCLE_MIN)) {
	    current_duty_cycle = fan_speed_data[--i].duty_cycle;
	}

#if 0	
	if (current_duty_cycle == duty_cycle) {
        /* Duty cycle does not change, just return */
	    return;
	}
#endif
	
	/* Update current duty cycle
	 */
	set_fan_speed(current_duty_cycle);
}

static int fan_speed_ctrl_routine(void *arg)
{
    do {
		int duty_cycle, temp = 0;

		msleep(FAN_STATUS_POLLING_INTERVAL);
		
		if (!fan_data || !fan_data->dev) {
			continue;
		}

		/* Check if any fan is in failed state, if so, set fan as full speed
		 */
		if (is_fan_failed()) {            
            // Vinesson: temperature for ignore this in order to HW thermal test.            
			set_fan_speed(FAN_DUTY_CYCLE_MAX);
			continue;
		}

		if (read_temperature(&temp) < 0) {
			continue;
		}
		
		/* Set fan speed by current duty cycle and temperature
		 */
		accton_as5710_48x_fan_update_device(fan_data->dev);
		duty_cycle = fan_reg_val_to_duty_cycle(fan_data->reg_val[1]);
		set_fan_speed_by_temp(duty_cycle, temp);
	} while (1);
	
	return 0;
}

static void fan_speed_ctrl_cleanup(void)
{
    if (fanctrl_tsk) {
        kthread_stop(fanctrl_tsk);
		fanctrl_tsk = NULL;
	}
}

static int fan_speed_ctrl_init(void)
{
    int ret;

	fanctrl_tsk = kthread_create(fan_speed_ctrl_routine, NULL, "accton_as5710_48x_fanctl");
	if (IS_ERR(fanctrl_tsk)) {
		ret = PTR_ERR(fanctrl_tsk);
		fanctrl_tsk = NULL;
        return -EPERM;
	}
	
	wake_up_process(fanctrl_tsk);

    return 0;
}

static int __init accton_as5710_48x_fan_init(void)
{
    int status = 0;

    fan_data = kzalloc(sizeof(struct accton_as5710_48x_fan), GFP_KERNEL);
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
    
    status = sysfs_create_group(&fan_data->dev->kobj, &accton_as5710_48x_fan_group);
    if (status) {
        goto exit_unregister;
    }

    dev_info(fan_data->dev, "accton_as5710_48x_fan\n");
    
	/* initialize fan speed control routine */
	fan_speed_ctrl_init();
	
    return 0;

exit_unregister:
    hwmon_device_unregister(fan_data->dev);
exit_free:
    kfree(fan_data);	
exit:
    return status;
}

static void __exit accton_as5710_48x_fan_exit(void)
{
    fan_speed_ctrl_cleanup();
	
    sysfs_remove_group(&fan_data->dev->kobj, &accton_as5710_48x_fan_group);
	hwmon_device_unregister(fan_data->dev);
    kfree(fan_data);
}

MODULE_AUTHOR("Brandon Chuang <brandon_chuang@accton.com.tw>");
MODULE_DESCRIPTION("accton_as5710_48x_fan driver");
MODULE_LICENSE("GPL");

module_init(accton_as5710_48x_fan_init);
module_exit(accton_as5710_48x_fan_exit);
