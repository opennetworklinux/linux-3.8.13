/*
 * A LED driver for the accton_as6700_32x_led
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/leds.h>
#include <linux/slab.h>

#define DRVNAME "accton_as6700_32x_led"

/*extern function */
extern int accton_i2c_cpld_read(unsigned short cpld_addr, u8 reg);
extern int accton_i2c_cpld_write(unsigned short cpld_addr, u8 reg, u8 value);

struct accton_as6700_32x_led_data {
    struct platform_device *pdev;
    struct mutex     update_lock;
    char             valid;           /* != 0 if registers are valid */
    unsigned long    last_updated;    /* In jiffies */
    u8               reg_val[2];     /* Register value, 0 = SYSTEM/ALARM LED
                                                        1 = FAN1/FAN2/FAN3/FAN4/FAN5 LED */
};

static struct accton_as6700_32x_led_data  *ledctl = NULL;

/* LED related data
 */
#define LED_TYPE_SYSTEM_REG_MASK         0x03
#define LED_MODE_SYSTEM_GREEN_MASK       0x02
#define LED_MODE_SYSTEM_GREEN_BLINK_MASK 0x01
#define LED_MODE_SYSTEM_OFF_MASK         0x03

#define LED_TYPE_ALARM_REG_MASK         0x0C
#define LED_MODE_ALARM_GREEN_MASK       0x08
#define LED_MODE_ALARM_RED_BLINK_MASK   0x04
#define LED_MODE_ALARM_OFF_MASK         0x0C

#define LED_TYPE_FAN1_REG_MASK    0x10
#define LED_MODE_FAN1_GREEN_MASK  0x0
#define LED_MODE_FAN1_RED_MASK    0x10

#define LED_TYPE_FAN2_REG_MASK    0x08
#define LED_MODE_FAN2_GREEN_MASK  0x0
#define LED_MODE_FAN2_RED_MASK    0x08
 
#define LED_TYPE_FAN3_REG_MASK    0x04
#define LED_MODE_FAN3_GREEN_MASK  0x0
#define LED_MODE_FAN3_RED_MASK    0x04

#define LED_TYPE_FAN4_REG_MASK    0x02
#define LED_MODE_FAN4_GREEN_MASK  0x0
#define LED_MODE_FAN4_RED_MASK    0x02

#define LED_TYPE_FAN5_REG_MASK    0x01
#define LED_MODE_FAN5_GREEN_MASK  0x0
#define LED_MODE_FAN5_RED_MASK    0x01

static const unsigned short cpld_reg[] = {
    0x31,        /* SYS/ALARM LED*/
    0x35,        /* FAN1/FAN2/FAN3/FAN4/FAN5 LED */
};

static const u8 led_reg[] = {
    0x2D,        /* SYS/ALARM LED*/
    0xB,        /* FAN1/FAN2/FAN3/FAN4/FAN5 LED */
};

enum led_type {
    LED_TYPE_SYSTEM,
    LED_TYPE_ALARM,
    LED_TYPE_FAN1,
    LED_TYPE_FAN2,
    LED_TYPE_FAN3,
    LED_TYPE_FAN4,
    LED_TYPE_FAN5
};

enum led_light_mode {
    /* system led mode */
    LED_MODE_SYSTEM_OFF = 0,
    LED_MODE_SYSTEM_GREEN_BLINK,
    LED_MODE_SYSTEM_GREEN,
    LED_MODE_SYSTEM_OFF_2,

    /* alarm led mode */
    LED_MODE_ALARM_OFF = 0,
    LED_MODE_ALARM_RED_BLINK,
    LED_MODE_ALARM_GREEN,
    LED_MODE_ALARM_OFF_2,

    /* fan led mode */
    LED_MODE_FAN_GREEN = 0,
    LED_MODE_FAN_RED
};

struct led_type_mode {
    enum led_type type;
    int  type_mask;
    enum led_light_mode mode;
    int  mode_mask;
};

static struct led_type_mode led_type_mode_data[] = {
{LED_TYPE_SYSTEM, LED_TYPE_SYSTEM_REG_MASK, LED_MODE_SYSTEM_GREEN,       LED_MODE_SYSTEM_GREEN_MASK},
{LED_TYPE_SYSTEM, LED_TYPE_SYSTEM_REG_MASK, LED_MODE_SYSTEM_GREEN_BLINK, LED_MODE_SYSTEM_GREEN_BLINK_MASK},
{LED_TYPE_SYSTEM, LED_TYPE_SYSTEM_REG_MASK, LED_MODE_SYSTEM_OFF,         LED_MODE_SYSTEM_OFF_MASK},
{LED_TYPE_ALARM,  LED_TYPE_ALARM_REG_MASK,  LED_MODE_ALARM_GREEN,        LED_MODE_ALARM_GREEN_MASK},
{LED_TYPE_ALARM,  LED_TYPE_ALARM_REG_MASK,  LED_MODE_ALARM_RED_BLINK,    LED_MODE_ALARM_RED_BLINK_MASK},
{LED_TYPE_ALARM,  LED_TYPE_ALARM_REG_MASK,  LED_MODE_ALARM_OFF,          LED_MODE_ALARM_OFF_MASK},
{LED_TYPE_FAN1,   LED_TYPE_FAN1_REG_MASK,   LED_MODE_FAN_GREEN,          LED_MODE_FAN1_GREEN_MASK},
{LED_TYPE_FAN1,   LED_TYPE_FAN1_REG_MASK,   LED_MODE_FAN_RED,            LED_MODE_FAN1_RED_MASK},
{LED_TYPE_FAN2,   LED_TYPE_FAN2_REG_MASK,   LED_MODE_FAN_GREEN,          LED_MODE_FAN2_GREEN_MASK},
{LED_TYPE_FAN2,   LED_TYPE_FAN2_REG_MASK,   LED_MODE_FAN_RED,            LED_MODE_FAN2_RED_MASK},
{LED_TYPE_FAN3,   LED_TYPE_FAN3_REG_MASK,   LED_MODE_FAN_GREEN,          LED_MODE_FAN3_GREEN_MASK},
{LED_TYPE_FAN3,   LED_TYPE_FAN3_REG_MASK,   LED_MODE_FAN_RED,            LED_MODE_FAN3_RED_MASK},
{LED_TYPE_FAN4,   LED_TYPE_FAN4_REG_MASK,   LED_MODE_FAN_GREEN,          LED_MODE_FAN4_GREEN_MASK},
{LED_TYPE_FAN4,   LED_TYPE_FAN4_REG_MASK,   LED_MODE_FAN_RED,            LED_MODE_FAN4_RED_MASK},
{LED_TYPE_FAN5,   LED_TYPE_FAN5_REG_MASK,   LED_MODE_FAN_GREEN,          LED_MODE_FAN5_GREEN_MASK},
{LED_TYPE_FAN5,   LED_TYPE_FAN5_REG_MASK,   LED_MODE_FAN_RED,            LED_MODE_FAN5_RED_MASK}
};

static int led_reg_val_to_light_mode(enum led_type type, u8 reg_val) {
    int i;
    
    for (i = 0; i < ARRAY_SIZE(led_type_mode_data); i++) {
        if (type != led_type_mode_data[i].type)
            continue;
            
        if ((led_type_mode_data[i].type_mask & reg_val) == 
             led_type_mode_data[i].mode_mask)
            return led_type_mode_data[i].mode;
    }
    
    return 0;
}

static u8 led_light_mode_to_reg_val(enum led_type type, 
                                    enum led_light_mode mode, u8 reg_val) {
    int i;
                                      
    for (i = 0; i < ARRAY_SIZE(led_type_mode_data); i++) {
        if (type != led_type_mode_data[i].type)
            continue;

        if (mode != led_type_mode_data[i].mode)
            continue;
            
        reg_val = led_type_mode_data[i].mode_mask | 
                 (reg_val & (~led_type_mode_data[i].type_mask));
    }
    
    return reg_val;
}

static int accton_as6700_32x_led_read_value(unsigned short cpld, u8 reg)
{
    return accton_i2c_cpld_read(cpld, reg);
}

static int accton_as6700_32x_led_write_value(unsigned short cpld,u8 reg, u8 value)
{
    return accton_i2c_cpld_write(cpld, reg, value);
}

static void accton_as6700_32x_led_update(void)
{
    mutex_lock(&ledctl->update_lock);

    if (time_after(jiffies, ledctl->last_updated + HZ + HZ / 2)
        || !ledctl->valid) {
        int i;

        dev_dbg(&ledctl->pdev->dev, "Starting accton_as6700_32x_led update\n");
        
        /* Update LED data
         */
        for (i = 0; i < ARRAY_SIZE(ledctl->reg_val); i++) {
            int status = accton_as6700_32x_led_read_value(cpld_reg[i],led_reg[i]);
            
            if (status < 0) {
                ledctl->valid = 0;
                dev_dbg(&ledctl->pdev->dev, "reg %d, reg %d, err %d\n", cpld_reg[i], led_reg[i], status);
                mutex_unlock(&ledctl->update_lock);
                return;
            }
            else
                ledctl->reg_val[i] = status;
        }
        
        ledctl->last_updated = jiffies;
        ledctl->valid = 1;
    }
    
    mutex_unlock(&ledctl->update_lock);
}

static void accton_as6700_32x_led_set(struct led_classdev *led_cdev,
                                      enum led_brightness led_light_mode, u8 cpld,
                                      u8 reg, enum led_type type)
{
    int reg_val;
    
    mutex_lock(&ledctl->update_lock);
    
    reg_val = accton_as6700_32x_led_read_value(cpld,reg);
    
    if (reg_val < 0) {
        dev_dbg(&ledctl->pdev->dev, "reg %d, reg %d, err %d\n", cpld, reg, reg_val);
        goto exit;
    }

    reg_val = led_light_mode_to_reg_val(type, led_light_mode, reg_val);
    accton_as6700_32x_led_write_value(cpld,reg, reg_val);
    
exit:
    mutex_unlock(&ledctl->update_lock);
}

static void accton_as6700_32x_led_system_set(struct led_classdev *led_cdev,
                                            enum led_brightness led_light_mode)
{
    accton_as6700_32x_led_set(led_cdev, led_light_mode, cpld_reg[0], led_reg[0], LED_TYPE_SYSTEM);
}

static enum led_brightness accton_as6700_32x_led_system_get(struct led_classdev *cdev)
{
    accton_as6700_32x_led_update();
    return led_reg_val_to_light_mode(LED_TYPE_SYSTEM, ledctl->reg_val[0]);
}

static void accton_as6700_32x_led_alarm_set(struct led_classdev *led_cdev,
                                            enum led_brightness led_light_mode)
{
    accton_as6700_32x_led_set(led_cdev, led_light_mode, cpld_reg[0], led_reg[0], LED_TYPE_ALARM);
}

static enum led_brightness accton_as6700_32x_led_alarm_get(struct led_classdev *cdev)
{
    accton_as6700_32x_led_update();
    return led_reg_val_to_light_mode(LED_TYPE_ALARM, ledctl->reg_val[0]);
}

static void accton_as6700_32x_led_fan1_set(struct led_classdev *led_cdev,
                                          enum led_brightness led_light_mode)
{
    accton_as6700_32x_led_set(led_cdev, led_light_mode, cpld_reg[1], led_reg[1], LED_TYPE_FAN1);
}

static enum led_brightness accton_as6700_32x_led_fan1_get(struct led_classdev *cdev)
{
    accton_as6700_32x_led_update();
    return led_reg_val_to_light_mode(LED_TYPE_FAN1, ledctl->reg_val[1]);
}

static void accton_as6700_32x_led_fan2_set(struct led_classdev *led_cdev,
                                          enum led_brightness led_light_mode)
{
    accton_as6700_32x_led_set(led_cdev, led_light_mode, cpld_reg[1], led_reg[1], LED_TYPE_FAN2);
}

static enum led_brightness accton_as6700_32x_led_fan2_get(struct led_classdev *cdev)
{
    accton_as6700_32x_led_update();
    return led_reg_val_to_light_mode(LED_TYPE_FAN2, ledctl->reg_val[1]);
}

static void accton_as6700_32x_led_fan3_set(struct led_classdev *led_cdev,
                                          enum led_brightness led_light_mode)
{
    accton_as6700_32x_led_set(led_cdev, led_light_mode, cpld_reg[1], led_reg[1], LED_TYPE_FAN3);
}

static enum led_brightness accton_as6700_32x_led_fan3_get(struct led_classdev *cdev)
{
    accton_as6700_32x_led_update();
    return led_reg_val_to_light_mode(LED_TYPE_FAN3, ledctl->reg_val[1]);
}

static void accton_as6700_32x_led_fan4_set(struct led_classdev *led_cdev,
                                          enum led_brightness led_light_mode)
{
    accton_as6700_32x_led_set(led_cdev, led_light_mode, cpld_reg[1], led_reg[1], LED_TYPE_FAN4);
}

static enum led_brightness accton_as6700_32x_led_fan4_get(struct led_classdev *cdev)
{
    accton_as6700_32x_led_update();
    return led_reg_val_to_light_mode(LED_TYPE_FAN4, ledctl->reg_val[1]);
}

static void accton_as6700_32x_led_fan5_set(struct led_classdev *led_cdev,
                                          enum led_brightness led_light_mode)
{
    accton_as6700_32x_led_set(led_cdev, led_light_mode, cpld_reg[1], led_reg[1], LED_TYPE_FAN1);
}

static enum led_brightness accton_as6700_32x_led_fan5_get(struct led_classdev *cdev)
{
    accton_as6700_32x_led_update();
    return led_reg_val_to_light_mode(LED_TYPE_FAN5, ledctl->reg_val[1]);
}


static struct led_classdev accton_as6700_32x_leds[7] = {
    [LED_TYPE_SYSTEM] = {
        .name             = "accton_as6700_32x_led::system",
        .default_trigger = "unused",
        .brightness_set     = accton_as6700_32x_led_system_set,
        .brightness_get  = accton_as6700_32x_led_system_get,
        .flags             = LED_CORE_SUSPENDRESUME,
        .max_brightness  = LED_MODE_SYSTEM_OFF,
    },
    [LED_TYPE_ALARM] = {
        .name             = "accton_as6700_32x_led::alarm",
        .default_trigger = "unused",
        .brightness_set     = accton_as6700_32x_led_alarm_set,
        .brightness_get  = accton_as6700_32x_led_alarm_get,
        .flags             = LED_CORE_SUSPENDRESUME,
        .max_brightness  = LED_MODE_ALARM_OFF,
    },
    [LED_TYPE_FAN1] = {
        .name             = "accton_as6700_32x_led::fan1",
        .default_trigger = "unused",
        .brightness_set     = accton_as6700_32x_led_fan1_set,
        .brightness_get  = accton_as6700_32x_led_fan1_get,
        .flags             = LED_CORE_SUSPENDRESUME,
        .max_brightness  = LED_MODE_FAN_RED,
    },
    [LED_TYPE_FAN2] = {
        .name             = "accton_as6700_32x_led::fan2",
        .default_trigger = "unused",
        .brightness_set     = accton_as6700_32x_led_fan2_set,
        .brightness_get  = accton_as6700_32x_led_fan2_get,
        .flags             = LED_CORE_SUSPENDRESUME,
        .max_brightness  = LED_MODE_FAN_RED,
    },
    [LED_TYPE_FAN3] = {
        .name             = "accton_as6700_32x_led::fan3",
        .default_trigger = "unused",
        .brightness_set     = accton_as6700_32x_led_fan3_set,
        .brightness_get  = accton_as6700_32x_led_fan3_get,
        .flags             = LED_CORE_SUSPENDRESUME,
        .max_brightness  = LED_MODE_FAN_RED,
    },
    [LED_TYPE_FAN4] = {
        .name             = "accton_as6700_32x_led::fan4",
        .default_trigger = "unused",
        .brightness_set     = accton_as6700_32x_led_fan4_set,
        .brightness_get  = accton_as6700_32x_led_fan4_get,
        .flags             = LED_CORE_SUSPENDRESUME,
        .max_brightness  = LED_MODE_FAN_RED,
    },
    [LED_TYPE_FAN5] = {
        .name             = "accton_as6700_32x_led::fan5",
        .default_trigger = "unused",
        .brightness_set     = accton_as6700_32x_led_fan5_set,
        .brightness_get  = accton_as6700_32x_led_fan5_get,
        .flags             = LED_CORE_SUSPENDRESUME,
        .max_brightness  = LED_MODE_FAN_RED,
    }
};

static int accton_as6700_32x_led_suspend(struct platform_device *dev,
        pm_message_t state)
{
    int i = 0;
    
    for (i = 0; i < ARRAY_SIZE(accton_as6700_32x_leds); i++) {
        led_classdev_suspend(&accton_as6700_32x_leds[i]);
    }

    return 0;
}

static int accton_as6700_32x_led_resume(struct platform_device *dev)
{
    int i = 0;
    
    for (i = 0; i < ARRAY_SIZE(accton_as6700_32x_leds); i++) {
        led_classdev_resume(&accton_as6700_32x_leds[i]);
    }

    return 0;
}

static int accton_as6700_32x_led_probe(struct platform_device *pdev)
{
    int ret, i;
    
    for (i = 0; i < ARRAY_SIZE(accton_as6700_32x_leds); i++) {
        ret = led_classdev_register(&pdev->dev, &accton_as6700_32x_leds[i]);
        
        if (ret < 0)
            break;
    }
    
    /* Check if all LEDs were successfully registered */
    if (i != ARRAY_SIZE(accton_as6700_32x_leds)){
        int j;
        
        /* only unregister the LEDs that were successfully registered */
        for (j = 0; j < i; j++) {
            led_classdev_unregister(&accton_as6700_32x_leds[i]);
        }
    }

    return ret;
}

static int accton_as6700_32x_led_remove(struct platform_device *pdev)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(accton_as6700_32x_leds); i++) {
        led_classdev_unregister(&accton_as6700_32x_leds[i]);
    }

    return 0;
}

static struct platform_driver accton_as6700_32x_led_driver = {
    .probe      = accton_as6700_32x_led_probe,
    .remove     = accton_as6700_32x_led_remove,
    .suspend    = accton_as6700_32x_led_suspend,
    .resume     = accton_as6700_32x_led_resume,
    .driver     = {
        .name   = DRVNAME,
        .owner  = THIS_MODULE,
    },
};

static int __init accton_as6700_32x_led_init(void)
{
    int ret;
    
    ret = platform_driver_register(&accton_as6700_32x_led_driver);
    if (ret < 0) {
        goto exit;
    }
        
    ledctl = kzalloc(sizeof(struct accton_as6700_32x_led_data), GFP_KERNEL);
    if (!ledctl) {
        ret = -ENOMEM;
        platform_driver_unregister(&accton_as6700_32x_led_driver);
        goto exit;
    }

    mutex_init(&ledctl->update_lock);
    
    ledctl->pdev = platform_device_register_simple(DRVNAME, -1, NULL, 0);
    if (IS_ERR(ledctl->pdev)) {
        ret = PTR_ERR(ledctl->pdev);
        platform_driver_unregister(&accton_as6700_32x_led_driver);
        kfree(ledctl);
        goto exit;
    }

exit:
    return ret;
}

static void __exit accton_as6700_32x_led_exit(void)
{
    platform_device_unregister(ledctl->pdev);
    platform_driver_unregister(&accton_as6700_32x_led_driver);
    kfree(ledctl);
}

module_init(accton_as6700_32x_led_init);
module_exit(accton_as6700_32x_led_exit);

MODULE_AUTHOR("Peter Huang <peter_huang@accton.com.tw>");
MODULE_DESCRIPTION("accton_as6700_32x_led driver");
MODULE_LICENSE("GPL");
