#ifndef __ACCTON_AS5710_48x_CPLD_H
#define __ACCTON_AS5710_48x_CPLD_H
/*
 * A hwmon driver for the accton_as5710_48x_cpld
 *
 * Copyright (C) 2013 Accton Technology Corporation.
 * Brandon Chuang <brandon_chuang@accton.com.tw>
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

#include <linux/types.h> 

int accton_as5710_48x_cpld_read(unsigned short cpld_addr, u8 reg);
int accton_as5710_48x_cpld_write(unsigned short cpld_addr, u8 reg, u8 value);

#endif /* __ACCTON_AS5710_48x_CPLD_H */
