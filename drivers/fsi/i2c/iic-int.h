/*
 *   Copyright (c) International Business Machines Corp., 2006, 2012
 *
 *   This program is free software;  you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY;  without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
 *   the GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program;  if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef IIC_INT_H
#define IIC_INT_H
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/fs.h>
#include <linux/kobject.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/wait.h>
#include <linux/i2cfsi.h>
#include <linux/semaphore.h>
#include <linux/fsi.h>
#include <asm/atomic.h>

#define FSI_ENGID_I2C		0x7
#define FSI_ENGID_I2C_BB	0x17

#ifdef FSI_I2C_DEBUG
#define IDBGs(num, msg, args...) printk(msg, ## args)
#define IDBGd(num, msg, args...) printk(msg, ## args)
#define IDBGf(num, msg, args...) printk(msg, ## args)
#define IDBGl(num, msg, args...) printk(msg, ## args)
#else
#define IDBGs(num, msg, args...)
#define IDBGd(num, msg, args...)
#define IDBGf(num, msg, args...)
#define IDBGl(num, msg, args...)
#endif

#define IENTER()
#define IEXIT(RC)

/* IFLDx traces will not get compiled out */
#define IFLDe(num, msg, args...)\
	printk("ERR: "msg, ## args)
#define IFLDi(num, msg, args...)\
	printk(msg, ## args)

#ifdef FSI_I2C_DEBUG
#define IFLDs(num, msg, args...) printk(msg, ## args)
#define IFLDd(num, msg, args...) printk(msg, ## args)
#define IFLDf(num, msg, args...) printk(msg, ## args)
#define IFLDl(num, msg, args...) printk(msg, ## args)
#else
#define IFLDs(num, msg, args...)
#define IFLDd(num, msg, args...)
#define IFLDf(num, msg, args...)
#define IFLDl(num, msg, args...)
#endif

struct iic_reg_access
{
    int (*bus_readb)(iic_eng_t*, unsigned int, unsigned char*, iic_ffdc_t**);
    int (*bus_readh)(iic_eng_t*, unsigned int, unsigned short*, iic_ffdc_t**);
    int (*bus_readw)(iic_eng_t*, unsigned int, unsigned long*, iic_ffdc_t**);
    int (*bus_writeb)(iic_eng_t*, unsigned int, unsigned char, iic_ffdc_t**);
    int (*bus_writeh)(iic_eng_t*, unsigned int, unsigned short, iic_ffdc_t**);
    int (*bus_writew)(iic_eng_t*, unsigned int, unsigned long, iic_ffdc_t**);
};

#endif
