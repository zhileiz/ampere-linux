/*
 *   Copyright (c) International Business Machines Corp., 2006, 2010, 2012
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

/*
 * This file contains the architecture independent IIC FSI code.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fsi.h>
#include <asm/io.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include "iic-int.h"
#include "iic-fsi.h"

#include <linux/delay.h>
#include <linux/moduleparam.h>

struct class* iic_fsi_class = 0;
dev_t iic_devnum_start = 0;

static const char iic_fsi_version[] = "3.0";

int iic_fsi_probe(struct device *dev);
int iic_fsi_remove(struct device *dev);

struct iic_reg_access fsi_reg_access =
{
	.bus_readb = readb_wrap,
	.bus_readh = readh_wrap,
	.bus_readw = readw_wrap,
	.bus_writeb = writeb_wrap,
	.bus_writeh = writeh_wrap,
	.bus_writew = writew_wrap,
};

static const struct fsi_device_id i2c_ids[] = {
	{
		.engine_type = FSI_ENGID_I2C,
		.version = FSI_VERSION_ANY,
	},
	{
		.engine_type = FSI_ENGID_I2C_BB,
		.version = FSI_VERSION_ANY,
	},
	{ 0 }
};

static const struct fsi_driver i2c_drv = {
	.id_table = i2c_ids,
	.drv = {
		.name = "iic_fsi_dd",
		.bus = &fsi_bus_type,
		.probe = iic_fsi_probe,
		.remove = iic_fsi_remove,
	}
};

/*
 * Called when an FSI IIC engine is plugged in.  
 * Causes creation of the /dev entry.
 * Not allowed to access engine registers in this function.
 *
 */
int iic_fsi_probe(struct device *dev)
{
	return 0;
}

/* This function is called when a link is removed or the driver is unloaded.
 * It's job is to remove the device from the device hierarchy including 
 * removal from sysfs (this is where device files get removed).
 */
int iic_fsi_remove(struct device* dev)
{
	return 0;
}

/*
 * Initialize this module.  Creates a class for fsi connected iic devices and
 * allocates device numbers for them.
 */
static int __init iic_fsi_init(void)
{
	int rc = 0;

	IENTER();

	rc = alloc_chrdev_region(&iic_devnum_start,
			0, /*starting minor number*/
			IIC_FSI_MAX_DEVS,
			"iic-fsi");
	if(rc)
	{
		 IFLDe(1, "master alloc_chrdev_region failed: rc = %d\n", rc);
		 return rc;
	}

	iic_fsi_class = class_create(THIS_MODULE, "iic-fsi");
	if (IS_ERR(iic_fsi_class))
	{
		IFLDe(1, "class_create failed: rc=%ld", 
				PTR_ERR(iic_fsi_class));
		goto exit_class_create;
	}
	/* Register this driver with the FSI infrastructure */
	rc = fsi_driver_register(&i2c_drv);
	if(rc)
	{
		IFLDe(1, "fsidrv_register failed: %d", rc);
		goto exit_drv_register;
	}

	printk("IIC FSI support loaded, ver. %s\n", iic_fsi_version);

	IEXIT(rc);
	return rc;

exit_drv_register:
	class_destroy(iic_fsi_class);
exit_class_create:
	unregister_chrdev_region(iic_devnum_start, IIC_FSI_MAX_DEVS);
	IEXIT(rc);
	return rc;
}

static void __exit iic_fsi_exit(void)
{
	IENTER();
	IDBGd(0, "fisdrv_unregister()\n");
	fsi_driver_unregister(&i2c_drv);
	IDBGd(0, "unregister_chrdev_region()\n");
	unregister_chrdev_region(iic_devnum_start, IIC_FSI_MAX_DEVS);
	if(iic_fsi_class)
	{
		IDBGd(0, "class_destroy\n");
		class_destroy(iic_fsi_class);
	}
	printk("IIC FSI support unloaded.\n");
	IEXIT(0);
}

module_init(iic_fsi_init);
module_exit(iic_fsi_exit);

MODULE_AUTHOR("Eddie James <eajames@us.ibm.com>");
MODULE_DESCRIPTION("IIC FSI Driver");
MODULE_LICENSE("GPL");

