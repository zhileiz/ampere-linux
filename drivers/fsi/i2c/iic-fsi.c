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
void iic_fsi_shutdown(struct device *dev);
int iic_fsi_suspend(struct device *dev);
int iic_fsi_resume(struct device *dev);
static void iic_eng_release(struct kobject* kobj);

/* callback function for when the reference count for an engine reaches 0 */
static void iic_eng_release(struct kobject* kobj)
{
	iic_eng_t* eng = container_of(kobj, iic_eng_t, kobj);
	struct device* dev = eng->dev;
	unsigned long flags;

	IENTER();

	/*remove all busses associated with the engine */
	spin_lock_irqsave(&eng->lock, flags);
	while(eng->busses){
		iic_bus_t* temp = eng->busses;
		eng->busses = temp->next;
		iic_delete_bus(iic_fsi_class, temp);
	}

	eng->enabled = 0x0ULL;
	spin_unlock_irqrestore(&eng->lock, flags);


	/* providing an arch specific cleanup routine is optional.
	 * if not specified, use the default.
	 */
	if (iic_eng_ops_is_vaild(eng->ops)) {
		if(eng->ops->cleanup_eng)
		{
			eng->ops->cleanup_eng(eng);
		}
		else
		{
			IDBGd(0, "free engine\n");
			kfree(eng);
		}
	}

	dev_set_drvdata(dev, 0);
	kobject_put(&dev->kobj);

	IEXIT(0);
}

static struct kobj_type iic_eng_ktype = {
	.release = iic_eng_release,
};

int readb_wrap(iic_eng_t* eng, unsigned int addr, unsigned char *val, 
	       iic_ffdc_t** ffdc)
{
	int rc;
	struct fsi_device *fsi_dev = to_fsi_dev(eng->dev);
	rc = fsi_device_read(fsi_dev, addr, val, 1);
	if(rc)
	{
		IFLDe(3, "eng[%08x]: fsi_readb_ffdc(%p)=%d\n", eng->id,
				addr, rc);
	}
	return rc;
};

int readh_wrap(iic_eng_t* eng, unsigned int addr, unsigned short *val, 
	       iic_ffdc_t** ffdc)
{
	int rc;
	struct fsi_device *fsi_dev = to_fsi_dev(eng->dev);
	rc = fsi_device_read(fsi_dev, addr, val, 2);
	if(rc)
	{
		IFLDe(3, "eng[%08x]: fsi_readh_ffdc(%p)=%d\n", eng->id,
				addr, rc);
	}
	return rc;
};

int readw_wrap(iic_eng_t* eng, unsigned int addr, unsigned long *val, 
	       iic_ffdc_t** ffdc)
{
	int rc;
	struct fsi_device *fsi_dev = to_fsi_dev(eng->dev);
	rc = fsi_device_read(fsi_dev, addr, val, 4);
	if(rc)
	{
		IFLDe(3, "eng[%08x]: fsi_readw_ffdc(%p)=%d\n", eng->id,
				addr, rc);
	}
	return rc;
};

int writeb_wrap(iic_eng_t* eng, unsigned int addr, unsigned char val, 
		iic_ffdc_t** ffdc)
{
	int rc;
	struct fsi_device *fsi_dev = to_fsi_dev(eng->dev);
	rc = fsi_device_write(fsi_dev, addr, &val, 1);
	if(rc)
	{
		IFLDe(3, "eng[%08x]: fsi_writeb_ffdc(%p)=%d\n", eng->id,
				addr, rc);
	}
	return rc;
};

int writeh_wrap(iic_eng_t* eng, unsigned int addr, unsigned short val,
	       	iic_ffdc_t** ffdc)
{
	int rc;
	struct fsi_device *fsi_dev = to_fsi_dev(eng->dev);
	rc = fsi_device_write(fsi_dev, addr, &val, 2);
	if(rc)
	{
		IFLDe(3, "eng[%08x]: fsi_writeh_ffdc(%p)=%d\n", eng->id,
				addr, rc);
	}
	return rc;
};

int writew_wrap(iic_eng_t* eng, unsigned int addr, unsigned long val, 
		iic_ffdc_t** ffdc)
{
	int rc;
	struct fsi_device *fsi_dev = to_fsi_dev(eng->dev);
	rc = fsi_device_write(fsi_dev, addr, &val, 4);
	if(rc)
	{
		IFLDe(3, "eng[%08x]: fsi_writew_ffdc(%p)=%d\n", eng->id,
				addr, rc);
	}
	return rc;
};

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

/* Adds ports to the eng->buses SLL.  Access to the SLL must be protected since
 * iic_fsi_remove or iic_eng_release could be called asynchronously and they
 * also traverse/modify the SLL.
 */
int iic_add_ports(iic_eng_t* eng, uint64_t ports)
{
	uint64_t ports_left;
	int bus_num = 0;
	unsigned long flags;
	iic_bus_t* new_bus = 0;
	int minor = 0;
	char name[64];
	int rc = 0;
	struct fsi_device *fdev = to_fsi_dev(eng->dev);

	IENTER();

	IFLDi(3, "Adding ports[0x%08x%08x] to eng[0x%08x]",
	      (uint32_t)(ports >> 32),
	      (uint32_t)ports,
	      eng->id);

	/* walk the mask adding master ports as needed */
	for(ports_left = ports; ports_left; ports_left = ports >> ++bus_num)
	{
		if(!(ports_left & 0x1))
			continue;


		if( minor < 0 ) {
			IFLDe(1, "bb_get_minor %d", minor);
			rc = minor;
			goto exit;
		}

		sprintf(name, "i2cfsi%02d", bus_num);

		/* results in hotplug event for each master bus */
		new_bus = iic_create_bus(iic_fsi_class, eng,
						MKDEV(MAJOR(iic_devnum_start), minor),
						name, bus_num, minor);
		if(!new_bus)
		{
			IFLDe(1, "iic_create_bus failed on eng %d", eng->id);
			rc = -ENODEV;
			goto exit;
		}

		/* atomically insert the new bus into the SLL unless
		 * iic_fsi_remove has been started. 
		 */
		spin_lock_irqsave(&eng->lock, flags);
		if(test_bit(IIC_ENG_REMOVED, &eng->flags))
		{
			/* if iic_fsi_remove has been started then
			 * don't add this bus to the SLL and delete it.
			 * Previously added buses will be removed by
			 * iic_eng_release
			 */
			rc = -ENODEV;
			iic_delete_bus(iic_fsi_class, new_bus);
			spin_unlock_irqrestore(&eng->lock, flags);
			goto exit;
		}
		else
		{
			new_bus->next = eng->busses;
			eng->busses = new_bus;
			eng->enabled |= 0x1ULL << bus_num;
		}
		spin_unlock_irqrestore(&eng->lock, flags);

		minor++;
	}

exit:
	IEXIT(rc);
	return rc;
}

/* Removes ports from the eng->buses SLL.  Access to the SLL must be protected
 * since iic_fsi_remove or iic_eng_release could be called at same time as this
 * and they also traverse/modify the SLL.
 */
int iic_del_ports(iic_eng_t* eng, uint64_t ports)
{
	unsigned long flags;
	iic_bus_t* abusp;
	iic_bus_t** p_abusp;

	IENTER();

	IFLDi(3, "removing ports[0x%08x%08x] from eng[0x%08x]",
	      (uint32_t)(ports >> 32),
	      (uint32_t)ports,
	      eng->id);

	/* walk unordered SLL and delete bus if it is in the ports bit mask */
	spin_lock_irqsave(&eng->lock, flags);
	if(test_bit(IIC_ENG_REMOVED, &eng->flags))
		goto exit;
	p_abusp = &eng->busses;
	abusp = *p_abusp;
	while(abusp)
	{
		if(ports & (0x1ULL << abusp->port))
		{
			/* found a match, remove it */
			*p_abusp = abusp->next;
			eng->enabled &= ~(0x1ULL << abusp->port);
			device_destroy(iic_fsi_class, abusp->devnum);
			iic_delete_bus(iic_fsi_class, abusp);
		}
		else
		{
			/* not a match, skip to next one */
			p_abusp = &abusp->next;
		}
		abusp = *p_abusp;
	}

exit:
	spin_unlock_irqrestore(&eng->lock, flags);
	IEXIT(0);
	return 0;
}

/*
 * Called when an FSI IIC engine is plugged in.  
 * Causes creation of the /dev entry.
 * Not allowed to access engine registers in this function.
 *
 */
int iic_fsi_probe(struct device *dev)
{
	uint64_t new_ports = 0;
	int rc = 0;
	struct fsi_device *dp = to_fsi_dev(dev);
	iic_eng_t* eng = 0;

	IENTER();

	eng = (iic_eng_t*)kmalloc(sizeof(iic_eng_t), GFP_KERNEL);

	if(!eng)
	{
		IFLDe(0, "Couldn't malloc engine\n");
		rc = -ENOMEM;
		goto error;
	}

	memset(eng, 0, sizeof(*eng));
	iic_init_eng(eng);
	set_bit(IIC_ENG_BLOCK, &eng->flags); //block until resumed
	eng->id = 0x00F5112C;
	IFLDi(1, "PROBE    eng[%08x]", eng->id);
	eng->ra = &fsi_reg_access;
	IFLDd(1, "vaddr=%#08lx\n", eng->base);
	eng->dev = dev;
	// The new kernel now requires 2 arguments
	kobject_init(&eng->kobj, &iic_eng_ktype); //ref count = 1
	eng->ops = iic_get_eng_ops(FSI_ENGID_I2C);
	if(!eng->ops)
	{
		IFLDi(1, "support for engine type 0x%08x not loaded.\n",
				FSI_ENGID_I2C);
		rc = -ENODEV;
		goto error;
	}
	
	/* Register interrupt handler with the kernel */	
	IDBGd(0, "requesting irq\n");
	dp->irq_handler = eng->ops->int_handler;

	IFLDd(1, "irq  = %d\n", eng->irq);

	new_ports = 0xFFFULL;
	set_bit(IIC_ENG_P8_Z8_CENTAUR, &eng->flags);


	/* Add master and slave ports to the engine */
	rc = iic_add_ports(eng, new_ports);
	if(rc)
		goto error;

	dev_set_drvdata(dev, eng);
	eng->private_data = 0; //unused


	/* set the callback function for when the eng ref count reaches 0 */
	kobject_get(&eng->dev->kobj);

	iic_fsi_resume(dev);

error:
	if(rc)
	{
		IFLDi(1, "IIC: iic_fsi_probe failed: %d\n", rc);
		while(eng && eng->busses){
			iic_bus_t* temp = eng->busses;
			eng->busses = temp->next;
			iic_delete_bus(iic_fsi_class, temp);
		}
		if(eng)
		{
			kfree(eng);
		}
	}
	else
	{
		IFLDd(1, "engine 0x%08X probed.\n", eng->id);
	}

	IEXIT(rc);
	return rc;
}

/* This function is called when a link is removed or the driver is unloaded.
 * It's job is to remove the device from the device hierarchy including 
 * removal from sysfs (this is where device files get removed).
 */
int iic_fsi_remove(struct device* dev)
{
	int rc = 0;
	iic_bus_t* bus;
	iic_eng_t* eng = (iic_eng_t*)dev_get_drvdata(dev);
	unsigned long flags;

	IENTER();
       
	if(!eng)
	{
		IFLDe(1, "iic_fsi_remove called with bad dev: %p\n", dev);
		rc = -1;
		goto error;
	}

	/* set ENG_REMOVED flag so that aborted operations have status
	 * set to ENOLINK (lost fsi link) instead of ENODEV (no lbus).
	 */
	set_bit(IIC_ENG_REMOVED, &eng->flags);

	iic_fsi_suspend(dev); //ignore rc

	IFLDi(1, "REMOVE   eng[%08x]\n", eng->id);

	/* Clean up device files immediately, don't wait for ref count */
	spin_lock_irqsave(&eng->lock, flags);
	bus = eng->busses;
	while(bus)
	{
		/* causes hot unplug event */
		device_destroy(iic_fsi_class, bus->devnum);
		bus = bus->next;
	}
	spin_unlock_irqrestore(&eng->lock, flags);

	/* cleans up engine and bus structures if ref count is zero */
	kobject_put(&eng->kobj);
	
error:
	IEXIT(0);
	return 0;
}

/* This function is called when a link is removed or the driver is unloaded.
 * It's job is to quiesce and disable the hardware if possible and unregister 
 * interrupts. It always precedes the remove function.
 *
 * The device may be in the resumed or suspended state when this function is
 * called.
 *
 * This function is no longer called for mcp5
 */
void iic_fsi_shutdown(struct device *dev)
{
	int rc = 0;
	iic_eng_t* eng = (iic_eng_t*)dev_get_drvdata(dev);
	struct fsi_device* fsidev = to_fsi_dev(dev);

	IENTER();
	if(!eng || !eng->ops)
	{
		rc = -1;
		goto error;
	}
	IFLDi(1, "SHUTDOWN eng[%08x]\n", eng->id);

	/* set ENG_REMOVED flag so that aborted operations have status
	 * set to ENOLINK (lost fsi link) instead of ENODEV (no lbus).
	 */
	set_bit(IIC_ENG_REMOVED, &eng->flags);

	iic_fsi_suspend(dev);
	
error:
	if(rc)
	{
		IFLDe(1, "iic_fsi_shutdown failed: %d\n", rc);
	}
	IEXIT(0);
	return;
}

/* This function is called when we loose ownership or are preparing to give
 * up ownership of the local bus.  If we still own lbus, then we try to
 * gracefully halt any pending transfer.  No hot unplug events are caused by
 * this function.
 *
 * This function also is called from iic_fsi_remove.
 *
 * Note: In the case where we lose local bus and then loose the link or
 * get rmmod'd, this function could be called twice without a resume
 * in the middle!
 */
int iic_fsi_suspend(struct device *dev)
{
	int rc = 0;
	iic_eng_t* eng = (iic_eng_t*)dev_get_drvdata(dev);
	unsigned long xfr_status;

	IENTER();
	if(!eng)
	{
		rc = -1;
		goto error;
	}

	IFLDi(2, "SUSPEND  eng[%08x]\n", eng->id);

	/* Prohibit new engine operations until resumed*/
	set_bit(IIC_ENG_BLOCK, &eng->flags);


	/* Terminate pending operations (set status to ENODEV)
	 * If we have access to the engine, halt any transfers 
	 * and Disable engine interrupts.
	 */

	/* lost lbus -> ENODEV, hot unplug -> ENOLINK */
	xfr_status = test_bit(IIC_ENG_REMOVED, &eng->flags)? -ENOLINK: -ENODEV;
	iic_abort_all(eng, 0, xfr_status);

	/* disable slave transfers */
	if (iic_eng_ops_is_vaild(eng->ops)) {
		if(eng->ops->slv_off) {
			eng->ops->slv_off(eng, 0);
		}
	}

	/* prohibit hw access using this engine object */
	set_bit(IIC_NO_ACCESS, &eng->flags);
	
	/* disable interrupt handler if not already done */
	if(test_and_clear_bit(IIC_ENG_RESUMED, &eng->flags))
	{
		fsi_disable_irq(to_fsi_dev(dev)); 
	}

error:	
	IEXIT(rc);
	return rc;
}

/* This function is called when we are given (back) ownership of the local
 * bus.
 */
int iic_fsi_resume(struct device *dev)
{
	iic_ffdc_t* ffdc = 0;
	int rc = 0;
	iic_eng_t* eng = 0;
	struct fsi_device *dp = to_fsi_dev(dev);

	IENTER();	
	// The device structure has changed for the new kernel.
	// The member driver_data has been deprecated.
	eng = (iic_eng_t*)dev_get_drvdata(dev);

	if(!eng || !iic_eng_ops_is_vaild(eng->ops))
	{
		rc = -EIO;
		goto error;
	}

	IFLDi(1, "RESUME   eng[%08x]\n", eng->id);

	eng->bus_speed = 20833333;
	IFLDd(1, "eng->bus_speed=%ld\n", eng->bus_speed);

	/* Reset the engine */
	rc = eng->ops->reset_eng(eng, &ffdc);
	if(rc)
	{
		goto error;
	}

	/* Give the engine time to determine the state of the bus before
	 * allowing transfers to begin after resetting the engine.
	 */
	udelay(200);

	/* Initialize the engine */
	rc = eng->ops->init(eng, &ffdc);
	if(rc)
	{
		goto error;
	}

	/* Enable interrupt handler in the kernel */	
	IDBGd(0, "enabling irq\n");
	rc = fsi_enable_irq(dp);
	if(rc)
	{
		IFLDe(1, "fsi_enable_irq failed rc=%d\n", rc);
		goto error;
	}

	set_bit(IIC_ENG_RESUMED, &eng->flags);

	/* allow operations to be submitted */
	clear_bit(IIC_ENG_BLOCK, &eng->flags);
	goto exit;

error:
	IFLDe(1, "iic_fsi_resume failed, rc=%d\n", rc);
exit:
	IEXIT(rc);
	return rc;
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

