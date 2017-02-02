/*
 *   Copyright (c) International Business Machines Corp., 2006, 2009, 2010
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/sysfs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/param.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <asm/page.h>
#include <linux/pagemap.h>
#include <linux/aio.h>
#include "iic-int.h"
#include <linux/fsi.h>
#include <linux/time.h>
#include <asm/io.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>

typedef struct
{
	unsigned long type;
	iic_eng_ops_t *ops;
	struct list_head list;
} iic_eng_type_t;

typedef struct
{
	unsigned long type;
	iic_bus_t * bus;
	struct list_head list;
} iic_bus_type_t;

/* This is a DLL of engine ops for the engines that are supported */
LIST_HEAD(iic_eng_type_list);

/* DLL for bus ops for the busses that are supported */
LIST_HEAD(iic_bus_type_list);

iic_opts_t iic_dflt_opts =
{
	.xfr_opts =
	{
		.dev_addr = 0,		/* 8 bits with LSB ignored */
		.dev_width = 0,		/* offset width in bytes */
		.offset = 0,		/* offset in bytes */
		.inc_addr = 0,		/* address increment mask */
		.timeout = 5000,	/* transfer timeout in milliseconds */
		.wdelay = 0,		/* write delay in milliseconds */
		.rdelay = 0,		/* read delay in milliseconds */ 
		.wsplit = 0,		/* write split chunk size (bytes) */
		.rsplit = 0,		/* read split chunk size (bytes) */
		.flags = 0,
	},
	.recovery =
	{
		.redo_pol = 0,
		.redo_delay = 0,
	}
};

static const char iic_mstr_version[] = "3.0";

/* save off the default cdev type pointer so we can call the default cdev
 * release function in our own bus release function
 */
static struct kobj_type* cdev_dflt_type = 0;
struct kobj_type iic_bus_type;

/* funtion called when cdev object (embedded in bus object) ref count
 * reaches zero.  (prevents cdev memory from being freed to early)
 */
void iic_bus_release(struct kobject* kobj)
{
	struct cdev *p = container_of(kobj, struct cdev, kobj);
	iic_bus_t* bus = container_of(p, iic_bus_t, cdev);

	IFLDi(1, "deleting bus[%08lx]\n", bus->bus_id);
	if(cdev_dflt_type && cdev_dflt_type->release)
		cdev_dflt_type->release(kobj);
	kfree(bus);
}

int iic_open(struct inode* inode, struct file* filp);
int iic_release(struct inode* inode, struct file* filp);
ssize_t iic_read(struct file *file, char __user *buf, size_t count,
		 loff_t *offset);
ssize_t iic_write(struct file *file, const char __user *buf, size_t count,
	       	  loff_t *offset);
ssize_t iic_aio_read(struct kiocb *iocb, const struct iovec *buf, unsigned long count, loff_t pos);
ssize_t iic_aio_write(struct kiocb *iocb, const struct iovec *buf, unsigned long count, loff_t pos);
long iic_ioctl(struct file *file, unsigned int cmd,
              unsigned long arg);
static int iic_mmap(struct file* file, struct vm_area_struct* vma);
int iic_xfr(iic_client_t* client, char* buf, size_t count, loff_t* offset, 
		char read_flag);
loff_t iic_llseek(struct file *filp, loff_t off, int whence);

struct file_operations iic_fops = {
	.owner = THIS_MODULE,
	.open = iic_open,
	.release = iic_release,
	.read = iic_read,
	.write = iic_write,
	.unlocked_ioctl = iic_ioctl,
	.llseek = iic_llseek,
	.mmap = iic_mmap,
};

static iic_bus_t * iic_get_bus(unsigned long port, unsigned long type)
{
	iic_bus_type_t* iterator;
	int found = 0;

	IENTER();

	list_for_each_entry(iterator, &iic_bus_type_list, list)
	{
		if((iterator->type == type) && (iterator->bus->port == port))
		{
			found = 1;
			break;
		}
	}

	IEXIT(0);
	return (found == 1)? iterator->bus: NULL;
}

/* Abort all pending xfrs for a client, or if client is 0, abort all
 * pending xfrs for the engine.  
 */
int iic_abort_all(iic_eng_t* eng, iic_client_t* client, int status)
{
	return 0;
}
EXPORT_SYMBOL(iic_abort_all);

int iic_register_eng_ops(iic_eng_ops_t* new_ops, unsigned long type)
{
	iic_eng_type_t* new_type = (iic_eng_type_t*)
		kmalloc(sizeof(iic_eng_type_t), GFP_KERNEL);
	IENTER();
	if(!new_type)
	{
		return -ENOMEM;
	}
	
	new_type->type = type;
	new_type->ops = new_ops;

	/* Add this eng type object to beginning of engine type list*/
	list_add(&new_type->list, &iic_eng_type_list);
	IDBGd(1, "eng type %08lx registered\n", type);
	IEXIT(0);
	return 0;
}
EXPORT_SYMBOL(iic_register_eng_ops);

int iic_unregister_eng_ops(unsigned long type)
{
	iic_eng_type_t *iterator, *found;
	IENTER();
	found = 0;

	list_for_each_entry(iterator, &iic_eng_type_list, list)
	{
		if(iterator->type == type)
		{
			found = iterator;
			break;
		}
	}
	if(found)
	{
		list_del(&found->list);
		kfree(found);
		IDBGd(1, "engine type %08lx unregistered\n", type);
	}
	IEXIT(0);
	return 0;
}
EXPORT_SYMBOL(iic_unregister_eng_ops);

void iic_register_bus(iic_bus_t * new_bus, unsigned long type)
{
        iic_bus_type_t* new_type = (iic_bus_type_t*)
                kmalloc(sizeof(iic_bus_type_t), GFP_KERNEL);

        IENTER();

        new_type->type = type;
        new_type->bus = new_bus;
        list_add(&new_type->list, &iic_bus_type_list);

        IEXIT(0);
}
EXPORT_SYMBOL(iic_register_bus);

void iic_unregister_bus(iic_bus_t *bus, unsigned long type)
{
        iic_bus_type_t *iterator, *temp;

        IENTER();

        list_for_each_entry_safe(iterator, temp, &iic_bus_type_list, list)
        {
                if((iterator->type == type) && (iterator->bus == bus))
                {
                        list_del(&iterator->list);
                        kfree(iterator);
                }
        }
        IEXIT(0);
}
EXPORT_SYMBOL(iic_unregister_bus);

void iic_init_eng(iic_eng_t* eng)
{
	IENTER();
	spin_lock_init(&eng->lock);
	sema_init(&eng->sem, 1);
	INIT_LIST_HEAD(&eng->xfrq);
	eng->cur_xfr = 0;
	iic_lck_mgr_init(&eng->lck_mgr);
	init_waitqueue_head(&eng->waitq);
	INIT_WORK(&eng->work, iic_finish_abort);
	atomic_set(&eng->xfr_num, 0);
	IEXIT(0);
}
EXPORT_SYMBOL(iic_init_eng);

int iic_eng_ops_is_vaild(struct iic_eng_ops *ops)
{
	int found = 0;
	iic_eng_type_t *iterator;

	list_for_each_entry(iterator, &iic_eng_type_list, list)
	{
		if(iterator->ops == ops)
		{
			found = 1;
			break;
		}
	}

	return found;
}
EXPORT_SYMBOL(iic_eng_ops_is_vaild);

struct iic_eng_ops* iic_get_eng_ops(unsigned long type)
{
	iic_eng_type_t *iterator;
	iic_eng_ops_t *found = 0;
	IENTER();

	/* return the eng ops for the given type of engine */
	list_for_each_entry(iterator, &iic_eng_type_list, list)
	{
		if(iterator->type == type)
		{
			found = iterator->ops;
			break;
		}
	}
	IEXIT((int)found);
	return found;
}
EXPORT_SYMBOL(iic_get_eng_ops);

/* called when an ffdc q for a bus is unlocked */
void iic_ffdc_q_unlocked(int scope, void* data)
{
	iic_eng_t* eng = (iic_eng_t*)data;
	unsigned long flags;
	if(eng)
	{
		spin_lock_irqsave(&eng->lock, flags);
		iic_start_next_xfr(eng);
		spin_unlock_irqrestore(&eng->lock, flags);
	}
}

/* Register this bus's minor number with the kernel and add
 * it to the iic class in sysfs so that a hotplug event is
 * sent to udev.  The sysfs name needs to be unique because
 * all entries are placed in the same directory.  udev
 * will take care of creating the correct /dev name.
 */
#define IIC_BUS_MAX_FFDC 4
iic_bus_t*  iic_create_bus(struct class* classp, iic_eng_t* eng,
			   dev_t devnum, char* name, unsigned char port,
			   unsigned long bus_id)
{
	int rc = 0;
	iic_bus_t* bus = 0;

	IENTER();

	if(!eng)
	{
		goto exit;
	}
	bus = (iic_bus_t*)kmalloc(sizeof(iic_bus_t), GFP_KERNEL);
	if(!bus)
	{
		goto exit;
	}
	memset(bus, 0, sizeof(iic_bus_t));
	bus->port = port;
	bus->bus_id = bus_id;
	bus->eng = eng;
	bus->devnum = devnum;
	bus->i2c_hz = 400000;
	cdev_init(&bus->cdev, &iic_fops); // ref count = 1
	/* since cdev is embedded in our bus structure, override the cdev
	 * cleanup function with our own so that the bus object doesn't get
	 * freed until the cdev ref count goes to zero.
	 */
	if(!cdev_dflt_type)
	{
		cdev_dflt_type = bus->cdev.kobj.ktype;
		memcpy(&iic_bus_type, cdev_dflt_type, sizeof(iic_bus_type));
		iic_bus_type.release = iic_bus_release;
	}
	bus->cdev.kobj.ktype = &iic_bus_type;
	kobject_set_name(&bus->cdev.kobj, name);
	rc = cdev_add(&bus->cdev, devnum, 1);
	if(rc)
	{
		IFLDe(1, "cdev_add failed for bus %08lx\n", bus->bus_id);
		kobject_put(&bus->cdev.kobj);
		goto exit_cdev_add;
	}

	bus->class_dev = device_create(classp, NULL,
			devnum,
			bus->eng->dev,
			(const char *)"%s",
			name);
	if(!bus->class_dev)
	{
		IFLDe(1, "device create failed, %08lx\n", bus->bus_id);
		goto exit_class_add;
	}

	IFLDi(1, "bus[%08lx] created\n", bus->bus_id);
	goto exit;

exit_q_create:
	device_destroy(classp, bus->devnum);
exit_class_add:
	cdev_del(&bus->cdev);
exit_cdev_add:
	bus = 0;
exit:
	IEXIT((int)bus);
	return bus;
}
EXPORT_SYMBOL(iic_create_bus);

void iic_delete_bus(struct class* classp, iic_bus_t* bus)
{
	IENTER();

	if(!bus)
	{
		goto exit;
	}
	IFLDi(1, "cleanup bus[%08lx]\n", bus->bus_id);
	cdev_del(&bus->cdev);
exit:
	IEXIT(0);
	return;
}
EXPORT_SYMBOL(iic_delete_bus);

static int __init iic_init(void)
{
	int rc = 0;

	IENTER();
	printk("IIC: base support loaded  ver. %s\n", iic_mstr_version);
	IEXIT(rc);
	return rc;
}

static void __exit iic_exit(void)
{
	IENTER();
	printk("IIC: base support unloaded.\n");
	IEXIT(0);
}

static int iic_set_trc_sz(const char* val, struct kernel_param *kp)
{
	int rc = param_set_int(val, kp);
	if(rc)
		return rc;
	return 0;
}

module_init(iic_init);
module_exit(iic_exit);
MODULE_AUTHOR("Eddie James <eajames@us.ibm.com>");
MODULE_DESCRIPTION("Base IIC Driver");
MODULE_LICENSE("GPL");
