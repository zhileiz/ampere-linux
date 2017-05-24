/*
 * Copyright 2017 IBM Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <asm/unaligned.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fsi-sbefifo.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#define OCC_SRAM_BYTES		4096
#define OCC_CMD_DATA_BYTES	4090
#define OCC_RESP_DATA_BYTES	4089

struct occ {
	struct device *sbefifo;
	char name[32];
	int idx;
	struct miscdevice mdev;
	struct list_head xfrs;
	spinlock_t list_lock;
	spinlock_t occ_lock;
	struct work_struct work;
};

#define to_occ(x)	container_of((x), struct occ, mdev)

struct occ_command {
	u8 seq_no;
	u8 cmd_type;
	u16 data_length;
	u8 data[OCC_CMD_DATA_BYTES];
	u16 checksum;
} __packed;

struct occ_response {
	u8 seq_no;
	u8 cmd_type;
	u8 return_status;
	u16 data_length;
	u8 data[OCC_RESP_DATA_BYTES];
	u16 checksum;
} __packed;

struct occ_xfr;

enum {
	CLIENT_NONBLOCKING,
};

struct occ_client {
	struct occ *occ;
	struct occ_xfr *xfr;
	spinlock_t lock;
	wait_queue_head_t wait;
	size_t read_offset;
	unsigned long flags;
};

enum {
	XFR_IN_PROGRESS,
	XFR_COMPLETE,
	XFR_CANCELED,
	XFR_WAITING,
};

struct occ_xfr {
	struct list_head link;
	struct occ_client *client;
	int rc;
	u8 buf[OCC_SRAM_BYTES];
	size_t cmd_data_length;
	size_t resp_data_length;
	unsigned long flags;
};

static struct workqueue_struct *occ_wq;

static DEFINE_IDA(occ_ida);

static void occ_enqueue_xfr(struct occ_xfr *xfr)
{
	int empty;
	struct occ *occ = xfr->client->occ;

	spin_lock_irq(&occ->list_lock);
	empty = list_empty(&occ->xfrs);
	list_add_tail(&xfr->link, &occ->xfrs);
	spin_unlock(&occ->list_lock);

	if (empty)
		queue_work(occ_wq, &occ->work);
}

static int occ_open(struct inode *inode, struct file *file)
{
	struct occ_client *client;
	struct miscdevice *mdev = file->private_data;
	struct occ *occ = to_occ(mdev);

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	client->occ = occ;
	spin_lock_init(&client->lock);
	init_waitqueue_head(&client->wait);

	if (file->f_flags & O_NONBLOCK)
		set_bit(CLIENT_NONBLOCKING, &client->flags);

	file->private_data = client;

	return 0;
}

static ssize_t occ_read(struct file *file, char __user *buf, size_t len,
			loff_t *offset)
{
	int rc;
	size_t bytes;
	struct occ_xfr *xfr;
	struct occ_client *client = file->private_data;

	if (!access_ok(VERIFY_WRITE, buf, len))
		return -EFAULT;

	if (len > OCC_SRAM_BYTES)
		return -EINVAL;

	spin_lock_irq(&client->lock);
	if (!client->xfr) {
		/* we just finished reading all data, return 0 */
		if (client->read_offset) {
			rc = 0;
			client->read_offset = 0;
		} else
			rc = -ENOMSG;

		goto done;
	}

	xfr = client->xfr;

	if (!test_bit(XFR_COMPLETE, &xfr->flags)) {
		if (client->flags & CLIENT_NONBLOCKING) {
			rc = -ERESTARTSYS;
			goto done;
		}

		set_bit(XFR_WAITING, &xfr->flags);
		spin_unlock(&client->lock);

		rc = wait_event_interruptible(client->wait,
			test_bit(XFR_COMPLETE, &xfr->flags) ||
			test_bit(XFR_CANCELED, &xfr->flags));

		spin_lock_irq(&client->lock);
		if (test_bit(XFR_CANCELED, &xfr->flags)) {
			kfree(xfr);
			spin_unlock(&client->lock);
			kfree(client);
			return -EBADFD;
		}

		clear_bit(XFR_WAITING, &xfr->flags);
		if (!test_bit(XFR_COMPLETE, &xfr->flags)) {
			rc = -EINTR;
			goto done;
		}
	}

	if (xfr->rc) {
		rc = xfr->rc;
		goto done;
	}

	bytes = min(len, xfr->resp_data_length - client->read_offset);
	if (copy_to_user(buf, &xfr->buf[client->read_offset], bytes)) {
		rc = -EFAULT;
		goto done;
	}

	client->read_offset += bytes;

	/* xfr done */
	if (client->read_offset == xfr->resp_data_length) {
		kfree(xfr);
		client->xfr = NULL;
	}

	rc = bytes;

done:
	spin_unlock(&client->lock);
	return rc;
}

static ssize_t occ_write(struct file *file, const char __user *buf,
			 size_t len, loff_t *offset)
{
	int rc;
	unsigned int i, j;
	u8 tmp;
	u16 data_length, checksum = 0;
	struct occ_xfr *xfr;
	struct occ_client *client = file->private_data;

	if (!access_ok(VERIFY_READ, buf, len))
		return -EFAULT;

	if (len > (OCC_CMD_DATA_BYTES + 3) || len < 3)
		return -EINVAL;

	spin_lock_irq(&client->lock);
	if (client->xfr) {
		rc = -EDEADLK;
		goto done;
	}

	xfr = kzalloc(sizeof(*xfr), GFP_KERNEL);
	if (!xfr) {
		rc = -ENOMEM;
		goto done;
	}

	xfr->buf[0] = 1;

	if (copy_from_user(&xfr->buf[1], buf, len)) {
		kfree(xfr);
		rc = -EFAULT;
		goto done;
	}

	data_length = (xfr->buf[2] << 8) + xfr->buf[3];
	if (data_length > OCC_CMD_DATA_BYTES) {
		kfree(xfr);
		rc = -EINVAL;
		goto done;
	}

	for (i = 0; i < data_length + 4; ++i)
		checksum += xfr->buf[i];

	xfr->buf[data_length + 4] = checksum >> 8;
	xfr->buf[data_length + 5] = checksum & 0xFF;

	/* byte swap the whole buffer */
	for (i = 0; i < data_length + 6; i += 4) {
		tmp = xfr->buf[i + 3];
		xfr->buf[i + 3] = xfr->buf[i];
		xfr->buf[i] = tmp;

		tmp = xfr->buf[i + 2];
		xfr->buf[i + 2] = xfr->buf[i + 1];
		xfr->buf[i + 1] = tmp;
	}

	xfr->client = client;
	xfr->cmd_data_length = data_length + 6;
	client->xfr = xfr;
	client->read_offset = 0;

	occ_enqueue_xfr(xfr);

	rc = len;

done:
	spin_unlock(&client->lock);
	return rc;
}

static int occ_release(struct inode *inode, struct file *file)
{
	struct occ_xfr *xfr;
	struct occ_client *client = file->private_data;
	struct occ *occ = client->occ;

	spin_lock_irq(&client->lock);
	xfr = client->xfr;
	if (!xfr) {
		spin_unlock(&client->lock);
		kfree(client);
		return 0;
	}

	spin_lock_irq(&occ->list_lock);
	set_bit(XFR_CANCELED, &xfr->flags);
	if (!test_bit(XFR_IN_PROGRESS, &xfr->flags)) {
		/* already deleted from list if complete */
		if (!test_bit(XFR_COMPLETE, &xfr->flags))
			list_del(&xfr->link);

		spin_unlock(&occ->list_lock);

		if (test_bit(XFR_WAITING, &xfr->flags)) {
			/* blocking read; let reader clean up */
			wake_up_interruptible(&client->wait);
			spin_unlock(&client->lock);
			return 0;
		}

		kfree(xfr);
		spin_unlock(&client->lock);
		kfree(client);
		return 0;
	}

	/* operation is in progress; let worker clean up*/
	spin_unlock(&occ->list_lock);
	spin_unlock(&client->lock);
	return 0;
}

static const struct file_operations occ_fops = {
	.owner = THIS_MODULE,
	.open = occ_open,
	.read = occ_read,
	.write = occ_write,
	.release = occ_release,
};

static int occ_getscom(struct device *sbefifo, u32 address, u8 *data)
{
	int i, rc;
	u32 buf[4];
	struct sbefifo_client *client;
	const size_t len = sizeof(buf);

	buf[0] = 0x4;
	buf[1] = 0xa201;
	buf[2] = 0;
	buf[3] = address;

	client = sbefifo_drv_open(sbefifo, 0);
	if (!client)
		return -ENODEV;

	rc = sbefifo_drv_write(client, (const char *)buf, len);
	if (rc < 0)
		goto done;
	else if (rc != len) {
		rc = -EMSGSIZE;
		goto done;
	}

	rc = sbefifo_drv_read(client, (char *)buf, len);
	if (rc < 0)
		goto done;
	else if (rc != len) {
		rc = -EMSGSIZE;
		goto done;
	}

	/* check for good response */
	if ((buf[2] >> 16) != 0xC0DE) {
		rc = -EFAULT;
		goto done;
	}

	rc = 0;

	for (i = 0; i < 4; ++i) {
		data[i] = ((u8 *)buf)[3 - i];
		data[i + 4] = ((u8 *)buf)[7 - i];
	}

done:
	sbefifo_drv_release(client);
	return rc;
}

static int occ_putscom(struct device *sbefifo, u32 address, u8 *data)
{
	int rc;
	u32 buf[6];
	struct sbefifo_client *client;
	const size_t len = sizeof(buf);

	buf[0] = 0x6;
	buf[1] = 0xa202;
	buf[2] = 0;
	buf[3] = address;
	memcpy(&buf[4], data, sizeof(u64));

	client = sbefifo_drv_open(sbefifo, 0);
	if (!client)
		return -ENODEV;

	rc = sbefifo_drv_write(client, (const char *)buf, len);
	if (rc < 0)
		goto done;
	else if (rc != len) {
		rc = -EMSGSIZE;
		goto done;
	}

	rc = 0;

	/* ignore response */
	sbefifo_drv_read(client, (char *)buf, len);

done:
	sbefifo_drv_release(client);
	return rc;
}

static int occ_putscom_u32(struct device *sbefifo, u32 address, u32 data0,
			   u32 data1)
{
	u8 buf[8];

	memcpy(buf, &data0, 4);
	memcpy(buf + 4, &data1, 4);

	return occ_putscom(sbefifo, address, buf);
}

static void occ_worker(struct work_struct *work)
{
	int i, empty, canceled, waiting, rc;
	u16 resp_data_length;
	struct occ *occ = container_of(work, struct occ, work);
	struct device *sbefifo = occ->sbefifo;
	struct occ_client *client;
	struct occ_xfr *xfr;
	struct occ_response *resp;

again:
	spin_lock_irq(&occ->list_lock);
	xfr = list_first_entry(&occ->xfrs, struct occ_xfr, link);
	if (!xfr) {
		spin_unlock(&occ->list_lock);
		return;
	}

	set_bit(XFR_IN_PROGRESS, &xfr->flags);
	spin_unlock(&occ->list_lock);

	resp = (struct occ_response *)xfr->buf;

	spin_lock_irq(&occ->occ_lock);

	/* set address reg to occ sram command buffer */
	rc = occ_putscom_u32(sbefifo, 0x6D050, 0xFFFBE000, 0);
	if (rc)
		goto done;

	/* write cmd data */
	for (i = 0; i < xfr->cmd_data_length; i += 8) {
		rc = occ_putscom(sbefifo, 0x6D055, &xfr->buf[i]);
		if (rc)
			goto done;
	}

	/* trigger attention */
	rc = occ_putscom_u32(sbefifo, 0x6D035, 0x20010000, 0);
	if (rc)
		goto done;

	/* set address reg to occ sram response buffer */
	rc = occ_putscom_u32(sbefifo, 0x6D050, 0xFFFBF000, 0);
	if (rc)
		goto done;

	rc = occ_getscom(sbefifo, 0x6D055, xfr->buf);
	if (rc)
		goto done;

	xfr->resp_data_length += 8;

	resp_data_length = (xfr->buf[3] << 8) + xfr->buf[4];
	if (resp_data_length > OCC_RESP_DATA_BYTES) {
		rc = -EDOM;
		goto done;
	}

	/* already read 3 bytes of resp data, but also need 2 bytes chksum */
	for (i = 8; i < resp_data_length + 7; i += 8) {
		rc = occ_getscom(sbefifo, 0x6D055, &xfr->buf[i]);
		if (rc)
			goto done;

		xfr->resp_data_length += 8;
	}

	/* no errors, got all data */
	xfr->resp_data_length = resp_data_length + 7;

done:
	spin_unlock(&occ->occ_lock);

	xfr->rc = rc;
	client = xfr->client;

	/* lock client to prevent race with read() */
	spin_lock_irq(&client->lock);
	set_bit(XFR_COMPLETE, &xfr->flags);
	waiting = test_bit(XFR_WAITING, &xfr->flags);
	spin_unlock(&client->lock);

	spin_lock_irq(&occ->list_lock);
	clear_bit(XFR_IN_PROGRESS, &xfr->flags);
	list_del(&xfr->link);
	empty = list_empty(&occ->xfrs);
	canceled = test_bit(XFR_CANCELED, &xfr->flags);
	spin_unlock(&occ->list_lock);

	if (waiting)
		wake_up_interruptible(&client->wait);
	else if (canceled) {
		kfree(xfr);
		kfree(xfr->client);
	}

	if (!empty)
		goto again;
}

static int occ_probe(struct platform_device *pdev)
{
	int rc, child_idx = 0;
	u32 reg;
	struct occ *occ;
	struct device_node *np;
	struct platform_device *child;
	struct device *dev = &pdev->dev;
	char child_name[32];

	dev_info(dev, "Found occ device\n");
	occ = devm_kzalloc(dev, sizeof(*occ), GFP_KERNEL);
	if (!occ)
		return -ENOMEM;

	occ->sbefifo = dev->parent;
	INIT_LIST_HEAD(&occ->xfrs);
	spin_lock_init(&occ->list_lock);
	spin_lock_init(&occ->occ_lock);
	INIT_WORK(&occ->work, occ_worker);

	if (dev->of_node) {
		rc = of_property_read_u32(dev->of_node, "reg", &reg);
		if (!rc) {
			/* make sure we don't have a duplicate from dts */
			occ->idx = ida_simple_get(&occ_ida, reg, reg + 1,
						  GFP_KERNEL);
			if (occ->idx < 0)
				occ->idx = ida_simple_get(&occ_ida, 1, INT_MAX,
							  GFP_KERNEL);
		} else
			occ->idx = ida_simple_get(&occ_ida, 1, INT_MAX,
						  GFP_KERNEL);
	} else
		occ->idx = ida_simple_get(&occ_ida, 1, INT_MAX, GFP_KERNEL);

	snprintf(occ->name, sizeof(occ->name), "occ%d", occ->idx);
	occ->mdev.fops = &occ_fops;
	occ->mdev.minor = MISC_DYNAMIC_MINOR;
	occ->mdev.name = occ->name;
	occ->mdev.parent = dev;

	rc = misc_register(&occ->mdev);
	if (rc) {
		dev_err(dev, "failed to register miscdevice\n");
		return rc;
	}

	platform_set_drvdata(pdev, occ);

	return 0;
}

static int occ_remove(struct platform_device *pdev)
{
	struct occ_xfr *xfr, *tmp;
	struct occ *occ = platform_get_drvdata(pdev);
	struct occ_client *client;

	misc_deregister(&occ->mdev);

	spin_lock_irq(&occ->list_lock);
	list_for_each_entry_safe(xfr, tmp, &occ->xfrs, link) {
		client = xfr->client;
		set_bit(XFR_CANCELED, &xfr->flags);

		if (!test_bit(XFR_IN_PROGRESS, &xfr->flags)) {
			list_del(&xfr->link);

			spin_lock_irq(&client->lock);
			if (test_bit(XFR_WAITING, &xfr->flags)) {
				wake_up_interruptible(&client->wait);
				spin_unlock(&client->lock);
			} else {
				kfree(xfr);
				spin_unlock(&client->lock);
				kfree(client);
			}
		}
	}
	spin_unlock(&occ->list_lock);

	flush_work(&occ->work);

	ida_simple_remove(&occ_ida, occ->idx);

	return 0;
}

static const struct of_device_id occ_match[] = {
	{ .compatible = "ibm,p9-occ" },
	{ },
};

static struct platform_driver occ_driver = {
	.driver = {
		.name = "occ",
		.of_match_table	= occ_match,
	},
	.probe	= occ_probe,
	.remove = occ_remove,
};

static int occ_init(void)
{
	occ_wq = create_singlethread_workqueue("occ");
	if (!occ_wq)
		return -ENOMEM;

	return platform_driver_register(&occ_driver);
}

static void occ_exit(void)
{
	destroy_workqueue(occ_wq);

	platform_driver_unregister(&occ_driver);
}

module_init(occ_init);
module_exit(occ_exit);

MODULE_AUTHOR("Eddie James <eajames@us.ibm.com>");
MODULE_DESCRIPTION("BMC P9 OCC driver");
MODULE_LICENSE("GPL");
