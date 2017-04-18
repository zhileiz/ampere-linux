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
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#define OCC_SRAM_BYTES		4096
#define OCC_CMD_DATA_BYTES	4090
#define OCC_RESP_DATA_BYTES	4089

struct occfifo {
	struct sbefifo_drv_ref ref;
	struct sbefifo *sbefifo;
	char name[32];
	struct miscdevice mdev;
	struct list_head xfrs;
	spinlock_t list_lock;
	spinlock_t occ_lock;
	struct work_struct work;
};

#define to_occfifo(x)	container_of((x), struct occfifo, mdev)

struct occ_command {
	u8 seq_no;
	u8 cmd_type;
	u16 data_length;
	u8 data[OCC_CMD_DATA_BYTES];
	u16 checksum;
};

struct occ_response {
	u8 seq_no;
	u8 cmd_type;
	u8 return_status;
	u16 data_length;
	u8 data[OCC_RESP_DATA_BYTES];
	u16 checksum;
};

struct occfifo_xfr;

enum {
	CLIENT_NONBLOCKING,
};

struct occfifo_client {
	struct occfifo *occfifo;
	struct occfifo_xfr *xfr;
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

struct occfifo_xfr {
	struct list_head link;
	struct occfifo_client *client;
	int rc;
	u8 buf[OCC_SRAM_BYTES];
	size_t cmd_data_length;
	size_t resp_data_length;
	unsigned long flags;
};

static struct workqueue_struct *occfifo_wq;

static void occfifo_enqueue_xfr(struct occfifo_xfr *xfr)
{
	int empty;
	struct occfifo *occfifo = xfr->client->occfifo;

	spin_lock_irq(&occfifo->list_lock);
	empty = list_empty(&occfifo->xfrs);
	list_add_tail(&xfr->link, &occfifo->xfrs);
	spin_unlock(&occfifo->list_lock);

	if (empty)
		queue_work(occfifo_wq, &occfifo->work);
}

static int occfifo_open(struct inode *inode, struct file *file)
{
	struct occfifo_client *client;
	struct miscdevice *mdev = file->private_data;
	struct occfifo *occfifo = to_occfifo(mdev);

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	client->occfifo = occfifo;
	spin_lock_init(&client->lock);
	init_waitqueue_head(&client->wait);

	if (file->f_flags & O_NONBLOCK)
		set_bit(CLIENT_NONBLOCKING, &client->flags);

	file->private_data = client;

	return 0;
}

static ssize_t occfifo_read(struct file *file, char __user *buf, size_t len,
			    loff_t *offset)
{
	int rc;
	size_t bytes;
	struct occfifo_xfr *xfr;
	struct occfifo_client *client = file->private_data;

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
		}
		else
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

static ssize_t occfifo_write(struct file *file, const char __user *buf,
			     size_t len, loff_t *offset)
{
	int rc;
	struct occfifo_xfr *xfr;
	struct occfifo_client *client = file->private_data;

	if (!access_ok(VERIFY_READ, buf, len))
		return -EFAULT;

	if (len > OCC_SRAM_BYTES)
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

	if (copy_from_user(xfr->buf, buf, len)) {
		kfree(xfr);
		rc = -EFAULT;
		goto done;
	}

	xfr->client = client;
	xfr->cmd_data_length = len;
	client->xfr = xfr;
	client->read_offset = 0;

	occfifo_enqueue_xfr(xfr);

	rc = len;

done:
	spin_unlock(&client->lock);
	return rc;
}

static int occfifo_release(struct inode *inode, struct file *file)
{
	struct occfifo_xfr *xfr;
	struct occfifo_client *client = file->private_data;
	struct occfifo *occfifo = client->occfifo;

	spin_lock_irq(&client->lock);
	xfr = client->xfr;
	if (!xfr) {
		spin_unlock(&client->lock);
		kfree(client);
		return 0;
	}

	spin_lock_irq(&occfifo->list_lock);
	set_bit(XFR_CANCELED, &xfr->flags);
	if (!test_bit(XFR_IN_PROGRESS, &xfr->flags)) {
		/* already deleted from list if complete */
		if (!test_bit(XFR_COMPLETE, &xfr->flags))
			list_del(&xfr->link);

		spin_unlock(&occfifo->list_lock);

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
	spin_unlock(&occfifo->list_lock);
	spin_unlock(&client->lock);
	return 0;
}

static const struct file_operations occfifo_fops = {
	.owner = THIS_MODULE,
	.open = occfifo_open,
	.read = occfifo_read,
	.write = occfifo_write,
	.release = occfifo_release,
};

static int occfifo_getscom(struct sbefifo *sbefifo, u32 address, u8 *data)
{
	int rc;
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
		rc = -EIO;
		goto done;
	}

	rc = sbefifo_drv_read(client, (char *)buf, len);
	if (rc < 0)
		goto done;
	else if (rc != len) {
		rc = -EIO;
		goto done;
	}

	rc = 0;

	memcpy(data, buf, sizeof(u64));

done:
	sbefifo_drv_release(client);
	return rc;
}

static int occfifo_putscom(struct sbefifo *sbefifo, u32 address, u8 *data)
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
		rc = -EIO;
		goto done;
	}

	rc = sbefifo_drv_read(client, (char *)buf, sizeof(u32) * 4);
	if (rc < 0) {
		rc = 0;
		goto done;
	} else if (rc != sizeof(u32) * 4) {
		rc = -EIO;
		goto done;
	}

	rc = 0;

done:
	sbefifo_drv_release(client);
	return rc;
}

static int occfifo_putscom_u32(struct sbefifo *sbefifo, u32 address, u32 data0,
			       u32 data1)
{
	u8 buf[8];

	memcpy(buf, &data0, 4);
	memcpy(buf + 4, &data1, 4);

	return occfifo_putscom(sbefifo, address, buf);
}

static void occfifo_worker(struct work_struct *work)
{
	int i, empty, canceled, waiting, rc;
	u16 resp_data_length;
	struct occfifo *occfifo = container_of(work, struct occfifo, work);
	struct sbefifo *sbefifo = occfifo->sbefifo;
	struct occfifo_client *client;
	struct occfifo_xfr *xfr;
	struct occ_response *resp;

again:
	spin_lock_irq(&occfifo->list_lock);
	xfr = list_first_entry(&occfifo->xfrs, struct occfifo_xfr, link);
	if (!xfr) {
		spin_unlock(&occfifo->list_lock);
		return;
	}

	set_bit(XFR_IN_PROGRESS, &xfr->flags);
	spin_unlock(&occfifo->list_lock);

	resp = (struct occ_response *)xfr->buf;

	spin_lock_irq(&occfifo->occ_lock);

	/* set stream mode enabled */
	rc = occfifo_putscom_u32(sbefifo, 0x6D053, 0x08000000, 0);
	if (rc)
		goto done;

	/* set stream mode to linear */
	rc = occfifo_putscom_u32(sbefifo, 0x6D052, 0x04000000, 0);
	if (rc)
		goto done;

	/* set address reg to occ sram command buffer */
	rc = occfifo_putscom_u32(sbefifo, 0x6D050, 0xFFFBE000, 0);
	if (rc)
		goto done;

	/* write cmd data */
	for (i = 0; i < xfr->cmd_data_length; i += 8) {
		rc = occfifo_putscom(sbefifo, 0x6D055, &xfr->buf[i]);
		if (rc)
			goto done;
	}

	/* set stream mode enabled and stream mode to circular */
	rc = occfifo_putscom_u32(sbefifo, 0x6D033, 0x0C000000, 0);
	if (rc)
		goto done;

	/* trigger attention */
	rc = occfifo_putscom_u32(sbefifo, 0x6D035, 0x20010000, 0);
	if (rc)
		goto done;

	/* set address reg to occ sram response buffer */
	rc = occfifo_putscom_u32(sbefifo, 0x6D050, 0xFFFBF000, 0);
	if (rc)
		goto done;

	rc = occfifo_getscom(sbefifo, 0x6D055, xfr->buf);
	if (rc)
		goto done;

	xfr->resp_data_length += 8;

	resp_data_length = be16_to_cpu(get_unaligned(&resp->data_length));
	if (resp_data_length > OCC_RESP_DATA_BYTES) {
		rc = -EFAULT;
		goto done;
	}

	/* already read 3 bytes of resp data, but also need 2 bytes chksum */
	for (i = 8; i < resp_data_length + 7; i += 8) {
		rc = occfifo_getscom(sbefifo, 0x6D055, &xfr->buf[i]);
		if (rc)
			goto done;

		xfr->resp_data_length += 8;
	}

	/* no errors, got all data */
	xfr->resp_data_length = resp_data_length + 7;

done:
	spin_unlock(&occfifo->occ_lock);

	xfr->rc = rc;
	client = xfr->client;

	/* lock client to prevent race with read() */
	spin_lock_irq(&client->lock);
	set_bit(XFR_COMPLETE, &xfr->flags);
	waiting = test_bit(XFR_WAITING, &xfr->flags);
	spin_unlock(&client->lock);

	spin_lock_irq(&occfifo->list_lock);
	clear_bit(XFR_IN_PROGRESS, &xfr->flags);
	list_del(&xfr->link);
	empty = list_empty(&occfifo->xfrs);
	canceled = test_bit(XFR_CANCELED, &xfr->flags);
	spin_unlock(&occfifo->list_lock);

	if (waiting)
		wake_up_interruptible(&client->wait);
	else if (canceled) {
		kfree(xfr);
		kfree(xfr->client);
	}

	if (!empty)
		goto again;
}

void occfifo_notify(struct sbefifo_drv_ref *ref)
{
	struct occfifo *occfifo = container_of(ref, struct occfifo, ref);

	/* TODO: find better solution? this does seem to work if we lose
	 * the sbefifo in the middle of a transfer
	 */
	occfifo->sbefifo = NULL;
}

static int occfifo_probe(struct platform_device *pdev)
{
	int rc;
	struct occfifo *occfifo;
	struct device_node *bus;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;

	occfifo = devm_kzalloc(dev, sizeof(*occfifo), GFP_KERNEL);
	if (!occfifo)
		return -ENOMEM;

	bus = of_parse_phandle(node, "bus", 0);
	if (!bus) {
		dev_err(dev, "failed to get dts phandle\n");
		return -ENODEV;
	}

	occfifo->ref.notify = occfifo_notify;
	occfifo->sbefifo = sbefifo_drv_reference(bus, &occfifo->ref);
	if (!occfifo->sbefifo) {
		dev_err(dev, "failed to get sbefifo reference\n");
		rc = -ENODEV;
		goto done;
	}

	INIT_LIST_HEAD(&occfifo->xfrs);
	spin_lock_init(&occfifo->list_lock);
	spin_lock_init(&occfifo->occ_lock);
	INIT_WORK(&occfifo->work, occfifo_worker);

	snprintf(occfifo->name, sizeof(occfifo->name), "occfifo%d",
		 sbefifo_drv_get_idx(occfifo->sbefifo));

	occfifo->mdev.fops = &occfifo_fops;
	occfifo->mdev.minor = MISC_DYNAMIC_MINOR;
	occfifo->mdev.name = occfifo->name;
	occfifo->mdev.parent = dev;

	rc = misc_register(&occfifo->mdev);
	if (rc) {
		dev_err(dev, "failed to register miscdevice\n");
		goto done;
	}

	platform_set_drvdata(pdev, occfifo);

done:
	of_node_put(bus);
	return rc;
}

static int occfifo_remove(struct platform_device *pdev)
{
	struct occfifo_xfr *xfr, *tmp;
	struct occfifo *occfifo = platform_get_drvdata(pdev);
	struct occfifo_client *client;

	misc_deregister(&occfifo->mdev);

	spin_lock_irq(&occfifo->list_lock);
	list_for_each_entry_safe(xfr, tmp, &occfifo->xfrs, link) {
		client = xfr->client;
		set_bit(XFR_CANCELED, &xfr->flags);

		if (!test_bit(XFR_IN_PROGRESS, &xfr->flags)) {
			list_del(&xfr->link);

			spin_lock_irq(&client->lock);
			if (test_bit(XFR_WAITING, &xfr->flags)) {
				wake_up_interruptible(&client->wait);
				spin_unlock(&client->lock);
			}
			else {
				kfree(xfr);
				spin_unlock(&client->lock);
				kfree(client);
			}
		}
	}
	spin_unlock(&occfifo->list_lock);

	flush_work(&occfifo->work);
	list_del(&occfifo->ref.link);

	return 0;
}

static const struct of_device_id occfifo_match[] = {
	{ .compatible = "ibm,occfifo" },
	{ },
};

static struct platform_driver occfifo_driver = {
	.driver = {
		.name = "occfifo",
		.of_match_table	= occfifo_match,
	},
	.probe	= occfifo_probe,
	.remove = occfifo_remove,
};

static int occfifo_init(void)
{
	occfifo_wq = create_singlethread_workqueue("occfifo");
	if (!occfifo_wq)
		return -ENOMEM;

	return platform_driver_register(&occfifo_driver);
}

static void occfifo_exit(void)
{
	destroy_workqueue(occfifo_wq);

	platform_driver_unregister(&occfifo_driver);
}

module_init(occfifo_init);
module_exit(occfifo_exit);

MODULE_AUTHOR("Eddie James <eajames@us.ibm.com>");
MODULE_DESCRIPTION("BMC P9 OCC driver");
MODULE_LICENSE("GPL");
