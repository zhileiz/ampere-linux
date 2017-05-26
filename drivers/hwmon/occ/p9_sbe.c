/*
 * Copyright 2017 IBM Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "common.h"
#include <linux/hwmon.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/occ.h>
#include <linux/sched.h>
#include <linux/workqueue.h>

#define OCC_TIMEOUT_MS			5000
#define OCC_CMD_IN_PRG_MS		100

#define RESP_RETURN_CMD_IN_PRG		0xFF
#define RESP_RETURN_SUCCESS		0
#define RESP_RETURN_CMD_INVAL		0x11
#define RESP_RETURN_CMD_LEN		0x12
#define RESP_RETURN_DATA_INVAL		0x13
#define RESP_RETURN_CHKSUM		0x14
#define RESP_RETURN_OCC_ERR		0x15
#define RESP_RETURN_STATE		0x16

struct p9_sbe_occ {
	struct occ occ;
	struct device *sbe;
};

#define to_p9_sbe_occ(x)	container_of((x), struct p9_sbe_occ, occ)

static int p9_sbe_occ_send_cmd(struct occ *occ, u8 *cmd)
{
	int rc;
	unsigned long start;
	struct occ_client *client;
	struct occ_response *resp = &occ->resp;
	struct p9_sbe_occ *p9_sbe_occ = to_p9_sbe_occ(occ);

	start = jiffies;

retry:
	client = occ_drv_open(p9_sbe_occ->sbe, 0);
	if (!client)
		return -ENODEV;

	rc = occ_drv_write(client, (const char *)&cmd[1], 7);
	if (rc < 0)
		goto err;

	rc = occ_drv_read(client, (char *)resp, sizeof(*resp));
	if (rc < 0)
		goto err;

	occ_drv_release(client);

	switch (resp->return_status) {
	case RESP_RETURN_CMD_IN_PRG:
		if (time_after(jiffies, start + msecs_to_jiffies(OCC_TIMEOUT_MS)))
			rc = -EALREADY;
		else {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(msecs_to_jiffies(OCC_CMD_IN_PRG_MS));

			goto retry;
		}
		break;
	case RESP_RETURN_SUCCESS:
		rc = 0;
		break;
	case RESP_RETURN_CMD_INVAL:
	case RESP_RETURN_CMD_LEN:
	case RESP_RETURN_DATA_INVAL:
	case RESP_RETURN_CHKSUM:
		rc = -EINVAL;
		break;
	case RESP_RETURN_OCC_ERR:
		rc = -EREMOTE;
		break;
	default:
		rc = -EFAULT;
	}

	if (rc < 0) {
		dev_warn(occ->bus_dev, "occ bad response:%d\n",
			 resp->return_status);
		return rc;
	}

	return 0;

err:
	occ_drv_release(client);
	dev_err(occ->bus_dev, "occ bus op failed rc:%d\n", rc);
	return rc;
}

static int p9_sbe_occ_probe(struct platform_device *pdev)
{
	struct occ *occ;
	struct p9_sbe_occ *p9_sbe_occ = devm_kzalloc(&pdev->dev,
						     sizeof(*p9_sbe_occ),
						     GFP_KERNEL);
	if (!p9_sbe_occ)
		return -ENOMEM;

	p9_sbe_occ->sbe = pdev->dev.parent;

	occ = &p9_sbe_occ->occ;
	occ->bus_dev = &pdev->dev;
	occ->poll_cmd_data = 0x20;
	occ->send_cmd = p9_sbe_occ_send_cmd;
	mutex_init(&occ->lock);
	platform_set_drvdata(pdev, p9_sbe_occ);

	return 0;
}

static const struct of_device_id p9_sbe_occ_of_match[] = {
	{ .compatible = "ibm,p9-occ-hwmon" },
	{ },
};

static struct platform_driver p9_sbe_occ_driver = {
	.driver = {
		.name = "occ-hwmon",
		.of_match_table	= p9_sbe_occ_of_match,
	},
	.probe	= p9_sbe_occ_probe,
};

module_platform_driver(p9_sbe_occ_driver);

MODULE_AUTHOR("Eddie James <eajames@us.ibm.com>");
MODULE_DESCRIPTION("BMC P9 OCC hwmon driver");
MODULE_LICENSE("GPL");
