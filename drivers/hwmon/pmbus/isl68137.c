// SPDX-License-Identifier: GPL-2.0+
/*
 * Hardware monitoring driver for Intersil ISL68137
 *
 * Copyright (c) 2017 Google Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/hwmon-sysfs.h>
#include "pmbus.h"

#define ISL68137_VOUT_AVS 0x30

static struct pmbus_driver_info isl68137_info = {
	.pages = 2,
	.format[PSC_VOLTAGE_IN] = direct,
	.format[PSC_VOLTAGE_OUT] = direct,
	.format[PSC_CURRENT_IN] = direct,
	.format[PSC_CURRENT_OUT] = direct,
	.format[PSC_POWER] = direct,
	.format[PSC_TEMPERATURE] = direct,
	.m[PSC_VOLTAGE_IN] = 1,
	.b[PSC_VOLTAGE_IN] = 0,
	.R[PSC_VOLTAGE_IN] = 3,
	.m[PSC_VOLTAGE_OUT] = 1,
	.b[PSC_VOLTAGE_OUT] = 0,
	.R[PSC_VOLTAGE_OUT] = 3,
	.m[PSC_CURRENT_IN] = 1,
	.b[PSC_CURRENT_IN] = 0,
	.R[PSC_CURRENT_IN] = 2,
	.m[PSC_CURRENT_OUT] = 1,
	.b[PSC_CURRENT_OUT] = 0,
	.R[PSC_CURRENT_OUT] = 1,
	.m[PSC_POWER] = 1,
	.b[PSC_POWER] = 0,
	.R[PSC_POWER] = 0,
	.m[PSC_TEMPERATURE] = 1,
	.b[PSC_TEMPERATURE] = 0,
	.R[PSC_TEMPERATURE] = 0,
	.func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_IIN | PMBUS_HAVE_PIN
	    | PMBUS_HAVE_STATUS_INPUT | PMBUS_HAVE_TEMP | PMBUS_HAVE_TEMP2
	    | PMBUS_HAVE_TEMP3 | PMBUS_HAVE_STATUS_TEMP
	    | PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT
	    | PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT | PMBUS_HAVE_POUT,
	.func[1] = PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT
	    | PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT | PMBUS_HAVE_POUT,
};

static ssize_t isl68137_avs_enable_show_page(struct i2c_client *client,
					     int page,
					     char *buf)
{
	int val = pmbus_read_byte_data(client, page, PMBUS_OPERATION);

	return sprintf(buf, "%d\n",
		       (val & ISL68137_VOUT_AVS) == ISL68137_VOUT_AVS ? 1 : 0);
}

static ssize_t isl68137_avs_enable_store_page(struct i2c_client *client,
					      int page,
					      const char *buf, size_t count)
{
	int rc, op_val;

	if (count < 1) {
		rc = -EINVAL;
		goto out;
	}

	switch (buf[0]) {
	case '0':
		op_val = 0;
		break;
	case '1':
		op_val = ISL68137_VOUT_AVS;
		break;
	default:
		rc = -EINVAL;
		goto out;
	}

	/*
	 * Writes to VOUT setpoint over AVSBus will persist after the VRM is
	 * switched to PMBus control. Switching back to AVSBus control
	 * restores this persisted setpoint rather than re-initializing to
	 * PMBus VOUT_COMMAND. Writing VOUT_COMMAND first over PMBus before
	 * enabling AVS control is the workaround.
	 */
	if (op_val == ISL68137_VOUT_AVS) {
		int vout_command = pmbus_read_word_data(client, page,
							PMBUS_VOUT_COMMAND);
		rc = pmbus_write_word_data(client, page, PMBUS_VOUT_COMMAND,
					   vout_command);
		if (rc)
			goto out;
	}

	rc = pmbus_update_byte_data(client, page, PMBUS_OPERATION,
				    ISL68137_VOUT_AVS, op_val);

out:
	return rc < 0 ? rc : count;
}

static ssize_t isl68137_avs_enable_show(struct device *dev,
					struct device_attribute *devattr,
					char *buf)
{
	struct i2c_client *client = kobj_to_i2c_client(&dev->kobj);
	struct sensor_device_attribute_2 *attr = to_sensor_dev_attr_2(devattr);

	return isl68137_avs_enable_show_page(client, attr->index, buf);
}

static ssize_t isl68137_avs_enable_store(struct device *dev,
				struct device_attribute *devattr,
				const char *buf, size_t count)
{
	struct i2c_client *client = kobj_to_i2c_client(&dev->kobj);
	struct sensor_device_attribute_2 *attr = to_sensor_dev_attr_2(devattr);

	return isl68137_avs_enable_store_page(client, attr->index, buf, count);
}

static SENSOR_DEVICE_ATTR_2(avs0_enabled, 0644,
			    isl68137_avs_enable_show, isl68137_avs_enable_store,
			    0, 0);
static SENSOR_DEVICE_ATTR_2(avs1_enabled, 0644,
			    isl68137_avs_enable_show, isl68137_avs_enable_store,
			    0, 1);

static int isl68137_remove(struct i2c_client *client);

static int isl68137_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int rc;

	rc = pmbus_do_probe(client, id, &isl68137_info);
	if (rc)
		return rc;

	rc = device_create_file(&client->dev,
				&sensor_dev_attr_avs0_enabled.dev_attr);
	if (rc)
		goto out_fail;
	rc = device_create_file(&client->dev,
				&sensor_dev_attr_avs1_enabled.dev_attr);
	if (rc)
		goto out_fail;

	return rc;

out_fail:
	isl68137_remove(client);
	return rc;
}

static int isl68137_remove(struct i2c_client *client)
{
	device_remove_file(&client->dev,
			   &sensor_dev_attr_avs1_enabled.dev_attr);
	device_remove_file(&client->dev,
			   &sensor_dev_attr_avs0_enabled.dev_attr);
	return pmbus_do_remove(client);
}

static const struct i2c_device_id isl68137_id[] = {
	{"isl68137", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, isl68137_id);

/* This is the driver that will be inserted */
static struct i2c_driver isl68137_driver = {
	.driver = {
		   .name = "isl68137",
		   },
	.probe = isl68137_probe,
	.remove = isl68137_remove,
	.id_table = isl68137_id,
};

module_i2c_driver(isl68137_driver);

MODULE_AUTHOR("Maxim Sloyko <maxims@google.com>");
MODULE_DESCRIPTION("PMBus driver for Intersil ISL68137");
MODULE_LICENSE("GPL");
