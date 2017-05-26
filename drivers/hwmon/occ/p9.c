/*
 * Copyright 2017 IBM Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <asm/unaligned.h>
#include "common.h"

struct p9_temp_sensor {
	u32 sensor_id;
	u8 fru_type;
	u8 value;
} __packed;

struct p9_freq_sensor {
	u32 sensor_id;
	u16 value;
} __packed;

struct p9_power_sensor {
	u32 sensor_id;
	u8 function_id;
	u8 apss_channel;
	u16 reserved;
	u32 update_tag;
	u64 accumulator;
	u16 value;
} __packed;

struct p9_caps_sensor {
	u16 curr_powercap;
	u16 curr_powerreading;
	u16 norm_powercap;
	u16 max_powercap;
	u16 min_powercap;
	u16 user_powerlimit;
	u8 user_powerlimit_source;
} __packed;

static ssize_t p9_occ_show_temp(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int rc;
	u32 val = 0;
	struct p9_temp_sensor *temp;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	temp = ((struct p9_temp_sensor *)sensors->temp.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = be32_to_cpu(get_unaligned(&temp->sensor_id));
		break;
	case 1:
		val = temp->fru_type;
		break;
	case 2:
		/* millidegree */
		val = temp->value * 1000;
		break;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%u\n", val);
}

static ssize_t p9_occ_show_freq(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int rc;
	u32 val = 0;
	struct p9_freq_sensor *freq;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	freq = ((struct p9_freq_sensor *)sensors->freq.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = be32_to_cpu(get_unaligned(&freq->sensor_id));
		break;
	case 1:
		val = be16_to_cpu(get_unaligned(&freq->value));
		break;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%u\n", val);
}

static ssize_t p9_occ_show_power(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	int rc;
	u64 val = 0;
	struct p9_power_sensor *power;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	power = ((struct p9_power_sensor *)sensors->power.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = be32_to_cpu(get_unaligned(&power->sensor_id));
		break;
	case 1:
		val = power->function_id;
		break;
	case 2:
		val = power->apss_channel;
		break;
	case 3:
		val = be32_to_cpu(get_unaligned(&power->update_tag));
		break;
	case 4:
		val = be64_to_cpu(get_unaligned(&power->accumulator));
		break;
	case 5:
		val = be16_to_cpu(get_unaligned(&power->value));
		break;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%llu\n", val);
}

static ssize_t p9_occ_show_caps(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int rc;
	u16 val = 0;
	struct p9_caps_sensor *caps;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	caps = ((struct p9_caps_sensor *)sensors->caps.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = be16_to_cpu(get_unaligned(&caps->curr_powercap));
		break;
	case 1:
		val = be16_to_cpu(get_unaligned(&caps->curr_powerreading));
		break;
	case 2:
		val = be16_to_cpu(get_unaligned(&caps->norm_powercap));
		break;
	case 3:
		val = be16_to_cpu(get_unaligned(&caps->max_powercap));
		break;
	case 4:
		val = be16_to_cpu(get_unaligned(&caps->min_powercap));
		break;
	case 5:
		val = be16_to_cpu(get_unaligned(&caps->user_powerlimit));
		break;
	case 6:
		val = caps->user_powerlimit_source;
		break;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%u\n", val);
}

static ssize_t p9_occ_store_caps_user(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	int rc;
	u16 user_power_cap;
	struct occ *occ = dev_get_drvdata(dev);

	rc = kstrtou16(buf, 0, &user_power_cap);
	if (rc)
		return rc;

	rc = occ_set_user_power_cap(occ, user_power_cap);
	if (rc)
		return rc;

	return count;
}

int p9_occ_setup_sensor_attrs(struct occ *occ)
{
	unsigned int i, s;
	struct device *dev = occ->bus_dev;
	struct occ_sensors *sensors = &occ->sensors;
	struct occ_attribute *attr;

	occ->num_attrs = (sensors->temp.num_sensors * 3);
	occ->num_attrs += (sensors->freq.num_sensors * 2);
	occ->num_attrs += (sensors->power.num_sensors * 6);
	occ->num_attrs += (sensors->caps.num_sensors * 7);

	occ->attrs = devm_kzalloc(dev, sizeof(*occ->attrs) * occ->num_attrs,
				  GFP_KERNEL);
	if (!occ->attrs)
		return -ENOMEM;

	occ->group.attrs = devm_kzalloc(dev, sizeof(*occ->group.attrs) *
					occ->num_attrs + 1, GFP_KERNEL);
	if (!occ->group.attrs)
		return -ENOMEM;

	attr = occ->attrs;
	for (i = 0; i < sensors->temp.num_sensors; ++i) {
		s = i + 1;

		snprintf(attr->name, sizeof(attr->name), "temp%d_label", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
					     p9_occ_show_temp, NULL, 0, i);
		attr++;

		snprintf(attr->name, sizeof(attr->name), "temp%d_fru_type", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
					     p9_occ_show_temp, NULL, 1, i);
		attr++;

		snprintf(attr->name, sizeof(attr->name), "temp%d_input", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
					     p9_occ_show_temp, NULL, 2, i);
		attr++;
	}

	for (i = 0; i < sensors->freq.num_sensors; ++i) {
		s = i + 1;

		snprintf(attr->name, sizeof(attr->name), "freq%d_label", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
					     p9_occ_show_freq, NULL, 0, i);
		attr++;

		snprintf(attr->name, sizeof(attr->name), "freq%d_input", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
					     p9_occ_show_freq, NULL, 1, i);
		attr++;
	}

	for (i = 0; i < sensors->power.num_sensors; ++i) {
		s = i + 1;

		snprintf(attr->name, sizeof(attr->name), "power%d_label", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
					     p9_occ_show_power, NULL, 0, i);
		attr++;

		snprintf(attr->name, sizeof(attr->name), "power%d_function_id",
			 s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
					     p9_occ_show_power, NULL, 1, i);
		attr++;

		snprintf(attr->name, sizeof(attr->name),
			 "power%d_apss_channel", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
					     p9_occ_show_power, NULL, 2, i);
		attr++;

		snprintf(attr->name, sizeof(attr->name), "power%d_update_tag",
			 s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
					     p9_occ_show_power, NULL, 3, i);
		attr++;

		snprintf(attr->name, sizeof(attr->name), "power%d_accumulator",
			 s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
					     p9_occ_show_power, NULL, 4, i);
		attr++;

		snprintf(attr->name, sizeof(attr->name), "power%d_input", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
					     p9_occ_show_power, NULL, 5, i);
		attr++;
	}

	for (i = 0; i < sensors->caps.num_sensors; ++i) {
		s = i + 1;

		snprintf(attr->name, sizeof(attr->name), "caps%d_curr", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
					     p9_occ_show_caps, NULL, 0, i);
		attr++;

		snprintf(attr->name, sizeof(attr->name), "caps%d_reading", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
					     p9_occ_show_caps, NULL, 1, i);
		attr++;

		snprintf(attr->name, sizeof(attr->name), "caps%d_norm", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
					     p9_occ_show_caps, NULL, 2, i);
		attr++;

		snprintf(attr->name, sizeof(attr->name), "caps%d_max", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
					     p9_occ_show_caps, NULL, 3, i);
		attr++;

		snprintf(attr->name, sizeof(attr->name), "caps%d_min", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
					     p9_occ_show_caps, NULL, 4, i);
		attr++;

		snprintf(attr->name, sizeof(attr->name), "caps%d_user", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0644,
					     p9_occ_show_caps,
					     p9_occ_store_caps_user, 5, i);
		attr++;

		snprintf(attr->name, sizeof(attr->name), "caps%d_user_source",
			 s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
					     p9_occ_show_caps, NULL, 6, i);
		attr++;
	}

	/* put the sensors in the group */
	for (i = 0; i < occ->num_attrs; ++i)
		occ->group.attrs[i] = &occ->attrs[i].sensor.dev_attr.attr;

	return 0;
}
