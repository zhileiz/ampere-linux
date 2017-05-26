/*
 * Copyright 2017 IBM Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __OCC_COMMON_H__
#define __OCC_COMMON_H__

#include <linux/hwmon-sysfs.h>
#include <linux/sysfs.h>

#define OCC_RESP_DATA_BYTES		4089

struct occ_response {
	u8 seq_no;
	u8 cmd_type;
	u8 return_status;
	u16 data_length_be;
	u8 data[OCC_RESP_DATA_BYTES];
	u16 checksum_be;
} __packed;

struct occ_sensor_data_block_header {
	u8 eye_catcher[4];
	u8 reserved;
	u8 sensor_format;
	u8 sensor_length;
	u8 num_sensors;
} __packed;

struct occ_sensor_data_block {
	struct occ_sensor_data_block_header header;
	u32 data;
} __packed;

struct occ_poll_response_header {
	u8 status;
	u8 ext_status;
	u8 occs_present;
	u8 config_data;
	u8 occ_state;
	u8 mode;
	u8 ips_status;
	u8 error_log_id;
	u32 error_log_start_address_be;
	u16 error_log_length_be;
	u16 reserved;
	u8 occ_code_level[16];
	u8 eye_catcher[6];
	u8 num_sensor_data_blocks;
	u8 sensor_data_block_header_version;
} __packed;

struct occ_poll_response {
	struct occ_poll_response_header header;
	struct occ_sensor_data_block block;
} __packed;

struct occ_sensor {
	u8 num_sensors;
	void *data;
};

struct occ_sensors {
	struct occ_sensor temp;
	struct occ_sensor freq;
	struct occ_sensor power;
	struct occ_sensor caps;
};

struct occ {
	struct device *bus_dev;

	struct occ_response resp;
	struct occ_sensors sensors;
};

int occ_poll(struct occ *occ);

#endif /* __OCC_COMMON_H__ */
