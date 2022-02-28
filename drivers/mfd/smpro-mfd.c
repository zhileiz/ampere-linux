// SPDX-License-Identifier: GPL-2.0-only
/*
 * Ampere Altra Family SMPro MFD - I2C
 *
 * Copyright (c) 2022, Ampere Computing LLC
 * Author: Quan Nguyen <quan@os.amperecomputing..com>
 */

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/mfd/core.h>
#include <linux/regmap.h>

/* Identification Registers */
#define MANUFACTURER_ID_REG     0x02
#define AMPERE_MANUFACTURER_ID  0xCD3A

static int smpro_mfd_write(void *context, const void *data, size_t count)
{
	struct device *dev = context;
	struct i2c_client *i2c = to_i2c_client(dev);
	int ret;

	ret = i2c_master_send(i2c, data, count);
	if (ret == count)
		return 0;
	else if (ret < 0)
		return ret;
	else
		return -EIO;
}

static int smpro_mfd_read(void *context, const void *reg, size_t reg_size,
			  void *val, size_t val_size)
{
	struct device *dev = context;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct i2c_msg xfer[2];
	unsigned char buf[2];
	int ret;

	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;

	buf[0] = *(u8 *)reg;
	buf[1] = val_size;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	xfer[1].addr = i2c->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = val_size;
	xfer[1].buf = val;

	ret = i2c_transfer(i2c->adapter, xfer, 2);
	if (ret == 2)
		return 0;
	else if (ret < 0)
		return ret;
	else
		return -EIO;
}

static const struct regmap_bus smpro_regmap_bus = {
	.read = smpro_mfd_read,
	.write = smpro_mfd_write,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};

static bool smpro_mfd_readable_noinc_reg(struct device *dev, unsigned int reg)
{
	return  (reg == 0x82 || reg == 0x85 || reg == 0x92 || reg == 0x95 ||
		 reg == 0xC2 || reg == 0xC5 || reg == 0xD2 || reg == 0xDA);
}

static const struct regmap_config smpro_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.readable_noinc_reg = smpro_mfd_readable_noinc_reg,
};

static const struct mfd_cell smpro_devs[] = {
	MFD_CELL_NAME("smpro-hwmon"),
	MFD_CELL_NAME("smpro-errmon"),
	MFD_CELL_NAME("smpro-misc"),
};

static int smpro_mfd_probe(struct i2c_client *i2c)
{
	const struct regmap_config *config;
	struct regmap *regmap;
	unsigned int val;
	int ret;

	config = device_get_match_data(&i2c->dev);
	if (!config)
		config = &smpro_regmap_config;

	regmap = devm_regmap_init(&i2c->dev, &smpro_regmap_bus, &i2c->dev, config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	/* Check for valid ID */
	ret = regmap_read(regmap, MANUFACTURER_ID_REG, &val);
	if (ret)
		return ret;

	if (val != AMPERE_MANUFACTURER_ID)
		return -ENODEV;

	return devm_mfd_add_devices(&i2c->dev, PLATFORM_DEVID_AUTO,
				    smpro_devs, ARRAY_SIZE(smpro_devs), NULL, 0, NULL);
}

static const struct of_device_id smpro_mfd_of_match[] = {
	{ .compatible = "ampere,smpro", .data = &smpro_regmap_config },
	{}
};
MODULE_DEVICE_TABLE(of, smpro_mfd_of_match);

static struct i2c_driver smpro_mfd_driver = {
	.probe_new = smpro_mfd_probe,
	.driver = {
		.name = "smpro-mfd-i2c",
		.of_match_table = smpro_mfd_of_match,
	},
};
module_i2c_driver(smpro_mfd_driver);

MODULE_AUTHOR("Quan Nguyen <quan@os.amperecomputing.com>");
MODULE_DESCRIPTION("SMPRO MFD - I2C driver");
MODULE_LICENSE("GPL v2");
