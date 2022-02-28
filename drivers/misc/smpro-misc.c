// SPDX-License-Identifier: GPL-2.0+
/*
 * Ampere Computing SoC's SMpro Misc Driver
 *
 * Copyright (c) 2019-2020, Ampere Computing LLC
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

/* Boot Stage/Progress Registers */
#define BOOT_STAGE_SELECT	0xB0
#define BOOT_STAGE_STATUS_LO	0xB1
#define BOOT_STAGE_CUR_STAGE	0xB2
#define BOOT_STAGE_STATUS_HI	0xB3

/* SOC State Registers */
#define SOC_POWER_LIMIT		0xE5

/* Boot stages */
enum {
	BOOT_STAGE_SMPRO = 0,
	BOOT_STAGE_PMPRO,
	BOOT_STAGE_ATF_BL1,
	BOOT_STAGE_DDR_INIT,
	BOOT_STAGE_DDR_INIT_PROGRESS,
	BOOT_STAGE_ATF_BL2,
	BOOT_STAGE_ATF_BL31,
	BOOT_STAGE_ATF_BL32,
	BOOT_STAGE_UEFI,
	BOOT_STAGE_OS,
	BOOT_STAGE_MAX
};

struct reg_addr {
	char dev_name[128];
	s16 addr;
};

struct reg_addr reg_addrs[2] = {
	{ "", -1 },
	{ "", -1 }
};

s16 get_addr(char *dev_name)
{
	if (strcmp(reg_addrs[0].dev_name, dev_name) == 0)
		return reg_addrs[0].addr;
	else if (strcmp(reg_addrs[1].dev_name, dev_name) == 0)
		return reg_addrs[1].addr;

	return -1;
}

int find_dev(char *dev_name)
{
	if (strcmp(reg_addrs[0].dev_name, dev_name) == 0 ||
	    strcmp(reg_addrs[1].dev_name, dev_name) == 0)
		return 1;
	return 0;
}

int init_addr(char *dev_name)
{
	if (strcmp(reg_addrs[0].dev_name, "") == 0) {
		snprintf(reg_addrs[0].dev_name, 128, "%s", dev_name);
		return 1;
	} else if (strcmp(reg_addrs[1].dev_name, "") == 0) {
		snprintf(reg_addrs[1].dev_name, 128, "%s", dev_name);
		return 1;
	}

	return 0;
}

void set_addr(char *dev_name, s16 value)
{
	if (strcmp(reg_addrs[0].dev_name, dev_name) == 0)
		reg_addrs[0].addr = value;
	else if (strcmp(reg_addrs[1].dev_name, dev_name) == 0)
		reg_addrs[1].addr = value;
	else
		pr_warn("Invalid deviceName %s\n", dev_name);
}

struct smpro_misc {
	struct regmap *regmap;
};

static ssize_t boot_progress_show(struct device *dev, struct device_attribute *da, char *buf)
{
	struct smpro_misc *misc = dev_get_drvdata(dev);
	u32 boot_stage_high_reg;
	u32 boot_stage_low_reg;
	u32 current_boot_stage;
	u32 boot_stage_reg;
	u32 boot_progress;
	u8 boot_status;
	u8 boot_stage;
	int ret;

	/* Read current boot stage */
	ret = regmap_read(misc->regmap, BOOT_STAGE_CUR_STAGE, &current_boot_stage);
	if (ret)
		return ret;

	current_boot_stage &= 0xff;
	/* Read the boot progress */
	ret = regmap_read(misc->regmap, BOOT_STAGE_SELECT, &boot_stage_reg);
	if (ret)
		return ret;

	boot_stage = (boot_stage_reg & 0xff00) >> 8;
	boot_status = boot_stage_reg & 0xff;
	if (boot_stage < current_boot_stage ||
	    (boot_status == 3 && boot_stage == 3)) {
		ret = regmap_write(misc->regmap, BOOT_STAGE_SELECT,
				   ((boot_stage_reg & 0xff00) | 0x1));
		if (ret)
			return ret;
	}

	switch (boot_stage) {
	case BOOT_STAGE_UEFI:
	case BOOT_STAGE_OS:
		/*
		 * The progress is 32 bits:
		 * B3.byte[0] B3.byte[1] B1.byte[0] B1.byte[1]
		 */
		ret = regmap_read(misc->regmap,	BOOT_STAGE_STATUS_LO, &boot_stage_low_reg);
		if (!ret)
			ret = regmap_read(misc->regmap, BOOT_STAGE_STATUS_HI, &boot_stage_high_reg);
		if (ret)
			return ret;

		boot_progress = swab16(boot_stage_low_reg) | swab16(boot_stage_high_reg) << 16;
		goto done;
	default:
		boot_progress = 0x0;
		goto done;
	}

done:
	return snprintf(buf, PAGE_SIZE, "0x%02x 0x%02x 0x%08X\n",
			boot_stage, boot_status, boot_progress);
}

static DEVICE_ATTR_RO(boot_progress);

static ssize_t soc_power_limit_show(struct device *dev, struct device_attribute *da, char *buf)
{
	struct smpro_misc *misc = dev_get_drvdata(dev);
	int ret;
	unsigned int value;

	ret = regmap_read(misc->regmap, SOC_POWER_LIMIT, &value);
	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t soc_power_limit_store(struct device *dev, struct device_attribute *da,
				     const char *buf, size_t count)
{
	struct smpro_misc *misc = dev_get_drvdata(dev);
	unsigned long val;
	s32 ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;

	ret = regmap_write(misc->regmap, SOC_POWER_LIMIT, (unsigned int)val);
	if (ret)
		return -EPROTO;

	return count;
}

static ssize_t reg_rw_show(struct device *dev, struct device_attribute *da, char *buf)
{
	struct smpro_misc *misc = dev_get_drvdata(dev);
	int ret;
	unsigned int value;

	s16 addr = get_addr(dev->kobj.name);

	if (addr == -1) {
		pr_err("Register address is not set!\n");
		return -ENODATA;
	}

	ret = regmap_read(misc->regmap, (unsigned int)addr, &value);

	set_addr(dev->kobj.name, -1);
	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%x\n",
		((value & 0xff00) >> 8) | ((value & 0xff) << 8));
}

static ssize_t reg_rw_store(struct device *dev, struct device_attribute *da,
			    const char *buf, size_t count)
{
	struct smpro_misc *misc = dev_get_drvdata(dev);
	unsigned long val;
	unsigned long swapval;
	s32 ret;
	s16 addr = get_addr(dev->kobj.name);

	if (addr == -1) {
		pr_err("Register address is not set!\n");
		return -ENODATA;
	}

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;
	swapval = ((val & 0xff00) >> 8) | ((val & 0xff) << 8);

	ret = regmap_write(misc->regmap, (unsigned int)addr, (unsigned int)swapval);

	if (ret)
		return -EPROTO;

	set_addr(dev->kobj.name, -1);

	return count;
}
static DEVICE_ATTR_RW(reg_rw);

static ssize_t reg_addr_show(struct device *dev, struct device_attribute *da, char *buf)
{
	struct smpro_misc *misc = dev_get_drvdata(dev);
	s16 addr = get_addr(dev->kobj.name);

	if (addr == -1) {
		pr_err("Register address is not set!\n");
		return -ENODATA;
	}

	return snprintf(buf, PAGE_SIZE, "%x\n", addr);
}

static ssize_t reg_addr_store(struct device *dev, struct device_attribute *da,
			      const char *buf, size_t count)
{
	struct smpro_misc *misc = dev_get_drvdata(dev);
	s32 ret;
	s16 addr;

	ret = kstrtos16(buf, 0, &addr);
	if (ret)
		return ret;

	if (addr < 0x00 || addr > 0xff) {
		pr_err("Invalid address. Range is 0 to 0xff!\n");
		return -EINVAL;
	}

	set_addr(dev->kobj.name, addr);

	return count;
}
static DEVICE_ATTR_RW(reg_addr);

static DEVICE_ATTR_RW(soc_power_limit);

static struct attribute *smpro_misc_attrs[] = {
	&dev_attr_boot_progress.attr,
	&dev_attr_soc_power_limit.attr,
	&dev_attr_reg_addr.attr,
	&dev_attr_reg_rw.attr,
	NULL
};

static const struct attribute_group smpro_misc_attr_group = {
	.attrs = smpro_misc_attrs
};

static int smpro_misc_probe(struct platform_device *pdev)
{
	struct smpro_misc *misc;
	int ret;

	misc = devm_kzalloc(&pdev->dev, sizeof(struct smpro_misc), GFP_KERNEL);
	if (!misc)
		return -ENOMEM;

	platform_set_drvdata(pdev, misc);

	misc->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!misc->regmap)
		return -ENODEV;

	ret = sysfs_create_group(&pdev->dev.kobj, &smpro_misc_attr_group);
	if (ret)
		dev_err(&pdev->dev, "SMPro misc sysfs registration failed\n");

	if (!find_dev(pdev->dev.kobj.name))
		init_addr(pdev->dev.kobj.name);

	return 0;
}

static int smpro_misc_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &smpro_misc_attr_group);
	pr_info("SMPro misc sysfs entries removed");

	return 0;
}

static struct platform_driver smpro_misc_driver = {
	.probe		= smpro_misc_probe,
	.remove		= smpro_misc_remove,
	.driver = {
		.name	= "smpro-misc",
	},
};

module_platform_driver(smpro_misc_driver);

MODULE_AUTHOR("Thu Ba Nguyen <thu@os.amperecomputing.com>");
MODULE_AUTHOR("Tung Nguyen <tung.nguyen@amperecomputing.com>");
MODULE_AUTHOR("Quan Nguyen <quan@os.amperecomputing.com>");
MODULE_DESCRIPTION("Ampere Altra SMpro Misc driver");
MODULE_LICENSE("GPL");
