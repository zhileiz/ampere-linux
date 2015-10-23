/*
 * Copyright 2015 IBM Corp.
 *
 * Joel Stanley <joel@jms.id.au>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/gpio/driver.h>

struct aspeed_gpio {
	struct gpio_chip chip;
	spinlock_t lock;
	void __iomem *base;
};

struct aspeed_gpio_bank {
	uint16_t	val_regs;
	char		names[4];
};

static struct aspeed_gpio_bank aspeed_gpio_banks[] = {
	{
		.val_regs = 0x0000,
		.names = { 'A', 'B', 'C', 'D' },
	},
	{
		.val_regs = 0x0020,
		.names = { 'E', 'F', 'G', 'H' },
	},
	{
		.val_regs = 0x0070,
		.names = { 'I', 'J', 'K', 'L' },
	},
	{
		.val_regs = 0x0078,
		.names = { 'M', 'N', 'O', 'P' },
	},
	{
		.val_regs = 0x0080,
		.names = { 'Q', 'R', 'S', 'T' },
	},
	{
		.val_regs = 0x0088,
		.names = { 'U', 'V', 'W', 'X' },
	},
};

#define GPIO_BANK(x)	((x) >> 5)
#define GPIO_OFFSET(x)	((x) & 0x1f)
#define GPIO_BIT(x)	BIT(GPIO_OFFSET(x))

#define GPIO_DATA	0x00
#define GPIO_DIR	0x04

static inline struct aspeed_gpio *to_aspeed_gpio(struct gpio_chip *chip)
{
	return container_of(chip, struct aspeed_gpio, chip);
}

static struct aspeed_gpio_bank *to_bank(unsigned int offset)
{
	unsigned int bank = GPIO_BANK(offset);
	WARN_ON(bank > ARRAY_SIZE(aspeed_gpio_banks));
	return &aspeed_gpio_banks[bank];
}

static void *bank_val_reg(struct aspeed_gpio *gpio,
		struct aspeed_gpio_bank *bank,
		unsigned int reg)
{
	return gpio->base + bank->val_regs + reg;
}

static int aspeed_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct aspeed_gpio *gpio = to_aspeed_gpio(gc);
	struct aspeed_gpio_bank *bank = to_bank(offset);

	return !!(ioread32(bank_val_reg(gpio, bank, GPIO_DATA))
			& GPIO_BIT(offset));
}

static void aspeed_gpio_set(struct gpio_chip *gc, unsigned int offset,
			    int val)
{
	struct aspeed_gpio *gpio = to_aspeed_gpio(gc);
	struct aspeed_gpio_bank *bank = to_bank(offset);
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&gpio->lock, flags);

	reg = ioread32(bank_val_reg(gpio, bank, GPIO_DATA));
	if (val)
		reg |= GPIO_BIT(offset);
	else
		reg &= ~GPIO_BIT(offset);

	iowrite32(reg, bank_val_reg(gpio, bank, GPIO_DATA));

	spin_unlock_irqrestore(&gpio->lock, flags);
}

static int aspeed_gpio_dir_in(struct gpio_chip *gc, unsigned int offset)
{
	struct aspeed_gpio *gpio = to_aspeed_gpio(gc);
	struct aspeed_gpio_bank *bank = to_bank(offset);
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&gpio->lock, flags);

	reg = ioread32(bank_val_reg(gpio, bank, GPIO_DIR));
	iowrite32(reg & ~GPIO_BIT(offset), bank_val_reg(gpio, bank, GPIO_DIR));

	spin_unlock_irqrestore(&gpio->lock, flags);

	return 0;
}

static int aspeed_gpio_dir_out(struct gpio_chip *gc,
			       unsigned int offset, int val)
{
	struct aspeed_gpio *gpio = to_aspeed_gpio(gc);
	struct aspeed_gpio_bank *bank = to_bank(offset);
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&gpio->lock, flags);

	reg = ioread32(bank_val_reg(gpio, bank, GPIO_DIR));
	iowrite32(reg | GPIO_BIT(offset), bank_val_reg(gpio, bank, GPIO_DIR));

	spin_unlock_irqrestore(&gpio->lock, flags);

	return 0;
}

static void aspeed_gpio_set_names(struct aspeed_gpio *gpio)
{
	const char format[] = "GPIOXn";
	char *namebuf, **names;
	unsigned int i;

	/* our buffer of name pointers */
	names = devm_kmalloc_array(gpio->chip.dev, gpio->chip.ngpio,
			sizeof(char *), GFP_KERNEL);

	/* and one contiguous buffer for the names themselves */
	namebuf = devm_kmalloc_array(gpio->chip.dev, gpio->chip.ngpio,
			sizeof(format), GFP_KERNEL);

	for (i = 0; i < gpio->chip.ngpio; i++) {
		struct aspeed_gpio_bank *bank = to_bank(i);
		char *name = namebuf + (i * sizeof(format));
		int bit = GPIO_OFFSET(i);

		memcpy(name, format, 4);
		name[4] = bank->names[bit >> 3];
		name[5] = '0' + (bit % 8);
		name[6] = '\0';
		names[i] = name;
	}

	gpio->chip.names = (const char * const *)names;
}

static int __init aspeed_gpio_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct aspeed_gpio *gpio;

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	gpio->base = devm_ioremap_resource(&pdev->dev, res);
	if (!gpio->base)
		return -ENOMEM;

	spin_lock_init(&gpio->lock);

	gpio->chip.ngpio = ARRAY_SIZE(aspeed_gpio_banks) * 32;

	gpio->chip.dev = &pdev->dev;
	gpio->chip.direction_input = aspeed_gpio_dir_in;
	gpio->chip.direction_output = aspeed_gpio_dir_out;
	gpio->chip.get = aspeed_gpio_get;
	gpio->chip.set = aspeed_gpio_set;
	gpio->chip.label = dev_name(&pdev->dev);
	gpio->chip.base = -1;

	aspeed_gpio_set_names(gpio);

	platform_set_drvdata(pdev, gpio);

	return gpiochip_add(&gpio->chip);
}

static int aspeed_gpio_remove(struct platform_device *pdev)
{
	struct aspeed_gpio *gpio = platform_get_drvdata(pdev);

	gpiochip_remove(&gpio->chip);
	return 0;
}

static const struct of_device_id aspeed_gpio_of_table[] = {
	{ .compatible = "aspeed,ast2400-gpio" },
	{}
};
MODULE_DEVICE_TABLE(of, aspeed_gpio_of_table);

static struct platform_driver aspeed_gpio_driver = {
	.remove = aspeed_gpio_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = aspeed_gpio_of_table,
	},
};

module_platform_driver_probe(aspeed_gpio_driver, aspeed_gpio_probe);

MODULE_DESCRIPTION("Aspeed AST2400 GPIO Driver");
