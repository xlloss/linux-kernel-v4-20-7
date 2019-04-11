// SPDX-License-Identifier: GPL-2.0+
/* 
 * tiny4412-at24c08 test driver
 * Copy from at24.c - handle most I2C EEPROMs
 *
 * Copyright (C) 2005-2007 David Brownell
 * Copyright (C) 2008 Wolfram Sang, Pengutronix
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/mod_devicetable.h>
#include <linux/log2.h>
#include <linux/bitops.h>
#include <linux/jiffies.h>
#include <linux/property.h>
#include <linux/acpi.h>
#include <linux/i2c.h>
#include <linux/nvmem-provider.h>
#include <linux/regmap.h>
#include <linux/pm_runtime.h>
#include <linux/gpio/consumer.h>

static int tiny4412_at24_probe(struct i2c_client *client)
{
	dev_info(&client->dev, "%s\r\n", __func__);

	return 0;
}

static int tiny4412_at24_remove(struct i2c_client *client)
{
	dev_info(&client->dev, "%s\r\n", __func__);

	return 0;
}

static const struct of_device_id tiny4412_at24_of_match[] = {
	{ .compatible = "atmel,24c08",		.data = NULL },
	{ /* END OF LIST */ },
};

static struct i2c_driver tiny4412_at24_driver = {
	.driver = {
		.name = "tiny4412_at24",
		.of_match_table = tiny4412_at24_of_match,
	},
	.probe_new = tiny4412_at24_probe,
	.remove = tiny4412_at24_remove,
};

static int __init tiny4412_at24_init(void)
{
	return i2c_add_driver(&tiny4412_at24_driver);
}
module_init(tiny4412_at24_init);

static void __exit tiny4412_at24_exit(void)
{
	i2c_del_driver(&tiny4412_at24_driver);
}
module_exit(tiny4412_at24_exit);

MODULE_DESCRIPTION("Driver for most I2C EEPROMs");
MODULE_AUTHOR("Redbean");
MODULE_LICENSE("GPL");
