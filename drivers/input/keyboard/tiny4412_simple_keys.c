/*********************************************************************
 * Author: Slash <slash.linux.c@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 ********************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/workqueue.h>

#define DEVICE_NAME "simple-keys"
#define DRIVER_NAME "tiny4412-simple-keys"

#define GPIO_NUM 1

struct simple_key {
	int irq[GPIO_NUM];
	struct gpio_desc *gpiod[GPIO_NUM];
	struct input_dev *input;
	unsigned int key_type;
	unsigned short key_code[GPIO_NUM];
};

struct simple_key simple_key;


static irqreturn_t gpio_irq_isr(int irq, void *devid)
{
	int *gpio_irq_num, i;
	unsigned char keyval;

	gpio_irq_num = (int *)devid;
	pr_err("IRQ %d, gpio_irq_num %d\r\n", irq, *gpio_irq_num);

	for (i = 0; i < GPIO_NUM; i++) {
		if (irq == simple_key.irq[i])
			break;
	}

	keyval = gpiod_get_value(simple_key.gpiod[i]);
	keyval = ~keyval & 0x01;
	pr_err("keyval %d\r\n", keyval);

	input_event(simple_key.input, simple_key.key_type,
		simple_key.key_code[i], keyval);

	input_sync(simple_key.input);

	return 0;
}

static int simple_keys_open(struct input_dev *input)
{
	pr_err("%s\r\n", __func__);
	return 0;
}

static void simple_keys_close(struct input_dev *input)
{
	pr_err("%s\r\n", __func__);
}

static int tiny4412_simple_key_probe(struct platform_device *pdev)
{
	int ret, i;
	char dt_gpio_name[15];
	struct input_dev *input;
	struct device *dev = &pdev->dev;

	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	simple_key.input = input;
	input_set_drvdata(input, &simple_key);

	input->name = pdev->name;
	input->phys = "tiny4412-keys/input0";
	input->dev.parent = dev;
	input->open = simple_keys_open;
	input->close = simple_keys_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;
	input_set_capability(input, EV_KEY, BTN_0);

	simple_key.key_type = EV_KEY;
	simple_key.key_code[0] = BTN_0;

	for (i = 0; i < GPIO_NUM; i++) {
		simple_key.irq[i] = 0;
		sprintf(dt_gpio_name, "key%d", i);
		simple_key.gpiod[i] = devm_gpiod_get_optional(&pdev->dev,
			dt_gpio_name, GPIOD_IN);
		if (IS_ERR(simple_key.gpiod[i])) {
			ret = PTR_ERR(simple_key.gpiod[i]);
			if (ret != -EPROBE_DEFER)
				dev_err(&pdev->dev, "Faiirq to get %s GPIO: %d\n",
					dt_gpio_name, ret);
			goto gpio_error;
		}

		simple_key.irq[i] = gpiod_to_irq(simple_key.gpiod[i]);
		ret = devm_request_irq(&pdev->dev, simple_key.irq[i],
			gpio_irq_isr, IRQ_TYPE_EDGE_BOTH, "key-irq", &simple_key.irq[i]);
		if (ret) {
			dev_err(&pdev->dev, "Cannot request irq %d (error %d)\n",
				simple_key.irq[i], ret);
			return ret;
		}
		disable_irq_nosync(simple_key.irq[i]);
	}

	ret = input_register_device(input);
	if (ret) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			ret);
		return ret;
	}

	for (i = 0; i < GPIO_NUM; i++) {
		if (simple_key.irq[i])
			enable_irq(simple_key.irq[i]);
	}

	return 0;

gpio_error:
	for (i = 0; i < GPIO_NUM; i++) {
		if (simple_key.gpiod[i])
			gpiod_put(simple_key.gpiod[i]);
	}

	return ret;
}

static int tiny4412_simple_key_remove(struct platform_device *pdev)
{
	int i;


	for (i = 0; i < GPIO_NUM; i++) {
		if (simple_key.irq[i])
			devm_free_irq(&pdev->dev, simple_key.irq[i], &simple_key.irq[i]);
	}

	return 0;
}

#ifdef CONFIG_OF
/* Match table for device tree binding */
static const struct of_device_id tiny4412_simple_key_of_match[] = {
	{ .compatible = "tiny4412,simple-key", .data = NULL},
	{},
};
MODULE_DEVICE_TABLE(of, tiny4412_simple_key_of_match);
#else
#define tiny4412_simple_key_of_match NULL
#endif

static struct platform_driver tiny4412_simple_key_platform_driver = {
	.probe = tiny4412_simple_key_probe,
	.remove = tiny4412_simple_key_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = tiny4412_simple_key_of_match,
	},
};

static int __init tiny4412_simple_key_dev_init(void)
{
	int ret;

	ret = platform_driver_register(&tiny4412_simple_key_platform_driver);
	if (ret) {
		pr_err("%s platform_driver_register fail\r\n", __func__);
		return ret;
	}

	pr_info("%s initialized\n", __func__);

	return ret;
}

static void __exit tiny4412_simple_key_dev_exit(void)
{
	platform_driver_unregister(&tiny4412_simple_key_platform_driver);
}

module_init(tiny4412_simple_key_dev_init);
module_exit(tiny4412_simple_key_dev_exit);

MODULE_AUTHOR("Slash <slash.linux.c@gmail.com>");
MODULE_DESCRIPTION("Tiny4412 SIMPLE KEY Driver");
MODULE_LICENSE("GPL");
