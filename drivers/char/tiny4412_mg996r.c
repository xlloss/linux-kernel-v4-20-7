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
#include <linux/pwm.h>


#define DEVICE_NAME "dev-mg996rs"
#define DRIVER_NAME "tiny4412-mg996rs"

#define IOCTL_TURN_ON 0xFF000000
#define IOCTL_TURN_OFF 0xFF000001
#define IOCTL_TURN_RIGHT 0xFF000002
#define IOCTL_TURN_LEFT 0xFF000003
#define MG996R_TURN_RIGHT 2000000
#define MG996R_TURN_LEFT 1000000
#define MG996R_PERIOD 20000000 /* 20ms */
struct mg996r {
	struct pwm_device *pwm_device;
};

struct mg996r mg996r;

static long tiny4412_mg996rs_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	pr_err("%s: %lu 0x%x\n", __func__, arg, cmd);

	switch (cmd) {
		case IOCTL_TURN_ON:
			pwm_enable(mg996r.pwm_device);
			break;
		case IOCTL_TURN_OFF:
			pwm_disable(mg996r.pwm_device);
			break;
		case IOCTL_TURN_RIGHT:
			pr_err("%s: IOCTL_TURN_RIGHT\n", __func__);
			pwm_config(mg996r.pwm_device, MG996R_TURN_RIGHT, MG996R_PERIOD);
			break;

		case IOCTL_TURN_LEFT:
			pr_err("%s: IOCTL_TURN_LEFT\n", __func__);
			pwm_config(mg996r.pwm_device, MG996R_TURN_LEFT, MG996R_PERIOD);
			break;

		default:
			return -EINVAL;
	}

	pr_err("period:%d, duty:%d\r\n",
		pwm_get_period(mg996r.pwm_device),
		pwm_get_duty_cycle(mg996r.pwm_device));

	return 0;
}

static struct file_operations tiny4412_mg996r_dev_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl	= tiny4412_mg996rs_ioctl,
};

static struct miscdevice tiny4412_mg996r_dev = {
	.minor			= MISC_DYNAMIC_MINOR,
	.name			= DEVICE_NAME,
	.fops			= &tiny4412_mg996r_dev_fops,
};

static int tiny4412_mg996r_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node	*node = pdev->dev.of_node;

	mg996r.pwm_device = devm_of_pwm_get(&pdev->dev, node, NULL);
	pwm_config(mg996r.pwm_device, MG996R_TURN_RIGHT, MG996R_PERIOD);

	pr_err("period:%d, duty:%d\r\n",
		pwm_get_period(mg996r.pwm_device),
		pwm_get_duty_cycle(mg996r.pwm_device));

	ret = misc_register(&tiny4412_mg996r_dev);
	if (ret) {
		dev_err(&pdev->dev, "misc_register fail\r\n");
		return ret;
	}

	return 0;
}

static int tiny4412_mg996r_remove(struct platform_device *op)
{
	pwm_disable(mg996r.pwm_device);
	misc_deregister(&tiny4412_mg996r_dev);

	return 0;
}

#ifdef CONFIG_OF
/* Match table for device tree binding */
static const struct of_device_id tiny4412_mg996r_of_match[] = {
	{ .compatible = "tiny4412,mg996r-sample", .data = NULL},
	{},
};
MODULE_DEVICE_TABLE(of, tiny4412_mg996r_of_match);
#else
#define tiny4412_mg996r_of_match NULL
#endif

static struct platform_driver tiny4412_mg996r_platform_driver = {
	.probe = tiny4412_mg996r_probe,
	.remove = tiny4412_mg996r_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = tiny4412_mg996r_of_match,
	},
};

static int __init tiny4412_mg996r_dev_init(void)
{
	int ret;

	ret = platform_driver_register(&tiny4412_mg996r_platform_driver);
	if (ret) {
		pr_err("%s platform_driver_register fail\r\n", __func__);
		return ret;
	}

	pr_info("%s initialized\n", __func__);

	return ret;
}

static void __exit tiny4412_mg996r_dev_exit(void)
{
	platform_driver_unregister(&tiny4412_mg996r_platform_driver);
}

module_init(tiny4412_mg996r_dev_init);
module_exit(tiny4412_mg996r_dev_exit);

MODULE_AUTHOR("Slash <slash.linux.c@gmail.com>");
MODULE_DESCRIPTION("Tiny4412 MG996r Driver");
MODULE_LICENSE("GPL");
