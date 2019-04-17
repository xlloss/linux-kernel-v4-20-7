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
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/device.h>

#define DRIVER_NAME "eeprom_chrdev"
#define DEVICE_NUM 1
#define DEVICE_NAME "eep_cdev"

struct eep_cdev_data {
    int id;
    struct i2c_client *eep_i2c_client;
    struct class *eepcdev_class;
    struct device *eepcdev_device;
	unsigned int eep_cdev_maj;
	dev_t eep_devt;
};

struct eep_cdev_data *eep_data;

static struct cdev eep_cdev;


int write_eep(struct i2c_client *client, unsigned char *val, loff_t count)
{
	int ret, i;
	unsigned char reg_addr = 0;

	for (i = 0; i < count; i++) {
		msleep(50);
		ret = i2c_smbus_write_byte_data(client, reg_addr, val[i]);
		if (ret) {
			dev_info(&client->dev, "%s fail\r\n)", __func__);
			return -1;
		}
		reg_addr++;
	}

	return 0;
}

int read_eep(struct i2c_client *client, unsigned char *val, size_t count)
{
	int i;
	unsigned char reg_addr = 0;

	for (i = 0; i < count; i++) {
		msleep(50);
		val[i] = i2c_smbus_read_byte_data(client, reg_addr);
		if (val[i] < 0) {
			dev_info(&client->dev, "%s fail\r\n)", __func__);
			return -1;
		}
		reg_addr++;
	}

	return 0;
}

static loff_t eep_cdev_llseek(struct file *file, loff_t offset, int whence)
{
	loff_t ret = -EINVAL;

	switch (whence) {
	case SEEK_END:
	case SEEK_DATA:
	case SEEK_HOLE:
		/* unsupported */
		return -EINVAL;
	case SEEK_CUR:
		if (offset == 0)
			return file->f_pos;

		offset += file->f_pos;
		break;
	case SEEK_SET:
		break;
	}

	if (offset != file->f_pos) {
		file->f_pos = offset;
		file->f_version = 0;
		ret = offset;
	}

	return ret;
}

#define MAX_BUF_SZ 256

ssize_t eep_cdev_read(struct file *filp, char __user *buf,
	size_t count, loff_t *f_pos)
{
	size_t ver_buf_sz, bytes_read = 0;
	unsigned char *data;
	ssize_t ret;

	ver_buf_sz = min_t(size_t, count, MAX_BUF_SZ);
	data = kmalloc(ver_buf_sz, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	ret = read_eep(eep_data->eep_i2c_client, data, count);
	if (ret) {
		pr_err("read_eep fail\r\n");
		return -1;
	}

	while (bytes_read < count) {
		if (copy_to_user(buf + bytes_read, data, ver_buf_sz)) {
			ret = -EFAULT;
			goto out;
		}
		bytes_read += ver_buf_sz;
		ver_buf_sz = min(count - bytes_read, (size_t)MAX_BUF_SZ);
	}

	(*f_pos) += bytes_read;
	ret = bytes_read;
out:
	kfree(data);
	return ret;
}


ssize_t eep_cdev_write(struct file *filp, const char __user *buf,
	size_t count, loff_t *f_pos)
{
	unsigned char *data;
	size_t buf_sz = 0, bytes_written = 0;
	int ret;

	buf_sz = min_t(size_t, count, MAX_BUF_SZ);
	data = kmalloc(buf_sz, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	do {
		if (copy_from_user(data, &buf[bytes_written], buf_sz)) {
			ret = -EFAULT;
			goto out;
		}

		bytes_written += buf_sz;
		buf_sz = min(count - bytes_written, (size_t)MAX_BUF_SZ);
	} while (bytes_written < count);

	(*f_pos) += bytes_written;
	ret = bytes_written;

	write_eep(eep_data->eep_i2c_client, data, count);

out:
	kfree(data);
	return ret;
}

static int eep_cdev_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int eep_cdev_close(struct inode *inode, struct file *file)
{
    printk("%s\r\n", __func__);
	return 0;
}


struct file_operations fops = {
    .owner	 = THIS_MODULE,
    .open	 = eep_cdev_open,
    .release = eep_cdev_close,
    .llseek  = eep_cdev_llseek,
    .read 	 = eep_cdev_read,
    .write 	 = eep_cdev_write,
};

static int tiny4412_at24_probe(struct i2c_client *client)
{
	int ret;
	dev_t devt;

	dev_info(&client->dev, "%s\r\n", __func__);

    ret = alloc_chrdev_region(&devt, 0, DEVICE_NUM, DRIVER_NAME);
    if(ret) {
		dev_err(&client->dev, "alloc_chrdev_region fail\r\n");
        goto error_unregister_chrdev;
	}

	cdev_init(&eep_cdev, &fops);
    ret = cdev_add(&eep_cdev, devt, DEVICE_NUM);
    if (ret) {
		dev_err(&client->dev, "cdev_add fail\r\n");
        goto error_cdev_del;
	}

	eep_data = kmalloc(sizeof(struct eep_cdev_data), GFP_KERNEL);
	if (!eep_data) {
		dev_err(&client->dev, "alloc eep_data fail\r\n");
		goto error_cdev_del;
	}

	eep_data->eepcdev_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (!eep_data->eepcdev_class) {
		dev_err(&client->dev, "class_create fail\r\n");
		goto error_cdev_del;
	}

	eep_data->eepcdev_device = device_create(eep_data->eepcdev_class,
		NULL, devt, NULL, "%s-%d", DEVICE_NAME, 0);

	if (!eep_data->eepcdev_device) {
		dev_err(&client->dev, "device_create fail\r\n");
		goto error_class_create;
	}

	eep_data->eep_i2c_client = client;
	eep_data->eep_cdev_maj = MAJOR(devt);
	eep_data->eep_devt = devt;

	return 0;

error_class_create:
	class_destroy(eep_data->eepcdev_class);

error_cdev_del:
	cdev_del(&eep_cdev);
	kfree(eep_data);

error_unregister_chrdev:
	unregister_chrdev_region(devt, DEVICE_NUM);

    return -1;

}

static int tiny4412_at24_remove(struct i2c_client *client)
{
	dev_t devt = MKDEV(eep_data->eep_cdev_maj, 0);

	dev_info(&client->dev, "%s\r\n", __func__);

    cdev_del(&eep_cdev);
    unregister_chrdev_region(devt, DEVICE_NUM);
    device_del(eep_data->eepcdev_device);
    class_destroy(eep_data->eepcdev_class);
    kfree(eep_data);
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
