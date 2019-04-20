#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/export.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

/*
    PWM 时钟频率 100M
    100M / 250 / 4 = 100000
    1/100000 = 10us
*/


static int major;
static struct cdev backlight_cdev;
static struct class *cls;
static struct pinctrl *pctrl;
static struct pinctrl_state *pstate_in;
static struct pinctrl_state *pstate_out;
static int one_write_pin;
static struct timer_list one_wire_timer;

#define TCFG0 0x0000
#define TCFG1 0x0004
#define TCON 0x0008 
#define TCNTB0 0x000C
#define TCMPB0 0x0010
#define TCNTO0 0x0014
#define TCNTB1 0x0018
#define TCMPB1 0x001C
#define TCNTO1 0x0020
#define TCNTB2 0x0024
#define TCMPB2 0x0028
#define TCNTO2 0x002C
#define TCNTB3 0x0030
#define TCMPB3 0x0034
#define TCNTO3 0x0038
#define TCNTB4 0x003C
#define TCNTO4 0x0040
#define TINT_CSTAT 0x0044

#define SLOW_LOOP_FEQ 25
#define FAST_LOOP_FEQ 60
#define REQ_TS   0x40U
#define REQ_INFO 0x60U

static int timer_interval = HZ / 50;
static unsigned lcd_type, firmware_ver;
static int has_ts_data = 1;
static int exitting;

void __iomem *timer_base;
static volatile unsigned int io_bit_count;
static volatile unsigned int io_data;
static volatile unsigned char one_wire_request;

// once a session complete
static unsigned total_received, total_error;
static unsigned last_req, last_res;

enum
{
	IDLE,
	START,
	REQUEST,
	WAITING,
	RESPONSE,
	STOPING,
} one_wire_status = IDLE;

/* CRC TABLE */
static const unsigned char crc8_tab[] = {
	0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
	0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
	0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
	0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
	0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
	0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
	0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
	0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
	0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
	0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
	0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
	0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
	0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
	0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
	0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
	0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
	0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
	0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
	0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
	0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
	0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
	0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
	0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
	0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
	0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
	0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
	0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
	0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
	0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
	0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
	0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
	0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3,
};

#define crc8_init(crc) ((crc) = 0xac)
#define crc8(crc, v) ( (crc) = crc8_tab[(crc) ^(v)])
#define SAMPLE_BPS 9600
#define TCON_T3START (1<<16)
#define TCON_T3MANUALUPD (1<<17)
#define TCON_T3START (1<<16)
#define TCON_T3RELOAD (1<<19)
#define TCFG1_MUX3_MASK (15<<12)
static int err_i = 0;

// one-wire protocol core
unsigned long TCNT_FOR_SAMPLE_BIT;
unsigned long TCNT_FOR_FAST_LOOP;
unsigned long TCNT_FOR_SLOW_LOOP;
static DECLARE_WAIT_QUEUE_HEAD(bl_waitq);
static int bl_ready;
static unsigned char backlight_req = 0;
static unsigned char backlight_init_success;
static struct device *dev;
static struct clk *base_clk;
static struct resource *res = NULL, *irq = NULL;


static inline void set_pin_as_input(void)
{
	gpio_direction_input(one_write_pin);
}

static inline void set_pin_as_output(void)
{
	gpio_direction_output(one_write_pin, 1);
}

static inline void set_pin_value(int v)
{
	if (v)
		gpio_set_value(one_write_pin, 1);
	else
		gpio_set_value(one_write_pin, 0);
}

static inline int get_pin_value(void)
{
	return gpio_get_value(one_write_pin);
}


static inline void stop_timer_for_1wire(void)
{
	unsigned long tcon;

	tcon = readl(timer_base + TCON);
	tcon &= ~TCON_T3START;
	writel(tcon, timer_base + TCON);
}

static inline void start_timer_for_1wire(void)
{
	unsigned int tint;

	tint = readl(timer_base + TINT_CSTAT);
	tint |= 0x108;
	writel(tint, timer_base + TINT_CSTAT);

}

static inline void notify_bl_data(unsigned char a, unsigned char b, unsigned char c)
{
	bl_ready = 1;
	backlight_init_success = 1;
	wake_up_interruptible(&bl_waitq);
}

static inline void notify_info_data(unsigned char _lcd_type,
		unsigned char ver_year, unsigned char week)
{
}

static void one_wire_session_complete(unsigned char req, unsigned int res)
{
	unsigned char crc;
	const unsigned char *p = (const unsigned char*)&res;
	total_received ++;

	last_res = res;

	crc8_init(crc);
	crc8(crc, p[3]);
	crc8(crc, p[2]);
	crc8(crc, p[1]);

	if (crc != p[0]) {
		/* CRC dismatch */
		if (total_received > 100)
			total_error++;
		return;
	}

	switch(req) {
		case REQ_TS:
			break;

		case REQ_INFO:
			notify_info_data(p[3], p[2], p[1]);
			break;
		default:
			notify_bl_data(p[3], p[2], p[1]);
			break;
	}
}

static irqreturn_t timer_for_1wire_interrupt(int irq, void *dev_id)
{
	unsigned int tint;

	/* clear int status */
	tint = readl(timer_base + TINT_CSTAT);
	tint |= 0x100;
	writel(tint, timer_base + TINT_CSTAT);

	io_bit_count--;

	switch(one_wire_status) {
	case START:
		if (io_bit_count == 0) {
			io_bit_count = 16;
			one_wire_status = REQUEST;
		}
		break;

	case REQUEST:
		/* Send a bit */
		set_pin_value(io_data & (1 << 31));
		io_data <<= 1;
		if (io_bit_count == 0) {
			io_bit_count = 2;
			one_wire_status = WAITING;
		}
		break;
		
	case WAITING:
		if (io_bit_count == 0) {
			io_bit_count = 32;
			one_wire_status = RESPONSE;
		}
		if (io_bit_count == 1) {
			set_pin_as_input();
			set_pin_value(1);
		}
		break;
		
	case RESPONSE:
		/* Get a bit */
		io_data = (io_data << 1) | get_pin_value();
		if (io_bit_count == 0) {
			io_bit_count = 2;
			one_wire_status = STOPING;
			set_pin_value(1);
			set_pin_as_output();
			one_wire_session_complete(one_wire_request, io_data);
		}
		break;

	case STOPING:
		if (io_bit_count == 0) {
			one_wire_status = IDLE;
			stop_timer_for_1wire();
		}
		break;
		
	default:
		stop_timer_for_1wire();

	}

	return IRQ_HANDLED;
}

static void start_one_wire_session(unsigned char req)
{
	unsigned long tcon;
	unsigned char crc;

	if (one_wire_status != IDLE) {
		if (++err_i < 3)
			return;
	}

	one_wire_status = START;

	set_pin_value(1);
	set_pin_as_output();

	/* IDLE to START */
	crc8_init(crc);
	crc8(crc, req);
	io_data = (req << 8) + crc;
	io_data <<= 16;

	last_req = (io_data >> 16);
	one_wire_request = req;
	io_bit_count = 1;
	set_pin_as_output();

	writel(TCNT_FOR_SAMPLE_BIT, timer_base + TCNTB3);
	/* init tranfer and start timer */
	tcon = readl(timer_base + TCON);
	tcon &= ~(0xF << 16);
	tcon |= TCON_T3MANUALUPD;
	writel(tcon, timer_base + TCON);

	tcon |= TCON_T3START;
	tcon |= TCON_T3RELOAD;
	tcon &= ~TCON_T3MANUALUPD;

	writel(tcon, timer_base + TCON);
	set_pin_value(0);
}

static unsigned char write_cnt = 0;

static ssize_t backlight_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
	unsigned char reg, ret;
	write_cnt++;

	pr_debug("%s\n", __func__);

	ret = copy_from_user(&reg, buf, 1);
	if (ret < 0)
		pr_err("%s copy_from_user error\n", __func__);


	if (reg > 127)
		reg = 127;

	backlight_req = reg;

	start_one_wire_session(backlight_req);

	return 1;
}


static int backlight_open(struct inode *inode, struct file *file)
{
    pr_debug("backlight_open\n");
    return 0;
}

static int backlight_release(struct inode *inode, struct file *file)
{
    pr_debug("backlight_exit\n");
    return 0;
}

static struct file_operations backlight_fops = {
	.owner = THIS_MODULE,
	.open = backlight_open,
	.release = backlight_release,
	.write = backlight_write,
};


static int init_timer_for_1wire(unsigned long pclk)
{
	unsigned long tcfg1;
	unsigned long tcfg0;
	unsigned prescale1_value;

	pr_info("PWM clock = %ld\n", pclk);

	/* get prescaler */
	tcfg0 = readl(timer_base + TCFG0);


	/* we use system prescaler value because timer 4 uses same one */
	prescale1_value = (tcfg0 >> 8) & 0xFF;

	/* calc the TCNT_FOR_SAMPLE_BIT, that is one of the goal */
	TCNT_FOR_SAMPLE_BIT = pclk / (prescale1_value + 1) / SAMPLE_BPS - 1;
	TCNT_FOR_FAST_LOOP  = pclk / (prescale1_value + 1) / FAST_LOOP_FEQ - 1;
	TCNT_FOR_SLOW_LOOP  = pclk / (prescale1_value + 1) / SLOW_LOOP_FEQ - 1;

	/* select timer 3, the 2rd goal */
	tcfg1 = readl(timer_base + TCFG1);
	tcfg1 &= ~TCFG1_MUX3_MASK;
	writel(tcfg1, timer_base + TCFG1);

	pr_info("TCNT_FOR_SAMPLE_BIT = %ld, TCFG1 = %08x\n",
			TCNT_FOR_SAMPLE_BIT, readl(timer_base + TCFG1));

	return 0;
}

void one_wire_timer_proc(struct timer_list *unused)
{
	unsigned char req;

	if (exitting)
		return;

	add_timer(&one_wire_timer);

	if (lcd_type == 0) {
		req = REQ_INFO;
	} else if (!backlight_init_success) {
		req = 127;
	} else if (backlight_req) {
		req = backlight_req;
		backlight_req = 0;
	} else {
		return;
	}

	start_one_wire_session(req);
}

static int backlight_probe(struct platform_device *pdev)
{
	int ret;
	dev_t devid;
	unsigned long pclk;
	unsigned int tint;

	dev = &pdev->dev;
	printk("enter %s v2.0\n", __func__);

	pctrl = devm_pinctrl_get(dev);
	if (pctrl == NULL) {
		dev_err(&pdev->dev, "devm_pinctrl_get error\n");
		return -EINVAL;
	}
	pstate_in = pinctrl_lookup_state(pctrl, "backlight_in");
	pstate_out = pinctrl_lookup_state(pctrl, "backlight_out");
	if (pstate_in == NULL || pstate_out == NULL) {
		dev_err(&pdev->dev, "pinctrl_lookup_state error\n");
		return -EINVAL;
	}

	one_write_pin = of_get_named_gpio(dev->of_node, "tiny4412,backlight", 0);
	if (!one_write_pin) {
		dev_err(dev, "of_get_named_gpio error\n");
		return -EINVAL;
	}

	devm_gpio_request_one(dev, one_write_pin, GPIOF_OUT_INIT_HIGH, "one_write");

	gpio_set_value(one_write_pin, 1);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
        dev_err(dev, "platform_get_resource error\n");
        return -EINVAL;
    }

	base_clk = devm_clk_get(&pdev->dev, "timers");
	if (IS_ERR(base_clk)) {
        dev_err(dev, "failed to get timer base clk\n");
        return PTR_ERR(base_clk);
    }

	ret = clk_prepare_enable(base_clk);
	if (ret < 0) {
        dev_err(dev, "failed to enable base clock\n");
        return ret;
    }

	pclk = clk_get_rate(base_clk);
	dev_info(dev, "PWM clock = %ld\n", pclk);

	timer_base = devm_ioremap_resource(&pdev->dev, res);
    if (timer_base == NULL) {
        dev_err(dev, "devm_ioremap_resource error\n");
        return -EINVAL;
    }

    irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if (irq == NULL) {
        dev_err(dev, "platform_get_resource irq error\n");
        return -EINVAL;
    }

	ret = devm_request_irq(dev, irq->start, timer_for_1wire_interrupt,
		IRQF_TIMER, "backlight", NULL);
    if (ret) {
        dev_err(dev, "unable to request irq\n");
        return -EINVAL;
    }

	init_timer_for_1wire(pclk);

	one_wire_timer.expires = jiffies + timer_interval;
	timer_setup(&one_wire_timer, one_wire_timer_proc, 0); 
	one_wire_timer_proc(0);


	/* enable TINT */
	tint = readl(timer_base + TINT_CSTAT);
	tint |= 0x108;
	writel(tint, timer_base + TINT_CSTAT);


    if (alloc_chrdev_region(&devid, 0, 1, "backlight") < 0) {
        dev_err(dev, "%s ERROR\n", __func__);
        return -EINVAL;
    }

    major = MAJOR(devid);
    cdev_init(&backlight_cdev, &backlight_fops);
    cdev_add(&backlight_cdev, devid, 1);
    cls = class_create(THIS_MODULE, "backlight_demo");
    device_create(cls, NULL, MKDEV(major, 0), NULL, "backlight");

    return 0;
}


static int backlight_remove(struct platform_device *pdev)
{
    device_destroy(cls, MKDEV(major, 0));
    class_destroy(cls);
    cdev_del(&backlight_cdev);
    unregister_chrdev_region(MKDEV(major, 0), 1);
    devm_pinctrl_put(pctrl);
    devm_free_irq(dev, irq->start, NULL);
    clk_disable_unprepare(base_clk);
    devm_gpio_free(dev, one_write_pin);
    return 0;
}

static const struct of_device_id backlight_dt_ids[] =
{
    { .compatible = "tiny4412,backlight_demo", },
    {},
};

MODULE_DEVICE_TABLE(of, backlight_dt_ids);

static struct platform_driver backlight_driver =
{
    .driver = {
        .name = "backlight_demo",
        .of_match_table = of_match_ptr(backlight_dt_ids),
    },
	.probe = backlight_probe,
    .remove = backlight_remove,
};


static int backlight_init(void)
{
    int ret;

	ret = platform_driver_register(&backlight_driver);
	if (ret)
        pr_err("backlight_init faid\n");

    return ret;
}


static void backlight_exit(void)
{
    platform_driver_unregister(&backlight_driver);
}
module_init(backlight_init);
module_exit(backlight_exit);
MODULE_LICENSE("GPL");
