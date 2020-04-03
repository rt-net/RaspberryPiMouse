/*
 *
 * rtmouse.c
 * Raspberry Pi Mouse device driver
 *
 * Version: 1:1.5
 *
 * Copyright (C) 2015-2020 RT Corporation <shop@rt-net.jp>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kdev_t.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/stat.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>

// define the Raspberry Pi version here
// Raspberry Pi 1 B/A/B+/A+: 1
// Raspberry Pi 2 B        : 2
// Raspberry Pi 3 B/A+/B+  : 2
// Raspberry Pi 4 B        : 4
#define RASPBERRYPI 2

MODULE_AUTHOR("RT Corporation");
MODULE_LICENSE("GPL");
MODULE_VERSION("1:1.4");
MODULE_DESCRIPTION("Raspberry Pi Mouse device driver");

/* --- Device Numbers --- */
#define NUM_DEV_LED 4
#define NUM_DEV_SWITCH 3
#define NUM_DEV_SENSOR 1
#define NUM_DEV_BUZZER 1
#define NUM_DEV_MOTORRAWR 1
#define NUM_DEV_MOTORRAWL 1
#define NUM_DEV_MOTOREN 1
#define NUM_DEV_MOTOR 1
#define NUM_DEV_CNT 2

#define NUM_DEV_TOTAL                                                          \
	(NUM_DEV_LED + NUM_DEV_SWITCH + NUM_DEV_BUZZER + NUM_DEV_MOTORRAWR +   \
	 NUM_DEV_MOTORRAWL + NUM_DEV_MOTOREN + NUM_DEV_SENSOR + NUM_DEV_MOTOR)

/* --- Device Names --- */
#define DEVNAME_LED "rtled"
#define DEVNAME_SWITCH "rtswitch"
#define DEVNAME_BUZZER "rtbuzzer"
#define DEVNAME_MOTORRAWR "rtmotor_raw_r"
#define DEVNAME_MOTORRAWL "rtmotor_raw_l"
#define DEVNAME_MOTOREN "rtmotoren"
#define DEVNAME_MOTOR "rtmotor"
#define DEVNAME_SENSOR "rtlightsensor"
#define DEVNAME_CNTR "rtcounter_r"
#define DEVNAME_CNTL "rtcounter_l"

#define DRIVER_NAME "rtmouse"

/* --- Device Major and Minor Numbers --- */
#define DEV_MAJOR 0
#define DEV_MINOR 0

static int spi_bus_num = 0;
static int spi_chip_select = 0;

static int _major_led = DEV_MAJOR;
static int _minor_led = DEV_MINOR;

static int _major_switch = DEV_MAJOR;
static int _minor_switch = DEV_MINOR;

static int _major_sensor = DEV_MAJOR;
static int _minor_sensor = DEV_MINOR;

static int _major_buzzer = DEV_MAJOR;
static int _minor_buzzer = DEV_MINOR;

static int _major_motorrawr = DEV_MAJOR;
static int _minor_motorrawr = DEV_MINOR;

static int _major_motorrawl = DEV_MAJOR;
static int _minor_motorrawl = DEV_MINOR;

static int _major_motoren = DEV_MAJOR;
static int _minor_motoren = DEV_MINOR;

static int _major_motor = DEV_MAJOR;
static int _minor_motor = DEV_MINOR;

/* --- General Options --- */
static struct cdev *cdev_array = NULL;
static struct class *class_led = NULL;
static struct class *class_buzzer = NULL;
static struct class *class_switch = NULL;
static struct class *class_sensor = NULL;
static struct class *class_motorrawr = NULL;
static struct class *class_motorrawl = NULL;
static struct class *class_motoren = NULL;
static struct class *class_motor = NULL;

static volatile void __iomem *pwm_base;
static volatile void __iomem *clk_base;
static volatile uint32_t *gpio_base;

static volatile int cdev_index = 0;

static struct mutex lock;

/* --- GPIO Pin Definitions --- */
#define R_AD_CH 3
#define L_AD_CH 0
#define RF_AD_CH 2
#define LF_AD_CH 1

#define R_LED_BASE 22
#define L_LED_BASE 4
#define RF_LED_BASE 27
#define LF_LED_BASE 17

#define LED0_BASE 25
#define LED1_BASE 24
#define LED2_BASE 23
#define LED3_BASE 18

#define SW1_PIN 20
#define SW2_PIN 26
#define SW3_PIN 21

#define BUZZER_BASE 19

#define MOTCLK_L_BASE 12
#define MOTDIR_L_BASE 16

#define MOTCLK_R_BASE 13
#define MOTDIR_R_BASE 6

#define MOTEN_BASE 5

#define PWM_ORG0_BASE 40
#define PWM_ORG1_BASE 45

/* --- Register Address --- */
/* Base Addr */
#if RASPBERRYPI == 1
#define RPI_REG_BASE 0x20000000
#elif RASPBERRYPI == 2
#define RPI_REG_BASE 0x3f000000
#elif RASPBERRYPI == 4
#define RPI_REG_BASE 0xfe000000
/* 2711 has a different mechanism for pin pull-up/down/enable  */
#define GPPUPPDN0 57 /* Pin pull-up/down for pins 15:0  */
#define GPPUPPDN1 58 /* Pin pull-up/down for pins 31:16 */
#define GPPUPPDN2 59 /* Pin pull-up/down for pins 47:32 */
#define GPPUPPDN3 60 /* Pin pull-up/down for pins 57:48 */
#endif

/* GPIO Addr */
#define RPI_GPIO_OFFSET 0x200000
#define RPI_GPIO_SIZE 0xC0
#define RPI_GPIO_BASE (RPI_REG_BASE + RPI_GPIO_OFFSET)
#define REG_GPIO_NAME "RPi mouse GPIO"

/* Pwm Addr */
#define RPI_PWM_OFFSET 0x20C000
#define RPI_PWM_SIZE 0xC0
#define RPI_PWM_BASE (RPI_REG_BASE + RPI_PWM_OFFSET)
#define REG_PWM_NAME "RPi mouse PWM"

/* Clock Addr */
#define RPI_CLK_OFFSET 0x101000
#define RPI_CLK_SIZE 0x100
#define RPI_CLK_BASE (RPI_REG_BASE + RPI_CLK_OFFSET)
#define REG_CLK_NAME "RPi mouse CLK"

/* --- General Options --- */
/* Clock Offset */
#define CLK_PWM_INDEX 0xa0
#define CLK_PWMDIV_INDEX 0xa4

/* GPIO PUPD select */
#if RASPBERRYPI == 4
#define GPIO_PULLNONE 0x0
#define GPIO_PULLUP 0x1
#define GPIO_PULLDOWN 0x2
#else
#define GPIO_PULLNONE 0x0
#define GPIO_PULLDOWN 0x1
#define GPIO_PULLUP 0x2
#endif

/* GPIO Function */
#define RPI_GPF_INPUT 0x00
#define RPI_GPF_OUTPUT 0x01
#define RPI_GPF_ALT0 0x04
#define RPI_GPF_ALT5 0x02

/* GPIO Register Index */
#define RPI_GPFSEL0_INDEX 0
#define RPI_GPFSEL1_INDEX 1
#define RPI_GPFSEL2_INDEX 2
#define RPI_GPFSEL3_INDEX 3

#define RPI_GPSET0_INDEX 7
#define RPI_GPCLR0_INDEX 10

/* GPIO Mask */
#define RPI_GPIO_P1MASK                                                        \
	(uint32_t)((0x01 << 2) | (0x01 << 3) | (0x01 << 4) | (0x01 << 7) |     \
		   (0x01 << 8) | (0x01 << 9) | (0x01 << 10) | (0x01 << 11) |   \
		   (0x01 << 14) | (0x01 << 15) | (0x01 << 17) | (0x01 << 18) | \
		   (0x01 << 22) | (0x01 << 23) | (0x01 << 24) | (0x01 << 25) | \
		   (0x01 << 27))
#define RPI_GPIO_P2MASK (uint32_t)0xffffffff

/* PWM Index */
#define RPI_PWM_CTRL 0x0
#define RPI_PWM_STA 0x4
#define RPI_PWM_DMAC 0x8
#define RPI_PWM_RNG1 0x10
#define RPI_PWM_DAT1 0x14
#define RPI_PWM_FIF1 0x18
#define RPI_PWM_RNG2 0x20
#define RPI_PWM_DAT2 0x24


#if RASPBERRYPI == 4
#define PWM_BASECLK 27000000
#else
#define PWM_BASECLK 9600000
#endif

/* A/D Parameter */
#define MCP320X_PACKET_SIZE 3
#define MCP320X_DIFF 0
#define MCP320X_SINGLE 1
#define MCP3204_CHANNELS 4

/* I2C Parameter */
#define DEV_ADDR_CNTL 0x10
#define DEV_ADDR_CNTR 0x11
#define CNT_ADDR_MSB 0x10
#define CNT_ADDR_LSB 0x11

/* --- Function Declarations --- */
static void set_motor_r_freq(int freq);
static void set_motor_l_freq(int freq);
static int mcp3204_remove(struct spi_device *spi);
static int mcp3204_probe(struct spi_device *spi);
static unsigned int mcp3204_get_value(int channel);
static int rtcnt_i2c_probe(struct i2c_client *client,
			   const struct i2c_device_id *id);
static int rtcnt_i2c_remove(struct i2c_client *client);

/* --- Variable Type definitions --- */
/* SPI */
struct mcp3204_drvdata {
	struct spi_device *spi;
	struct mutex lock;
	unsigned char tx[MCP320X_PACKET_SIZE] ____cacheline_aligned;
	unsigned char rx[MCP320X_PACKET_SIZE] ____cacheline_aligned;
	struct spi_transfer xfer ____cacheline_aligned;
	struct spi_message msg ____cacheline_aligned;
};

/* --- Static variables --- */
/* SPI device ID */
static struct spi_device_id mcp3204_id[] = {
    {"mcp3204", 0}, {},
};

/* SPI Info */
static struct spi_board_info mcp3204_info = {
    .modalias = "mcp3204",
    .max_speed_hz = 100000,
    .bus_num = 0,
    .chip_select = 0,
    .mode = SPI_MODE_3,
};

/* SPI Dirver Info */
static struct spi_driver mcp3204_driver = {
    .driver =
	{
	    .name = DEVNAME_SENSOR, .owner = THIS_MODULE,
	},
    .id_table = mcp3204_id,
    .probe = mcp3204_probe,
    .remove = mcp3204_remove,
};

/* I2C */
struct rtcnt_device_info {
	struct cdev cdev;
	unsigned int device_major;
	unsigned int device_minor;
	struct class *device_class;
	struct i2c_client *client;
	struct mutex lock;
	int signed_pulse_count;
	int raw_pulse_count;
};

static struct i2c_client *i2c_client_r = NULL;
static struct i2c_client *i2c_client_l = NULL;
static unsigned int motor_l_freq_is_positive = 1;
static unsigned int motor_r_freq_is_positive = 1;
#define SIGNED_COUNT_SIZE 32767
#define MAX_PULSE_COUNT 65535

/* I2C Device ID */
static struct i2c_device_id i2c_counter_id[] = {
    {DEVNAME_CNTL, 0}, {DEVNAME_CNTR, 1}, {},
};

/* I2C Dirver Info */
static struct i2c_driver i2c_counter_driver = {
	.driver =
	{
		.name = "rtcounter",
		.owner = THIS_MODULE,
	},
	.id_table = i2c_counter_id,
	.probe = rtcnt_i2c_probe,
	.remove = rtcnt_i2c_remove,
};

/* -- Device Addition -- */
MODULE_DEVICE_TABLE(spi, mcp3204_id);

MODULE_DEVICE_TABLE(i2c, i2c_counter_id);

/* -- Buffer -- */
#define MAX_BUFLEN 64
// static int buflen = 0;

#define MOTOR_MOTION 0
#if MOTOR_MOTION
/* Variable Type Definition for motor motion */
typedef struct {
	signed int r_hz;
	signed int l_hz;
	unsigned int time;
} t_motor_motion;

#define MAX_MOTORBUFLEN 16
static t_motor_motion motor_motion[MAX_MOTORBUFLEN];
static unsigned int motor_motion_head = 0, motor_motion_tail = 0;

static int motor_motion_push(int r_hz, int l_hz, int time)
{
	unsigned int next_tail = motor_motion_tail + 1;

	if (next_tail >= MAX_MOTORBUFLEN) {
		next_tail = 0;
	}

	if (next_tail == motor_motion_head) {
		return -1;
	}

	motor_motion[motor_motion_tail].r_hz = r_hz;
	motor_motion[motor_motion_tail].l_hz = l_hz;
	motor_motion[motor_motion_tail].time = time;

	motor_motion_tail = next_tail;

	return 0;
}

static int motor_motion_pop(t_motor_motion **ret)
{
	unsigned int next_head = motor_motion_head + 1;

	if (motor_motion_tail == motor_motion_head) {
		return -1;
	}

	if (next_head >= MAX_MOTORBUFLEN) {
		next_head = 0;
	}

	*ret = (motor_motion + motor_motion_head);

	motor_motion_head = next_head;

	return 0;
}
#endif

/* --- GPIO Operation --- */
/* getPWMCount function for GPIO Operation */
static int getPWMCount(int freq)
{
	if (freq < 1)
		return PWM_BASECLK;
	if (freq > 10000)
		return PWM_BASECLK / 10000;

	return PWM_BASECLK / freq;
}

/* set function */
static int rpi_gpio_function_set(int pin, uint32_t func)
{
	int index = RPI_GPFSEL0_INDEX + pin / 10;
	uint32_t mask = ~(0x7 << ((pin % 10) * 3));

	gpio_base[index] =
	    (gpio_base[index] & mask) | ((func & 0x7) << ((pin % 10) * 3));

	return 1;
}

/*set mask and value */
static void rpi_gpio_set32(uint32_t mask, uint32_t val)
{
	gpio_base[RPI_GPSET0_INDEX] = val & mask;
}

/* clear mask and value */
static void rpi_gpio_clear32(uint32_t mask, uint32_t val)
{
	gpio_base[RPI_GPCLR0_INDEX] = val & mask;
}

/* pwm set function */
static void rpi_pwm_write32(uint32_t offset, uint32_t val)
{
	iowrite32(val, pwm_base + offset);
}

/* left motor function */
static void set_motor_l_freq(int freq)
{
	int dat;

	rpi_gpio_function_set(BUZZER_BASE, RPI_GPF_OUTPUT);

	if (freq == 0) {
		rpi_gpio_function_set(MOTCLK_L_BASE, RPI_GPF_OUTPUT);
		return;
	} else {
		rpi_gpio_function_set(MOTCLK_L_BASE, RPI_GPF_ALT0);
	}

	if (freq > 0) {
		motor_l_freq_is_positive = 1;
		rpi_gpio_clear32(RPI_GPIO_P2MASK, 1 << MOTDIR_L_BASE);
	} else {
		motor_l_freq_is_positive = 0;
		rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << MOTDIR_L_BASE);
		freq = -freq;
	}

	dat = getPWMCount(freq);

	rpi_pwm_write32(RPI_PWM_RNG1, dat);
	rpi_pwm_write32(RPI_PWM_DAT1, dat >> 1);

	return;
}

/* right motor function */
static void set_motor_r_freq(int freq)
{
	int dat;

	rpi_gpio_function_set(BUZZER_BASE, RPI_GPF_OUTPUT);

	if (freq == 0) {
		rpi_gpio_function_set(MOTCLK_R_BASE, RPI_GPF_OUTPUT);
		return;
	} else {
		rpi_gpio_function_set(MOTCLK_R_BASE, RPI_GPF_ALT0);
	}

	if (freq > 0) {
		motor_r_freq_is_positive = 1;
		rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << MOTDIR_R_BASE);
	} else {
		motor_r_freq_is_positive = 0;
		rpi_gpio_clear32(RPI_GPIO_P2MASK, 1 << MOTDIR_R_BASE);
		freq = -freq;
	}

	dat = getPWMCount(freq);

	rpi_pwm_write32(RPI_PWM_RNG2, dat);
	rpi_pwm_write32(RPI_PWM_DAT2, dat >> 1);

	return;
}

/* --- Function for device file operations --- */
/*
 * Read Push Switches
 * return 0 : device close
 */
static ssize_t sw_read(struct file *filep, char __user *buf, size_t count,
		       loff_t *f_pos)
{
	int buflen = 0;
	unsigned char rw_buf[MAX_BUFLEN];
	unsigned int ret = 0;
	int len;
	int index;
	unsigned int pin = SW1_PIN;
	uint32_t mask;
	int minor = *((int *)filep->private_data);
#if RASPBERRYPI == 4
	int pullreg = GPPUPPDN1 + (pin >> 4); // SW1, 2, 3 is between GPIO16-31
	int pullshift = (pin & 0xf) << 1;
	unsigned int pullbits;
	unsigned int pull;
#endif

	switch (minor) {
	case 0:
		pin = SW1_PIN;
		break;
	case 1:
		pin = SW2_PIN;
		break;
	case 2:
		pin = SW3_PIN;
		break;
	default:
		return 0;
		break;
	}

	if (*f_pos > 0)
		return 0; /* End of file */

#if RASPBERRYPI == 4
	pull = GPIO_PULLUP;
	pullbits = *(gpio_base + pullreg);
	pullbits &= ~(3 << pullshift);
	pullbits |= (pull << pullshift);
	*(gpio_base + pullreg) = pullbits;
#else
	//  プルモード (2bit)を書き込む NONE/DOWN/UP
	gpio_base[37] = GPIO_PULLUP & 0x3; //  GPPUD
	//  ピンにクロックを供給（前後にウェイト）
	msleep(1);
	gpio_base[38] = 0x1 << pin; //  GPPUDCLK0
	msleep(1);
	//  プルモード・クロック状態をクリアして終了
	gpio_base[37] = 0;
	gpio_base[38] = 0;
#endif

	index = RPI_GPFSEL0_INDEX + pin / 10;
	mask = ~(0x7 << ((pin % 10) * 3));

	ret = ((gpio_base[13] & (0x01 << pin)) != 0);
	sprintf(rw_buf, "%d\n", ret);

	buflen = strlen(rw_buf);
	count = buflen;
	len = buflen;

	if (copy_to_user((void *)buf, &rw_buf, count)) {
		printk(KERN_INFO "err read buffer from ret  %d\n", ret);
		printk(KERN_INFO "err read buffer from %s\n", rw_buf);
		printk(KERN_INFO "err sample_char_read size(%d)\n", count);
		printk(KERN_INFO "sample_char_read size err(%d)\n", -EFAULT);
		return 0;
	}
	*f_pos += count;

	return count;
}

/*
 * Read Sensor information
 * return 0 : device close
 */
static ssize_t sensor_read(struct file *filep, char __user *buf, size_t count,
			   loff_t *f_pos)
{
	int buflen = 0;
	unsigned char rw_buf[MAX_BUFLEN];
	unsigned int ret = 0;
	int len;

	// printk(KERN_INFO "new\n");

	int usecs = 30;
	int rf = 0, lf = 0, r = 0, l = 0;
	int orf = 0, olf = 0, or = 0, ol = 0;

	if (*f_pos > 0)
		return 0; /* End of file */

	/* get values through MCP3204 */
	/* Right side */
	or = mcp3204_get_value(R_AD_CH);
	rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << R_LED_BASE);
	udelay(usecs);
	r = mcp3204_get_value(R_AD_CH);
	rpi_gpio_clear32(RPI_GPIO_P2MASK, 1 << R_LED_BASE);
	udelay(usecs);
	/* Left side */
	ol = mcp3204_get_value(L_AD_CH);
	rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << L_LED_BASE);
	udelay(usecs);
	l = mcp3204_get_value(L_AD_CH);
	rpi_gpio_clear32(RPI_GPIO_P2MASK, 1 << L_LED_BASE);
	udelay(usecs);
	/* Right front side */
	orf = mcp3204_get_value(RF_AD_CH);
	rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << RF_LED_BASE);
	udelay(usecs);
	rf = mcp3204_get_value(RF_AD_CH);
	rpi_gpio_clear32(RPI_GPIO_P2MASK, 1 << RF_LED_BASE);
	udelay(usecs);
	/* Left front side */
	olf = mcp3204_get_value(LF_AD_CH);
	rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << LF_LED_BASE);
	udelay(usecs);
	lf = mcp3204_get_value(LF_AD_CH);
	rpi_gpio_clear32(RPI_GPIO_P2MASK, 1 << LF_LED_BASE);
	udelay(usecs);

	/* set sensor data to rw_buf(static buffer) */
	snprintf(rw_buf, sizeof(rw_buf), "%d %d %d %d\n", rf - orf, r - or,
		 l - ol, lf - olf);
	buflen = strlen(rw_buf);
	count = buflen;
	len = buflen;

	/* copy data to user area */
	if (copy_to_user((void *)buf, &rw_buf, count)) {
		printk(KERN_INFO "err read buffer from ret  %d\n", ret);
		printk(KERN_INFO "err read buffer from %s\n", rw_buf);
		printk(KERN_INFO "err sample_char_read size(%d)\n", count);
		printk(KERN_INFO "sample_char_read size err(%d)\n", -EFAULT);
		return 0;
	}

	*f_pos += count;

	return count;
}

/*
 * Turn On LEDs
 * return 0 : device close
 */
static int led_put(int ledno)
{
	switch (ledno) {
	case 0:
		rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << LED0_BASE);
		break;
	case 1:
		rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << LED1_BASE);
		break;
	case 2:
		rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << LED2_BASE);
		break;
	case 3:
		rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << LED3_BASE);
		break;
	}
	return 0;
}

/*
 * Turn Off LEDs
 * return 0 : device close
 */
static int led_del(int ledno)
{
	switch (ledno) {
	case 0:
		rpi_gpio_clear32(RPI_GPIO_P2MASK, 1 << LED0_BASE);
		break;
	case 1:
		rpi_gpio_clear32(RPI_GPIO_P2MASK, 1 << LED1_BASE);
		break;
	case 2:
		rpi_gpio_clear32(RPI_GPIO_P2MASK, 1 << LED2_BASE);
		break;
	case 3:
		rpi_gpio_clear32(RPI_GPIO_P2MASK, 1 << LED3_BASE);
		break;
	}

	return 0;
}

/*
 * Initialize buzzer
 * return 0 : device close
 */
static int buzzer_init(void)
{

	rpi_gpio_function_set(BUZZER_BASE, RPI_GPF_OUTPUT); // io is pwm out
	rpi_pwm_write32(RPI_PWM_CTRL, 0x00000000);
	udelay(1000);
	rpi_pwm_write32(RPI_PWM_CTRL, 0x00008181); // PWM1,2 enable

	// printk(KERN_DEBUG "%s: rpi_pwm_ctrl:%08X\n", DRIVER_NAME, ioread32(pwm_base + RPI_PWM_CTRL));

	return 0;
}

/* --- GPIO mapping for Device Open/Close --- */
/*
 * Get gpio addresses and set them to global variables.
 *   - gpio_base
 *   - pwm_base
 *   - clk_base
 *   - clk_status
 */
static int gpio_map(void)
{
	static int clk_status = 1;

	if (gpio_base == NULL) {
		gpio_base = ioremap_nocache(RPI_GPIO_BASE, RPI_GPIO_SIZE);
	}

	if (pwm_base == NULL) {
		pwm_base = ioremap_nocache(RPI_PWM_BASE, RPI_PWM_SIZE);
	}

	if (clk_base == NULL) {
		clk_base = ioremap_nocache(RPI_CLK_BASE, RPI_CLK_SIZE);
	}

	/* kill */
	if (clk_status == 1) {
		iowrite32(0x5a000000 | (1 << 5), clk_base + CLK_PWM_INDEX);
		udelay(1000);

		/* clk set */
		iowrite32(0x5a000000 | (2 << 12), clk_base + CLK_PWMDIV_INDEX);
		iowrite32(0x5a000011, clk_base + CLK_PWM_INDEX);

		udelay(1000); /* wait for 1msec */

		clk_status = 0;
	}

	return 0;
}

/* Unmap GPIO addresses */
static int gpio_unmap(void)
{
	iounmap(gpio_base);
	iounmap(pwm_base);
	iounmap(clk_base);

	gpio_base = NULL;
	pwm_base = NULL;
	clk_base = NULL;
	return 0;
}

/* --- Device File Operation --- */
/* Open Device */
static int dev_open(struct inode *inode, struct file *filep)
{
	int *minor = (int *)kmalloc(sizeof(int), GFP_KERNEL);
	int major = MAJOR(inode->i_rdev);
	*minor = MINOR(inode->i_rdev);

	filep->private_data = (void *)minor;

	if (_major_motor == major) {
		printk(KERN_INFO "motor write\n");
	}

	return 0;
}

/* Close device */
static int dev_release(struct inode *inode, struct file *filep)
{
	kfree(filep->private_data);
	return 0;
}

static int i2c_dev_open(struct inode *inode, struct file *filep)
{
	struct rtcnt_device_info *dev_info;
	dev_info = container_of(inode->i_cdev, struct rtcnt_device_info, cdev);
	if (dev_info == NULL || dev_info->client == NULL) {
		printk(KERN_ERR "%s: i2c dev_open failed.\n", DRIVER_NAME);
	}
	dev_info->device_minor =  MINOR(inode->i_rdev);
	filep->private_data = dev_info;
	return 0;
}

static int i2c_dev_release(struct inode *inode, struct file *filep)
{
	return 0;
}

/* Parse Frequency */
static int parseFreq(const char __user *buf, size_t count, int *ret)
{
	char cval;
	int error = 0, i = 0, tmp, bufcnt = 0, freq;
	size_t readcount = count;
	int sgn = 1;

	char *newbuf = kmalloc(sizeof(char) * count, GFP_KERNEL);

	while (readcount > 0) {
		if (copy_from_user(&cval, buf + i, sizeof(char))) {
			kfree(newbuf);
			return -EFAULT;
		}

		if (cval == '-') {
			if (bufcnt == 0) {
				sgn = -1;
			}
		} else if (cval < '0' || cval > '9') {
			newbuf[bufcnt] = 'e';
			error = 1;
		} else {
			newbuf[bufcnt] = cval;
		}

		i++;
		bufcnt++;
		readcount--;

		if (cval == '\n') {
			break;
		}
	}

	freq = 0;
	for (i = 0, tmp = 1; i < bufcnt; i++) {
		char c = newbuf[bufcnt - i - 1];

		if (c >= '0' && c <= '9') {
			freq += (newbuf[bufcnt - i - 1] - '0') * tmp;
			tmp *= 10;
		}
	}

	*ret = sgn * freq;

	kfree(newbuf);

	return bufcnt;
}

/* Parse motor command  */
static int parseMotorCmd(const char __user *buf, size_t count, int *ret)
{
	int r_motor_val, l_motor_val, time_val;
	char *newbuf = kmalloc(sizeof(char) * count, GFP_KERNEL);

	if (copy_from_user(newbuf, buf, sizeof(char) * count)) {
		kfree(newbuf);
		return -EFAULT;
	}

	sscanf(newbuf, "%d%d%d\n", &l_motor_val, &r_motor_val, &time_val);

	kfree(newbuf);

	mutex_lock(&lock);

	set_motor_l_freq(l_motor_val);
	set_motor_r_freq(r_motor_val);

	msleep_interruptible(time_val);

	set_motor_l_freq(0);
	set_motor_r_freq(0);

	mutex_unlock(&lock);

	return count;
}

/* Parse I2C pulse counter value */
static int parse_count(const char __user *buf, size_t count, int *ret)
{
	char cval;
	int error = 0, i = 0, tmp, bufcnt = 0, freq;
	size_t readcount = count;
	int sgn = 1;

	char *newbuf = kmalloc(sizeof(char) * count, GFP_KERNEL);

	while (readcount > 0) {
		if (copy_from_user(&cval, buf + i, sizeof(char))) {
			kfree(newbuf);
			return -EFAULT;
		}

		if (cval == '-') {
			if (bufcnt == 0) {
				sgn = -1;
			}
		} else if (cval < '0' || cval > '9') {
			newbuf[bufcnt] = 'e';
			error = 1;
		} else {
			newbuf[bufcnt] = cval;
		}

		i++;
		bufcnt++;
		readcount--;

		if (cval == '\n') {
			break;
		}
	}

	freq = 0;
	for (i = 0, tmp = 1; i < bufcnt; i++) {
		char c = newbuf[bufcnt - i - 1];

		if (c >= '0' && c <= '9') {
			freq += (newbuf[bufcnt - i - 1] - '0') * tmp;
			tmp *= 10;
		}
	}

	*ret = sgn * freq;

	kfree(newbuf);

	return bufcnt;
}

/*
 * led_write - Trun ON/OFF LEDs
 * Write function of /dev/rtled
 */
static ssize_t led_write(struct file *filep, const char __user *buf,
			 size_t count, loff_t *f_pos)
{
	char cval;
	int ret;
	int minor = *((int *)filep->private_data);

	if (count > 0) {
		if (copy_from_user(&cval, buf, sizeof(char))) {
			return -EFAULT;
		}
		switch (cval) {
		case '1':
			ret = led_put(minor);
			break;
		case '0':
			ret = led_del(minor);
			break;
		}
		return sizeof(char);
	}
	return 0;
}

/*
 * buzzer_write - Write buzzer frequency
 * Write function of /dev/rtbuzzer
 */
static ssize_t buzzer_write(struct file *filep, const char __user *buf,
			    size_t count, loff_t *f_pos)
{
	int bufcnt;
	int freq, dat;

	bufcnt = parseFreq(buf, count, &freq);

	if (freq != 0) {
		if (freq < 1) {
			freq = 1;
		}

		if (freq > 20000) {
			freq = 20000;
		}

		rpi_gpio_function_set(BUZZER_BASE,
				      RPI_GPF_ALT5); // io is pwm out
		dat = PWM_BASECLK / freq;
		rpi_pwm_write32(RPI_PWM_RNG2, dat);
		rpi_pwm_write32(RPI_PWM_DAT2, dat >> 1);
	} else {
		rpi_gpio_function_set(BUZZER_BASE,
				      RPI_GPF_OUTPUT); // io is pwm out
	}

	return bufcnt;
}

/*
 *  rawmotor_l_write - Output frequency to the left motor
 *  Write function of /dev/rtmotor_raw_l
 */
static ssize_t rawmotor_l_write(struct file *filep, const char __user *buf,
				size_t count, loff_t *f_pos)
{
	int freq, bufcnt;
	bufcnt = parseFreq(buf, count, &freq);

	set_motor_l_freq(freq);

	return bufcnt;
}

/*
 *  rawmotor_r_write - Output frequency to the right motor
 *  Write function of /dev/rtmotor_raw_r
 */
static ssize_t rawmotor_r_write(struct file *filep, const char __user *buf,
				size_t count, loff_t *f_pos)
{
	int freq, bufcnt;
	bufcnt = parseFreq(buf, count, &freq);

	set_motor_r_freq(freq);

	return bufcnt;
}

/*
 * motoren_write - Turn ON/OFF SteppingMotor Power
 * Write function of /dev/rtmotoren
 */
static ssize_t motoren_write(struct file *filep, const char __user *buf,
			     size_t count, loff_t *f_pos)
{
	char cval;

	if (count > 0) {
		if (copy_from_user(&cval, buf, sizeof(char))) {
			return -EFAULT;
		}

		switch (cval) {
		case '1':
			rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << MOTEN_BASE);
			break;
		case '0':
			rpi_gpio_clear32(RPI_GPIO_P2MASK, 1 << MOTEN_BASE);
			break;
		}
		return sizeof(char);
	}
	return 0;
}

/*
 *  motor_write - Output frequency to right and left both motors
 *  Write function of /dev/rtmotor
 */
static ssize_t motor_write(struct file *filep, const char __user *buf,
			   size_t count, loff_t *f_pos)
{
	int tmp;
	int bufcnt;
	bufcnt = parseMotorCmd(buf, count, &tmp);

	return bufcnt;
}

/*
 * i2c_counter_set - set value to I2C pulse counter
 * called by cntr_write() and cntl_write()
 */
static int i2c_counter_set(struct rtcnt_device_info *dev_info, int setval)
{
	int ret = 0;
	int lsb = 0, msb = 0;
	struct i2c_client *client = dev_info->client;

	// printk(KERN_INFO "set 0x%x = 0x%x\n", client->addr, setval);
	msb = (setval >> 8) & 0xFF;
	lsb = setval & 0xFF;
	mutex_lock(&dev_info->lock);
	// printk(KERN_INFO "set 0x%x msb = 0x%x\n", client->addr, msb);
	ret = i2c_smbus_write_byte_data(client, 0x10, msb);
	if (ret < 0) {
		printk(KERN_ERR
		       "%s: Failed writing to i2c counter device, addr=0x%x\n",
		       __func__, client->addr);
		return -ENODEV;
	}
	// printk(KERN_INFO "set 0x%x lsb = 0x%x\n", client->addr, lsb);
	ret = i2c_smbus_write_byte_data(client, 0x11, lsb);
	if (ret < 0) {
		printk(KERN_ERR
		       "%s: Failed writing to i2c counter device, addr=0x%x\n",
		       __func__, client->addr);
		return -ENODEV;
	}
	mutex_unlock(&dev_info->lock);
	return ret;
}

/*
 * i2c_counter_read - get value from I2C pulse counter
 * called by rtcnt_read()
 */
static int i2c_counter_read(struct rtcnt_device_info *dev_info, int *ret)
{
	int lsb = 0, msb = 0;
	// printk(KERN_INFO "read 0x%x\n", client->addr);
	struct i2c_client *client = dev_info->client;
	mutex_lock(&dev_info->lock);

	lsb = i2c_smbus_read_byte_data(client, CNT_ADDR_LSB);
	if (lsb < 0) {
		printk(
		    KERN_ERR
		    "%s: Failed reading from i2c counter device, addr=0x%x\n",
		    __func__, client->addr);
		return -ENODEV;
	}
	msb = i2c_smbus_read_byte_data(client, CNT_ADDR_MSB);
	if (msb < 0) {
		printk(
		    KERN_ERR
		    "%s: Failed reading from i2c counter device, addr=0x%x\n",
		    __func__, client->addr);
		return -ENODEV;
	}
	mutex_unlock(&dev_info->lock);

	*ret = ((msb << 8) & 0xFF00) + (lsb & 0xFF);

	// printk(KERN_INFO "0x%x == 0x%x\n", client->addr, *ret);
	return 0;
}

/*
 * update_signed_count - update signed pulse count of dev_info
 * called by rtcnt_read()
 */
void update_signed_count(struct rtcnt_device_info *dev_info,
	int rtcnt_count)
{
	int diff_count = rtcnt_count - dev_info->raw_pulse_count;

	// カウントがMAX_PULSE_COUNTから0に変わる場合の処理
	// ただし、それ以外でもdiffが負の値になることがある
	// そのため、diffが十分に大きな負の値の場合に処理する
	// if(diff_count < 0) では正常に動作しない
	if(diff_count < -SIGNED_COUNT_SIZE){
		diff_count += MAX_PULSE_COUNT;
	}

	if(dev_info->client->addr == DEV_ADDR_CNTL){
		if(motor_l_freq_is_positive){
			dev_info->signed_pulse_count += diff_count;
		}else{
			dev_info->signed_pulse_count -= diff_count;
		}
	}else{
		if(motor_r_freq_is_positive){
			dev_info->signed_pulse_count += diff_count;
		}else{
			dev_info->signed_pulse_count -= diff_count;
		}
	}

	if(dev_info->signed_pulse_count > SIGNED_COUNT_SIZE ||
		dev_info->signed_pulse_count < -SIGNED_COUNT_SIZE){
		dev_info->signed_pulse_count = 0;
	}
}

/*
 * reset_signed_count - reset signed pulse count of dev_info
 * called by rtcnt_write()
 */
void reset_signed_count(struct rtcnt_device_info *dev_info,
	int rtcnt_count)
{
	int raw_count;

	if(rtcnt_count > SIGNED_COUNT_SIZE){
		rtcnt_count = SIGNED_COUNT_SIZE;
	}else if(rtcnt_count < - SIGNED_COUNT_SIZE){
		rtcnt_count = -SIGNED_COUNT_SIZE;
	}
	dev_info->signed_pulse_count = rtcnt_count;
	i2c_counter_read(dev_info, &raw_count);
	dev_info->raw_pulse_count = raw_count;
}

/*
 *  rtcnt_read - Read value from right/left pulse counter
 *  Read function of /dev/rtcounter_*
 */
static ssize_t rtcnt_read(struct file *filep, char __user *buf, size_t count,
			  loff_t *f_pos)
{
	struct rtcnt_device_info *dev_info = filep->private_data;

	unsigned char rw_buf[64];
	int buflen;

	int rtcnt_count = 0;
	if (*f_pos > 0)
		return 0; /* close device */
	i2c_counter_read(dev_info, &rtcnt_count);

	if(dev_info->device_minor == 1){
		update_signed_count(dev_info, rtcnt_count);
		dev_info->raw_pulse_count = rtcnt_count;
		rtcnt_count = dev_info->signed_pulse_count;
	}else{
		dev_info->raw_pulse_count = rtcnt_count;
	}

	/* set sensor data to rw_buf(static buffer) */
	sprintf(rw_buf, "%d\n", rtcnt_count);
	buflen = strlen(rw_buf);
	count = buflen;

	/* copy data to user area */
	if (copy_to_user((void *)buf, &rw_buf, count)) {
		printk(KERN_INFO "err read buffer from %s\n", rw_buf);
		printk(KERN_INFO "err sample_char_read size(%d)\n", count);
		printk(KERN_INFO "sample_char_read size err(%d)\n", -EFAULT);
		return -EFAULT;
	}
	*f_pos += count;
	return count;
}

/*
 *  cnt_write - Set value to right/left pulse counter
 *  Write function of /dev/rtcounter
 */
static ssize_t rtcnt_write(struct file *filep, const char __user *buf,
			   size_t count, loff_t *pos)
{
	struct rtcnt_device_info *dev_info = filep->private_data;

	int bufcnt = 0;
	int rtcnt_count = 0;

	if (count < 0)
		return 0;

	bufcnt = parse_count(buf, count, &rtcnt_count);

	i2c_counter_set(dev_info, rtcnt_count);

	if(dev_info->device_minor == 1){
		reset_signed_count(dev_info, rtcnt_count);
	}

	printk(KERN_INFO "%s: set pulse counter value %d\n", DRIVER_NAME,
	       rtcnt_count);
	return bufcnt;
}

/* --- Device File Operations --- */
/* /dev/rtled */
static struct file_operations led_fops = {
    .open = dev_open, .release = dev_release, .write = led_write,
};
/* /dev/rtbuzzer */
static struct file_operations buzzer_fops = {
    .open = dev_open, .release = dev_release, .write = buzzer_write,
};
/* /dev/rtswitch */
static struct file_operations sw_fops = {
    .open = dev_open, .read = sw_read, .release = dev_release,
};
/* /dev/rtlightsensor */
static struct file_operations sensor_fops = {
    .open = dev_open, .read = sensor_read, .release = dev_release,
};
/* /dev/rtmotor_raw_r */
static struct file_operations motorrawr_fops = {
    .open = dev_open, .write = rawmotor_r_write, .release = dev_release,
};
/* /dev/rtmotor_raw_l */
static struct file_operations motorrawl_fops = {
    .open = dev_open, .write = rawmotor_l_write, .release = dev_release,
};
/* /dev/rtmotoren */
static struct file_operations motoren_fops = {
    .open = dev_open, .write = motoren_write, .release = dev_release,
};
/* /dev/rtmotor */
static struct file_operations motor_fops = {
    .open = dev_open, .write = motor_write, .release = dev_release,
};
/* /dev/rtcounter_* */
static struct file_operations rtcnt_fops = {
    .open = i2c_dev_open,
    .release = i2c_dev_release,
    .read = rtcnt_read,
    .write = rtcnt_write,
};

/* --- Device Driver Registration and Device File Creation --- */
/* /dev/rtled0,/dev/rtled1,/dev/rtled2 */
static int led_register_dev(void)
{
	int retval;
	dev_t dev;
	dev_t devno;
	int i;

	/* 空いているメジャー番号を使ってメジャー&
	   マイナー番号をカーネルに登録する */
	retval = alloc_chrdev_region(&dev, /* 結果を格納するdev_t構造体 */
				     DEV_MINOR, /* ベースマイナー番号 */
				     NUM_DEV_LED, /* デバイスの数 */
				     DEVNAME_LED /* デバイスドライバの名前 */
				     );

	if (retval < 0) {
		printk(KERN_ERR "alloc_chrdev_region failed.\n");
		return retval;
	}
	_major_led = MAJOR(dev);

	/* デバイスクラスを作成する */
	class_led = class_create(THIS_MODULE, DEVNAME_LED);
	if (IS_ERR(class_led)) {
		return PTR_ERR(class_led);
	}

	for (i = 0; i < NUM_DEV_LED; i++) {
		/* デバイスの数だけキャラクタデバイスを登録する */
		devno = MKDEV(_major_led, _minor_led + i);

		/* キャラクタデバイスとしてこのモジュールをカーネルに登録する */
		cdev_init(&(cdev_array[cdev_index]), &led_fops);
		cdev_array[cdev_index].owner = THIS_MODULE;
		if (cdev_add(&(cdev_array[cdev_index]), devno, 1) < 0) {
			/* 登録に失敗した */
			printk(KERN_ERR "cdev_add failed minor = %d\n",
			       _minor_led + i);
		} else {
			/* デバイスノードの作成 */
			device_create(class_led, NULL, devno, NULL,
				      DEVNAME_LED "%u", _minor_led + i);
		}
		cdev_index++;
	}

	return 0;
}

/* /dev/rtbuzzer0 */
static int buzzer_register_dev(void)
{
	int retval;
	dev_t dev;
	dev_t devno;

	/* 空いているメジャー番号を使ってメジャー&
	   マイナー番号をカーネルに登録する */
	retval = alloc_chrdev_region(&dev, /* 結果を格納するdev_t構造体 */
				     DEV_MINOR, /* ベースマイナー番号 */
				     NUM_DEV_BUZZER, /* デバイスの数 */
				     DEVNAME_BUZZER /* デバイスドライバの名前 */
				     );

	if (retval < 0) {
		printk(KERN_ERR "alloc_chrdev_region failed.\n");
		return retval;
	}
	_major_buzzer = MAJOR(dev);

	/* デバイスクラスを作成する */
	class_buzzer = class_create(THIS_MODULE, DEVNAME_BUZZER);
	if (IS_ERR(class_buzzer)) {
		return PTR_ERR(class_buzzer);
	}

	/*  create device file */
	devno = MKDEV(_major_buzzer, _minor_buzzer);
	/* キャラクタデバイスとしてこのモジュールをカーネルに登録する */
	cdev_init(&(cdev_array[cdev_index]), &buzzer_fops);
	cdev_array[cdev_index].owner = THIS_MODULE;
	if (cdev_add(&(cdev_array[cdev_index]), devno, 1) < 0) {
		/* 登録に失敗した */
		printk(KERN_ERR "cdev_add failed minor = %d\n", _minor_buzzer);
	} else {
		/* デバイスノードの作成 */
		device_create(class_buzzer, NULL, devno, NULL,
			      DEVNAME_BUZZER "%u", _minor_buzzer);
	}
	cdev_index++;

	return 0;
}

/* /dev/rtmotor_raw_r0 */
static int motorrawr_register_dev(void)
{
	int retval;
	dev_t dev;
	dev_t devno;

	/* 空いているメジャー番号を使ってメジャー&
	   マイナー番号をカーネルに登録する */
	retval =
	    alloc_chrdev_region(&dev, /* 結果を格納するdev_t構造体 */
				DEV_MINOR, /* ベースマイナー番号 */
				NUM_DEV_MOTORRAWR, /* デバイスの数 */
				DEVNAME_MOTORRAWR /* デバイスドライバの名前 */
				);

	if (retval < 0) {
		printk(KERN_ERR "alloc_chrdev_region failed.\n");
		return retval;
	}
	_major_motorrawr = MAJOR(dev);

	/* デバイスクラスを作成する */
	class_motorrawr = class_create(THIS_MODULE, DEVNAME_MOTORRAWR);
	if (IS_ERR(class_buzzer)) {
		return PTR_ERR(class_buzzer);
	}

	devno = MKDEV(_major_motorrawr, _minor_motorrawr);

	/* キャラクタデバイスとしてこのモジュールをカーネルに登録する */
	cdev_init(&(cdev_array[cdev_index]), &motorrawr_fops);
	cdev_array[cdev_index].owner = THIS_MODULE;
	if (cdev_add(&(cdev_array[cdev_index]), devno, 1) < 0) {
		/* 登録に失敗した */
		printk(KERN_ERR "cdev_add failed minor = %d\n",
		       _minor_motorrawr);
	} else {
		/* デバイスノードの作成 */
		device_create(class_motorrawr, NULL, devno, NULL,
			      DEVNAME_MOTORRAWR "%u", _minor_motorrawr);
	}

	cdev_index++;

	return 0;
}

/* /dev/rtmotor_raw_r0 */
static int motorrawl_register_dev(void)
{
	int retval;
	dev_t dev;
	dev_t devno;

	/* 空いているメジャー番号を使ってメジャー&
	   マイナー番号をカーネルに登録する */
	retval =
	    alloc_chrdev_region(&dev, /* 結果を格納するdev_t構造体 */
				DEV_MINOR, /* ベースマイナー番号 */
				NUM_DEV_MOTORRAWL, /* デバイスの数 */
				DEVNAME_MOTORRAWL /* デバイスドライバの名前 */
				);

	if (retval < 0) {
		printk(KERN_ERR "alloc_chrdev_region failed.\n");
		return retval;
	}
	_major_motorrawl = MAJOR(dev);

	/* デバイスクラスを作成する */
	class_motorrawl = class_create(THIS_MODULE, DEVNAME_MOTORRAWL);
	if (IS_ERR(class_buzzer)) {
		return PTR_ERR(class_buzzer);
	}

	devno = MKDEV(_major_motorrawl, _minor_motorrawl);
	/* キャラクタデバイスとしてこのモジュールをカーネルに登録する */
	cdev_init(&(cdev_array[cdev_index]), &motorrawl_fops);
	cdev_array[cdev_index].owner = THIS_MODULE;
	if (cdev_add(&(cdev_array[cdev_index]), devno, 1) < 0) {
		/* 登録に失敗した */
		printk(KERN_ERR "cdev_add failed minor = %d\n",
		       _minor_motorrawl);
	} else {
		/* デバイスノードの作成 */
		device_create(class_motorrawl, NULL, devno, NULL,
			      DEVNAME_MOTORRAWL "%u", _minor_motorrawl);
	}

	cdev_index++;

	return 0;
}

/* /dev/rtswitch0, /dev/rtswitch1, /dev/rtswitch2 */
static int switch_register_dev(void)
{
	int retval;
	dev_t dev;
	int i;
	dev_t devno;

	/* 空いているメジャー番号を使ってメジャー&
		マイナー番号をカーネルに登録する */
	retval = alloc_chrdev_region(&dev, /* 結果を格納するdev_t構造体 */
				     DEV_MINOR, /* ベースマイナー番号 */
				     NUM_DEV_SWITCH, /* デバイスの数 */
				     DEVNAME_SWITCH /* デバイスドライバの名前 */
				     );

	if (retval < 0) {
		printk(KERN_ERR "alloc_chrdev_region failed.\n");
		return retval;
	}
	_major_switch = MAJOR(dev);

	/* デバイスクラスを作成する */
	class_switch = class_create(THIS_MODULE, DEVNAME_SWITCH);
	if (IS_ERR(class_switch)) {
		return PTR_ERR(class_switch);
	}

	/* デバイスの数だけキャラクタデバイスを登録する */
	for (i = 0; i < NUM_DEV_SWITCH; i++) {
		devno = MKDEV(_major_switch, _minor_switch + i);
		/* キャラクタデバイスとしてこのモジュールをカーネルに登録する */
		cdev_init(&(cdev_array[cdev_index]), &sw_fops);
		cdev_array[cdev_index].owner = THIS_MODULE;

		if (cdev_add(&(cdev_array[cdev_index]), devno, 1) < 0) {
			/* 登録に失敗した */
			printk(KERN_ERR "cdev_add failed minor = %d\n",
			       _minor_switch + i);
		} else {
			/* デバイスノードの作成 */
			device_create(class_switch, NULL, devno, NULL,
				      DEVNAME_SWITCH "%u", _minor_switch + i);
		}
		cdev_index++;
	}

	return 0;
}

/* /dev/rtlightsensor0 */
static int sensor_register_dev(void)
{
	int retval;
	dev_t dev;
	dev_t devno;

	/* 空いているメジャー番号を使ってメジャー
		&マイナー番号をカーネルに登録する */
	retval = alloc_chrdev_region(&dev, /* 結果を格納するdev_t構造体 */
				     DEV_MINOR, /* ベースマイナー番号 */
				     NUM_DEV_SENSOR, /* デバイスの数 */
				     DEVNAME_SENSOR /* デバイスドライバの名前 */
				     );

	if (retval < 0) {
		printk(KERN_ERR "alloc_chrdev_region failed.\n");
		return retval;
	}
	_major_sensor = MAJOR(dev);

	/* デバイスクラスを作成する */
	class_sensor = class_create(THIS_MODULE, DEVNAME_SENSOR);
	if (IS_ERR(class_sensor)) {
		return PTR_ERR(class_sensor);
	}

	/* デバイスの数だけキャラクタデバイスを登録する */
	devno = MKDEV(_major_sensor, _minor_sensor);
	/* キャラクタデバイスとしてこのモジュールをカーネルに登録する */
	cdev_init(&(cdev_array[cdev_index]), &sensor_fops);
	cdev_array[cdev_index].owner = THIS_MODULE;
	if (cdev_add(&(cdev_array[cdev_index]), devno, 1) < 0) {
		/* 登録に失敗した */
		printk(KERN_ERR "cdev_add failed minor = %d\n", _minor_sensor);
	} else {
		/* デバイスノードの作成 */
		device_create(class_sensor, NULL, devno, NULL,
			      DEVNAME_SENSOR "%u", _minor_sensor);
	}
	cdev_index++;

	return 0;
}

/* /dev/rtmotoren0 */
static int motoren_register_dev(void)
{
	int retval;
	dev_t dev;
	dev_t devno;

	/* 空いているメジャー番号を使ってメジャー
		&マイナー番号をカーネルに登録する */
	retval =
	    alloc_chrdev_region(&dev, /* 結果を格納するdev_t構造体 */
				DEV_MINOR, /* ベースマイナー番号 */
				NUM_DEV_MOTOREN, /* デバイスの数 */
				DEVNAME_MOTOREN /* デバイスドライバの名前 */
				);

	if (retval < 0) {
		printk(KERN_ERR "alloc_chrdev_region failed.\n");
		return retval;
	}
	_major_motoren = MAJOR(dev);

	/* デバイスクラスを作成する */
	class_motoren = class_create(THIS_MODULE, DEVNAME_MOTOREN);
	if (IS_ERR(class_motoren)) {
		return PTR_ERR(class_motoren);
	}

	devno = MKDEV(_major_motoren, _minor_motoren);
	/* キャラクタデバイスとしてこのモジュールをカーネルに登録する */
	cdev_init(&(cdev_array[cdev_index]), &motoren_fops);
	cdev_array[cdev_index].owner = THIS_MODULE;
	if (cdev_add(&(cdev_array[cdev_index]), devno, 1) < 0) {
		/* 登録に失敗した */
		printk(KERN_ERR "cdev_add failed minor = %d\n", _minor_motoren);
	} else {
		/* デバイスノードの作成 */
		device_create(class_motoren, NULL, devno, NULL,
			      DEVNAME_MOTOREN "%u", _minor_motoren);
	}

	cdev_index++;

	return 0;
}

/* /dev/rtmotor0 */
static int motor_register_dev(void)
{
	int retval;
	dev_t dev;
	dev_t devno;

	/* 空いているメジャー番号を使ってメジャー
		&マイナー番号をカーネルに登録する */
	retval = alloc_chrdev_region(&dev, /* 結果を格納するdev_t構造体 */
				     DEV_MINOR, /* ベースマイナー番号 */
				     NUM_DEV_MOTOR, /* デバイスの数 */
				     DEVNAME_MOTOR /* デバイスドライバの名前 */
				     );

	if (retval < 0) {
		printk(KERN_ERR "alloc_chrdev_region failed.\n");
		return retval;
	}
	_major_motor = MAJOR(dev);

	/* デバイスクラスを作成する */
	class_motor = class_create(THIS_MODULE, DEVNAME_MOTOR);
	if (IS_ERR(class_motor)) {
		return PTR_ERR(class_motor);
	}

	devno = MKDEV(_major_motor, _minor_motor);
	/* キャラクタデバイスとしてこのモジュールをカーネルに登録する */
	cdev_init(&(cdev_array[cdev_index]), &motor_fops);
	cdev_array[cdev_index].owner = THIS_MODULE;
	if (cdev_add(&(cdev_array[cdev_index]), devno, 1) < 0) {
		/* 登録に失敗した */
		printk(KERN_ERR "cdev_add failed minor = %d\n", _minor_motor);
	} else {
		/* デバイスノードの作成 */
		device_create(class_motor, NULL, devno, NULL,
			      DEVNAME_MOTOR "%u", _minor_motor);
	}

	cdev_index++;

	return 0;
}

/* mcp3204_remove - remove function lined with spi_dirver */
static int mcp3204_remove(struct spi_device *spi)
{
	struct mcp3204_drvdata *data;
	/* get drvdata */
	data = (struct mcp3204_drvdata *)spi_get_drvdata(spi);
	/* free kernel memory */
	kfree(data);
	printk(KERN_INFO "%s: mcp3204 removed\n", DRIVER_NAME);
	return 0;
}

/* mcp3204_probe - probe function lined with spi_dirver */
static int mcp3204_probe(struct spi_device *spi)
{
	struct mcp3204_drvdata *data;

	spi->max_speed_hz = mcp3204_info.max_speed_hz;
	spi->mode = mcp3204_info.mode;
	spi->bits_per_word = 8;

	if (spi_setup(spi)) {
		printk(KERN_ERR "%s:spi_setup failed!\n", __func__);
		return -ENODEV;
	}

	/* alloc kernel memory */
	data = kzalloc(sizeof(struct mcp3204_drvdata), GFP_KERNEL);
	if (data == NULL) {
		printk(KERN_ERR "%s:kzalloc() failed!\n", __func__);
		return -ENODEV;
	}

	data->spi = spi;

	mutex_init(&data->lock);

	// memset(data->tx, 0, MCP320X_PACKET_SIZE);
	// memset(data->rx, 0, MCP320X_PACKET_SIZE);

	data->xfer.tx_buf = data->tx;
	data->xfer.rx_buf = data->rx;
	data->xfer.bits_per_word = 8;
	data->xfer.len = MCP320X_PACKET_SIZE;
	data->xfer.cs_change = 0;
	data->xfer.delay_usecs = 0;
	data->xfer.speed_hz = 100000;

	spi_message_init_with_transfers(&data->msg, &data->xfer, 1);

	/* set drvdata */
	spi_set_drvdata(spi, data);

	printk(KERN_INFO "%s: mcp3204 probed", DRIVER_NAME);

	return 0;
}

/*
 * mcp3204_get_value - get sensor data from MCP3204
 * called by 'sensor_read'
 */
static unsigned int mcp3204_get_value(int channel)
{
	struct device *dev;
	struct mcp3204_drvdata *data;
	struct spi_device *spi;
	char str[128];
	struct spi_master *master;

	unsigned int r = 0;
	unsigned char c = channel & 0x03;

	master = spi_busnum_to_master(mcp3204_info.bus_num);
	snprintf(str, sizeof(str), "%s.%u", dev_name(&master->dev),
		 mcp3204_info.chip_select);

	dev = bus_find_device_by_name(&spi_bus_type, NULL, str);
	spi = to_spi_device(dev);
	data = (struct mcp3204_drvdata *)spi_get_drvdata(spi);

	mutex_lock(&data->lock);
	data->tx[0] = 1 << 2;  // start bit
	data->tx[0] |= 1 << 1; // Single
	data->tx[1] = c << 6;  // channel
	data->tx[2] = 0;

	if (spi_sync(data->spi, &data->msg)) {
		printk(KERN_INFO "%s: spi_sync_transfer returned non zero\n",
		       __func__);
	}

	mutex_unlock(&data->lock);

	r = (data->rx[1] & 0xf) << 8;
	r |= data->rx[2];

	// printk(KERN_INFO "%s: get result on ch[%d] : %04d\n", __func__,
	// channel, r);

	return r;
}

/*
 * spi_remove_device - remove SPI device
 * called by mcp3204_init and mcp3204_exit
 */
static void spi_remove_device(struct spi_master *master, unsigned int cs)
{
	struct device *dev;
	char str[128];

	snprintf(str, sizeof(str), "%s.%u", dev_name(&master->dev), cs);

	dev = bus_find_device_by_name(&spi_bus_type, NULL, str);
	//ここを参考にspi_deviceを取得するプログラムを作成する
	if (dev) {
		device_del(dev);
	}
}

/*
 * mcp3204_init - initialize MCP3204
 * called by 'dev_init_module'
 */
static int mcp3204_init(void)
{
	struct spi_master *master;
	struct spi_device *spi_device;

	spi_register_driver(&mcp3204_driver);

	mcp3204_info.bus_num = spi_bus_num;
	mcp3204_info.chip_select = spi_chip_select;

	master = spi_busnum_to_master(mcp3204_info.bus_num);

	if (!master) {
		printk(KERN_ERR "%s: spi_busnum_to_master returned NULL\n",
		       __func__);
		spi_unregister_driver(&mcp3204_driver);
		return -ENODEV;
	}

	spi_remove_device(master, mcp3204_info.chip_select);

	spi_device = spi_new_device(master, &mcp3204_info);
	if (!spi_device) {
		printk(KERN_ERR "%s: spi_new_device returned NULL\n", __func__);
		spi_unregister_driver(&mcp3204_driver);
		return -ENODEV;
	}

	return 0;
}

/*
 * mcp3204_exit - cleanup MCP3204
 * called by dev_cleanup_module()
 */
static void mcp3204_exit(void)
{
	struct spi_master *master;

	master = spi_busnum_to_master(mcp3204_info.bus_num);

	if (master) {
		spi_remove_device(master, mcp3204_info.chip_select);
	} else {
		printk(KERN_ERR "mcp3204 remove error\n");
	}

	spi_unregister_driver(&mcp3204_driver);
}

static int rtcntr_i2c_create_cdev(struct rtcnt_device_info *dev_info)
{
	int minor;
	int alloc_ret = 0;
	int cdev_err = 0;
	dev_t dev;

	/* 空いているメジャー番号を確保する */
	alloc_ret =
	    alloc_chrdev_region(&dev, DEV_MINOR, NUM_DEV_CNT, DEVNAME_CNTR);
	if (alloc_ret != 0) {
		printk(KERN_ERR "alloc_chrdev_region = %d\n", alloc_ret);
		return -1;
	}

	/* 取得したdev( = メジャー番号 +  マイナー番号)
     * からメジャー番号を取得して保持しておく */
	dev_info->device_major = MAJOR(dev);
	dev = MKDEV(dev_info->device_major, DEV_MINOR);

	/* cdev構造体の初期化とシステムコールハンドラテーブルの登録 */
	cdev_init(&dev_info->cdev, &rtcnt_fops);
	dev_info->cdev.owner = THIS_MODULE;

	/* このデバイスドライバ(cdev)をカーネルに登録する */
	cdev_err = cdev_add(&dev_info->cdev, dev, NUM_DEV_CNT);
	if (cdev_err != 0) {
		printk(KERN_ERR "cdev_add = %d\n", alloc_ret);
		unregister_chrdev_region(dev, NUM_DEV_CNT);
		return -1;
	}

	/* このデバイスのクラス登録をする(/sys/class/mydevice/ を作る) */
	dev_info->device_class = class_create(THIS_MODULE, DEVNAME_CNTR);
	if (IS_ERR(dev_info->device_class)) {
		printk(KERN_ERR "class_create\n");
		cdev_del(&dev_info->cdev);
		unregister_chrdev_region(dev, NUM_DEV_CNT);
		return -1;
	}

	for (minor = DEV_MINOR; minor < DEV_MINOR + NUM_DEV_CNT; minor++) {
		device_create(dev_info->device_class, NULL,
			      MKDEV(dev_info->device_major, minor), NULL,
			      "rtcounter_r%d", minor);
	}

	return 0;
}

static int rtcntl_i2c_create_cdev(struct rtcnt_device_info *dev_info)
{
	int minor;
	int alloc_ret = 0;
	int cdev_err = 0;
	dev_t dev;

	/* 空いているメジャー番号を確保する */
	alloc_ret =
	    alloc_chrdev_region(&dev, DEV_MINOR, NUM_DEV_CNT, DEVNAME_CNTL);
	if (alloc_ret != 0) {
		printk(KERN_ERR "alloc_chrdev_region = %d\n", alloc_ret);
		return -1;
	}

	/* 取得したdev( = メジャー番号 + マイナー番号)
     * からメジャー番号を取得して保持しておく */
	dev_info->device_major = MAJOR(dev);
	dev = MKDEV(dev_info->device_major, DEV_MINOR);

	/* cdev構造体の初期化とシステムコールハンドラテーブルの登録 */
	cdev_init(&dev_info->cdev, &rtcnt_fops);
	dev_info->cdev.owner = THIS_MODULE;

	/* このデバイスドライバ(cdev)をカーネルに登録する */
	cdev_err = cdev_add(&dev_info->cdev, dev, NUM_DEV_CNT);
	if (cdev_err != 0) {
		printk(KERN_ERR "cdev_add = %d\n", alloc_ret);
		unregister_chrdev_region(dev, NUM_DEV_CNT);
		return -1;
	}

	/* このデバイスのクラス登録をする(/sys/class/mydevice/ を作る) */
	dev_info->device_class = class_create(THIS_MODULE, DEVNAME_CNTL);
	if (IS_ERR(dev_info->device_class)) {
		printk(KERN_ERR "class_create\n");
		cdev_del(&dev_info->cdev);
		unregister_chrdev_region(dev, NUM_DEV_CNT);
		return -1;
	}

	/* /sys/class/mydevice/mydevice* を作る */
	for (minor = DEV_MINOR; minor < DEV_MINOR + NUM_DEV_CNT; minor++) {
		device_create(dev_info->device_class, NULL,
			      MKDEV(dev_info->device_major, minor), NULL,
			      "rtcounter_l%d", minor);
	}

	return 0;
}

static int rtcnt_i2c_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct rtcnt_device_info *dev_info;
	int msb = 0, lsb = 0;
	// printk(KERN_DEBUG "%s: probing i2c device", __func__);

	/* check i2c device */
	// printk(KERN_DEBUG "%s: checking i2c device", __func__);
	msb = i2c_smbus_read_byte_data(client, CNT_ADDR_MSB);
	lsb = i2c_smbus_read_byte_data(client, CNT_ADDR_LSB);
	if ((msb < 0) || (lsb < 0)) {
		printk(KERN_INFO
		    "%s: rtcounter not found, or wrong i2c device probed",
		    DRIVER_NAME);
		// printk(KERN_DEBUG "%s: addr 0x%x, msb %d, lsb %d", __func__,
		//        client->addr, msb, lsb);
		return -ENODEV;
	}
	printk(KERN_INFO "%s: new i2c device probed, id.name=%s, "
			 "id.driver_data=%d, addr=0x%x\n",
	       DRIVER_NAME, id->name, (int)(id->driver_data), client->addr);

	dev_info = (struct rtcnt_device_info *)devm_kzalloc(
	    &client->dev, sizeof(struct rtcnt_device_info), GFP_KERNEL);
	dev_info->client = client;
	i2c_set_clientdata(client, dev_info);
	mutex_init(&dev_info->lock);

	/* create character device */
	if ((int)(id->driver_data) == 0) {
		if (rtcntl_i2c_create_cdev(dev_info))
			return -ENOMEM;
	} else if ((int)(id->driver_data) == 1) {
		if (rtcntr_i2c_create_cdev(dev_info))
			return -ENOMEM;
	}

	return 0;
}


/*
 * i2c_counter_init - initialize I2C counter
 * called by dev_init_module()
 */
static int i2c_counter_init(void)
{
	int retval = 0;
	struct i2c_adapter *i2c_adap_l;
	struct i2c_adapter *i2c_adap_r;
	struct i2c_board_info i2c_board_info_l = {
	    I2C_BOARD_INFO(DEVNAME_CNTL, DEV_ADDR_CNTL)};
	struct i2c_board_info i2c_board_info_r = {
	    I2C_BOARD_INFO(DEVNAME_CNTR, DEV_ADDR_CNTR)};

	// printk(KERN_DEBUG "%s: initializing i2c device", __func__);
	retval = i2c_add_driver(&i2c_counter_driver);
	if (retval != 0) {
		printk(KERN_INFO "%s: failed adding i2c device", DRIVER_NAME);
		return retval;
	}

	/*
	 * 動的にデバイス実体を作成
	 * (https://www.kernel.org/doc/Documentation/i2c/instantiating-devices)
	 */
	// printk(KERN_DEBUG "%s: adding i2c device", __func__);
	i2c_adap_l = i2c_get_adapter(1);
	i2c_client_l = i2c_new_device(i2c_adap_l, &i2c_board_info_l);
	i2c_put_adapter(i2c_adap_l);
	// printk(KERN_DEBUG "%s: added i2c device rtcntl", __func__);

	// printk(KERN_DEBUG "%s: adding i2c device", __func__);
	i2c_adap_r = i2c_get_adapter(1);
	i2c_client_r = i2c_new_device(i2c_adap_r, &i2c_board_info_r);
	i2c_put_adapter(i2c_adap_r);
	// printk(KERN_DEBUG "%s: added i2c device rtcntr", __func__);


	return retval;
}

/*
 * i2c_counter_exit - cleanup I2C device
 * called by dev_cleanup_module()
 */
static void i2c_counter_exit(void)
{
	/* delete I2C driver */
	i2c_del_driver(&i2c_counter_driver);
	/* free memory */
	if (i2c_client_r)
		i2c_unregister_device(i2c_client_r);
	if (i2c_client_l)
		i2c_unregister_device(i2c_client_l);
}

static void rtcnt_i2c_delete_cdev(struct rtcnt_device_info *dev_info)
{
	dev_t dev = MKDEV(dev_info->device_major, DEV_MINOR);
	int minor;
	/* /sys/class/mydevice/mydevice* を削除する */
	for (minor = DEV_MINOR; minor < DEV_MINOR + NUM_DEV_CNT; minor++) {
		device_destroy(dev_info->device_class,
			       MKDEV(dev_info->device_major, minor));
	}
	/* このデバイスのクラス登録を取り除く(/sys/class/mydevice/を削除する) */
	class_destroy(dev_info->device_class);
	/* このデバイスドライバ(cdev)をカーネルから取り除く */
	cdev_del(&dev_info->cdev);
	/* このデバイスドライバで使用していたメジャー番号の登録を取り除く */
	unregister_chrdev_region(dev, NUM_DEV_CNT);
}

/*
 * i2c_counter_remove - I2C pulse counter
 * called when I2C pulse counter removed
 */
static int rtcnt_i2c_remove(struct i2c_client *client)
{
	struct rtcnt_device_info *dev_info;
	// printk(KERN_DEBUG "%s: removing i2c device 0x%x\n", __func__, client->addr);
	dev_info = i2c_get_clientdata(client);
	rtcnt_i2c_delete_cdev(dev_info);
	printk(KERN_INFO "%s: i2c device 0x%x removed\n", DRIVER_NAME,
	       client->addr);
	return 0;
}

/*
 * dev_init_module - register driver module
 * called by module_init(dev_init_module)
 */
int dev_init_module(void)
{
	int retval, i;
	int registered_devices = 0;
	size_t size;

	/* log loding message */
	printk(KERN_INFO "%s: loading driver...\n", DRIVER_NAME);

	/* Initialize mutex lock */
	mutex_init(&lock);

	retval = i2c_counter_init();
	if (retval == 0) {
		registered_devices += 2 * NUM_DEV_CNT;
	}
	else{
		printk(KERN_ALERT
		       "%s: i2c counter device driver register failed.\n",
		       DRIVER_NAME);
		return retval;
	}

	/* GPIOレジスタがマップ可能か調べる */
	retval = gpio_map();
	if (retval != 0) {
		printk(KERN_ALERT "%s on %s: cannot use GPIO registers.\n",
		       __func__, DRIVER_NAME);
		return -EBUSY;
	}

	/* GPIO初期化 */
	// printk(KERN_DEBUG "%s: gpio initializing...\n", __func__);
	rpi_gpio_function_set(LED0_BASE, RPI_GPF_OUTPUT);
	rpi_gpio_function_set(LED1_BASE, RPI_GPF_OUTPUT);
	rpi_gpio_function_set(LED2_BASE, RPI_GPF_OUTPUT);
	rpi_gpio_function_set(LED3_BASE, RPI_GPF_OUTPUT);

	rpi_gpio_function_set(R_LED_BASE, RPI_GPF_OUTPUT);
	rpi_gpio_function_set(L_LED_BASE, RPI_GPF_OUTPUT);
	rpi_gpio_function_set(RF_LED_BASE, RPI_GPF_OUTPUT);
	rpi_gpio_function_set(LF_LED_BASE, RPI_GPF_OUTPUT);

	rpi_gpio_function_set(BUZZER_BASE, RPI_GPF_OUTPUT);
	rpi_gpio_function_set(MOTDIR_L_BASE, RPI_GPF_OUTPUT);
	rpi_gpio_function_set(MOTDIR_R_BASE, RPI_GPF_OUTPUT);
	rpi_gpio_function_set(MOTEN_BASE, RPI_GPF_OUTPUT);
	rpi_gpio_function_set(MOTCLK_L_BASE, RPI_GPF_OUTPUT);
	rpi_gpio_function_set(MOTCLK_L_BASE, RPI_GPF_OUTPUT);

	rpi_gpio_function_set(SW1_PIN, RPI_GPF_INPUT);
	rpi_gpio_function_set(SW2_PIN, RPI_GPF_INPUT);
	rpi_gpio_function_set(SW3_PIN, RPI_GPF_INPUT);

	rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << MOTEN_BASE);
	rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << MOTDIR_L_BASE);

	rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << R_LED_BASE);
	rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << L_LED_BASE);
	rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << RF_LED_BASE);
	rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << LF_LED_BASE);

	for (i = 0; i < 100; i++) {
		rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << BUZZER_BASE);
		udelay(500);
		rpi_gpio_clear32(RPI_GPIO_P2MASK, 1 << BUZZER_BASE);
		udelay(500);
	}

	buzzer_init();

	rpi_gpio_clear32(RPI_GPIO_P2MASK, 1 << R_LED_BASE);
	rpi_gpio_clear32(RPI_GPIO_P2MASK, 1 << L_LED_BASE);
	rpi_gpio_clear32(RPI_GPIO_P2MASK, 1 << RF_LED_BASE);
	rpi_gpio_clear32(RPI_GPIO_P2MASK, 1 << LF_LED_BASE);

	// printk(KERN_DEBUG "%s: gpio initialized\n", __func__);

	/* cdev構造体の用意 */
	size = sizeof(struct cdev) * NUM_DEV_TOTAL;
	cdev_array = (struct cdev *)kmalloc(size, GFP_KERNEL);

	/* デバイスドライバをカーネルに登録 */
	retval = led_register_dev();
	if (retval != 0) {
		printk(KERN_ALERT "%s: led driver register failed.\n",
		       DRIVER_NAME);
		return retval;
	}

	retval = buzzer_register_dev();
	if (retval != 0) {
		printk(KERN_ALERT "%s: buzzer driver register failed.\n",
		       DRIVER_NAME);
		return retval;
	}

	retval = switch_register_dev();
	if (retval != 0) {
		printk(KERN_ALERT "%s: switch driver register failed.\n",
		       DRIVER_NAME);
		return retval;
	}

	retval = sensor_register_dev();
	if (retval != 0) {
		printk(KERN_ALERT "%s: switch driver register failed.\n",
		       DRIVER_NAME);
		return retval;
	}

	retval = motorrawr_register_dev();
	if (retval != 0) {
		printk(KERN_ALERT "%s: motor driver register failed.\n",
		       DRIVER_NAME);
		return retval;
	}

	retval = motorrawl_register_dev();
	if (retval != 0) {
		printk(KERN_ALERT "%s: motor driver register failed.\n",
		       DRIVER_NAME);
		return retval;
	}

	retval = motoren_register_dev();
	if (retval != 0) {
		printk(KERN_ALERT "%s: motor driver register failed.\n",
		       DRIVER_NAME);
		return retval;
	}

	retval = motor_register_dev();
	if (retval != 0) {
		printk(KERN_ALERT "%s: motor driver register failed.\n",
		       DRIVER_NAME);
		return retval;
	}

	retval = mcp3204_init();
	if (retval != 0) {
		printk(KERN_ALERT
		       "%s: optical sensor driver register failed.\n",
		       DRIVER_NAME);
		return retval;
	}

	printk(KERN_INFO "%s: %d devices loaded.\n", DRIVER_NAME,
	       registered_devices+NUM_DEV_TOTAL);

	printk(KERN_INFO "%s: module installed at %lu\n", DRIVER_NAME, jiffies);
	return 0;
}

/*
 * dev_cleanup_module - cleanup driver module
 * called by module_exit(dev_cleanup_module)
 */
void dev_cleanup_module(void)
{
	int i;
	dev_t devno;
	dev_t devno_top;

	/* --- remove char device --- */
	for (i = 0; i < NUM_DEV_TOTAL; i++) {
		cdev_del(&(cdev_array[i]));
	}

	/* --- free device num. and remove device --- */
	/* /dev/rtled0,1,2,3 */
	devno_top = MKDEV(_major_led, _minor_led);
	for (i = 0; i < NUM_DEV_LED; i++) {
		devno = MKDEV(_major_led, _minor_led + i);
		device_destroy(class_led, devno);
	}
	unregister_chrdev_region(devno_top, NUM_DEV_LED);
	/* /dev/rtswitch0,1,2 */
	devno_top = MKDEV(_major_switch, _minor_switch);
	for (i = 0; i < NUM_DEV_SWITCH; i++) {
		devno = MKDEV(_major_switch, _minor_switch + i);
		device_destroy(class_switch, devno);
	}
	unregister_chrdev_region(devno_top, NUM_DEV_SWITCH);
	/* /dev/rtlightsensor0 */
	devno = MKDEV(_major_sensor, _minor_sensor);
	device_destroy(class_sensor, devno);
	unregister_chrdev_region(devno, NUM_DEV_SENSOR);
	/* /dev/rtbuzzer0 */
	devno = MKDEV(_major_buzzer, _minor_buzzer);
	device_destroy(class_buzzer, devno);
	unregister_chrdev_region(devno, NUM_DEV_BUZZER);
	/* /dev/rtmotor_raw_r0 */
	devno = MKDEV(_major_motorrawr, _minor_motorrawr);
	device_destroy(class_motorrawr, devno);
	unregister_chrdev_region(devno, NUM_DEV_MOTORRAWR);
	/* /dev/rtmotor_raw_l0 */
	devno = MKDEV(_major_motorrawl, _minor_motorrawl);
	device_destroy(class_motorrawl, devno);
	unregister_chrdev_region(devno, NUM_DEV_MOTORRAWL);
	/* /dev/rtmotoren0 */
	devno = MKDEV(_major_motoren, _minor_motoren);
	device_destroy(class_motoren, devno);
	unregister_chrdev_region(devno, NUM_DEV_MOTOREN);
	/* /dev/rtmotor0 */
	devno = MKDEV(_major_motor, _minor_motor);
	device_destroy(class_motor, devno);
	unregister_chrdev_region(devno, NUM_DEV_MOTOR);

	/* --- remove device node --- */
	class_destroy(class_led);
	class_destroy(class_switch);
	class_destroy(class_sensor);
	class_destroy(class_buzzer);
	class_destroy(class_motorrawr);
	class_destroy(class_motorrawl);
	class_destroy(class_motoren);
	class_destroy(class_motor);

	/* remove MCP3204 */
	mcp3204_exit();

	/* remove I2C device */
	i2c_counter_exit();

	/* free cdev memory */
	kfree(cdev_array);
	gpio_unmap();
	printk(KERN_INFO "%s: module removed at %lu\n", DRIVER_NAME, jiffies);
}

/* --- MAIN PROCESS --- */
module_init(dev_init_module);
module_exit(dev_cleanup_module);
