/*
 *
 * rtmouse_main.c
 * Raspberry Pi Mouse device driver
 *
 * Version: 3.3.3
 *
 * Copyright (C) 2015-2024 RT Corporation <shop@rt-net.jp>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#include "rtmouse.h"

MODULE_AUTHOR("RT Corporation");
MODULE_LICENSE("GPL");
MODULE_VERSION("3.3.3");
MODULE_DESCRIPTION("Raspberry Pi Mouse device driver");

/*
 * --- Device Numbers ---
 * used in rtmouse_i2c.c
 */
const unsigned int NUM_DEV[ID_DEV_SIZE] = {
    [ID_DEV_LED] = 4,	  [ID_DEV_SWITCH] = 3,	  [ID_DEV_SENSOR] = 1,
    [ID_DEV_BUZZER] = 1,  [ID_DEV_MOTORRAWR] = 1, [ID_DEV_MOTORRAWL] = 1,
    [ID_DEV_MOTOREN] = 1, [ID_DEV_MOTOR] = 1,	  [ID_DEV_CNT] = 2};

/*
 * --- Device Names ---
 * used in rtmouse_dev.c
 */
const char *NAME_DEV[ID_DEV_SIZE] = {[ID_DEV_LED] = "rtled",
				     [ID_DEV_SWITCH] = "rtswitch",
				     [ID_DEV_SENSOR] = "rtlightsensor",
				     [ID_DEV_BUZZER] = "rtbuzzer",
				     [ID_DEV_MOTORRAWR] = "rtmotor_raw_r",
				     [ID_DEV_MOTORRAWL] = "rtmotor_raw_l",
				     [ID_DEV_MOTOREN] = "rtmotoren",
				     [ID_DEV_MOTOR] = "rtmotor"};

/*
 * --- Device Names(+%u) ---
 * used in rtmouse_dev.c
 */
const char *NAME_DEV_U[ID_DEV_SIZE] = {[ID_DEV_LED] = "rtled%u",
				       [ID_DEV_SWITCH] = "rtswitch%u",
				       [ID_DEV_SENSOR] = "rtlightsensor%u",
				       [ID_DEV_BUZZER] = "rtbuzzer%u",
				       [ID_DEV_MOTORRAWR] = "rtmotor_raw_r%u",
				       [ID_DEV_MOTORRAWL] = "rtmotor_raw_l%u",
				       [ID_DEV_MOTOREN] = "rtmotoren%u",
				       [ID_DEV_MOTOR] = "rtmotor%u"};

// used in by rtmouse_dev.c and cleanup_each_dev()
int _major_dev[ID_DEV_SIZE] = {
    [ID_DEV_LED] = DEV_MAJOR,	    [ID_DEV_SWITCH] = DEV_MAJOR,
    [ID_DEV_SENSOR] = DEV_MAJOR,    [ID_DEV_BUZZER] = DEV_MAJOR,
    [ID_DEV_MOTORRAWR] = DEV_MAJOR, [ID_DEV_MOTORRAWL] = DEV_MAJOR,
    [ID_DEV_MOTOREN] = DEV_MAJOR,   [ID_DEV_MOTOR] = DEV_MAJOR};

// used in rtmouse_dev.c and cleanup_each_dev()
int _minor_dev[ID_DEV_SIZE] = {
    [ID_DEV_LED] = DEV_MINOR,	    [ID_DEV_SWITCH] = DEV_MINOR,
    [ID_DEV_SENSOR] = DEV_MINOR,    [ID_DEV_BUZZER] = DEV_MINOR,
    [ID_DEV_MOTORRAWR] = DEV_MINOR, [ID_DEV_MOTORRAWL] = DEV_MINOR,
    [ID_DEV_MOTOREN] = DEV_MINOR,   [ID_DEV_MOTOR] = DEV_MINOR};

/*
 * --- General Options ---
 * used in rtmouse_dev.c
 */
struct class *class_dev[ID_DEV_SIZE] = {
    [ID_DEV_LED] = NULL,       [ID_DEV_SWITCH] = NULL,
    [ID_DEV_SENSOR] = NULL,    [ID_DEV_BUZZER] = NULL,
    [ID_DEV_MOTORRAWR] = NULL, [ID_DEV_MOTORRAWL] = NULL,
    [ID_DEV_MOTOREN] = NULL,   [ID_DEV_MOTOR] = NULL};

static volatile void __iomem *clk_base;

// used in rtmouse_i2c.c
struct cdev *cdev_array = NULL;
volatile int cdev_index = 0;

// used in rtmouse_dev_fops.c
volatile void __iomem *pwm_base;
volatile uint32_t *gpio_base;
struct mutex lock;

/* --- Static variables --- */
// used in rtmouse_dev_fops.c
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0)
struct device *mcp320x_dev;
#endif

/*
 * set function
 * called by buzzer_init(), set_motor_l_freq(), set_motor_r_freq() and
 * buzzer_write()
 */
int rpi_gpio_function_set(int pin, uint32_t func)
{
	int index = RPI_GPFSEL0_INDEX + pin / 10;
	uint32_t mask = ~(0x7 << ((pin % 10) * 3));

	gpio_base[index] =
	    (gpio_base[index] & mask) | ((func & 0x7) << ((pin % 10) * 3));

	return 1;
}

/*
 * set mask and value
 * called by sensor_read(), set_motor_l_freq(), set_motor_r_freq(), led_put()
 * and motoren_write()
 */
void rpi_gpio_set32(uint32_t mask, uint32_t val)
{
	gpio_base[RPI_GPSET0_INDEX] = val & mask;
}

/*
 * clear mask and value
 * called by sensor_read(), set_motor_l_freq(), set_motor_r_freq(), led_del()
 * and motoren_write()
 */
void rpi_gpio_clear32(uint32_t mask, uint32_t val)
{
	gpio_base[RPI_GPCLR0_INDEX] = val & mask;
}

/*
 * pwm set function
 * called by buzzer_init(), set_motor_l_freq(), set_motor_r_freq()
 * and buzzer_write()
 */
void rpi_pwm_write32(uint32_t offset, uint32_t val)
{
	iowrite32(val, pwm_base + offset);
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

#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 0, 0)
	if (gpio_base == NULL) {
		gpio_base = ioremap_nocache(RPI_GPIO_BASE, RPI_GPIO_SIZE);
	}

	if (pwm_base == NULL) {
		pwm_base = ioremap_nocache(RPI_PWM_BASE, RPI_PWM_SIZE);
	}

	if (clk_base == NULL) {
		clk_base = ioremap_nocache(RPI_CLK_BASE, RPI_CLK_SIZE);
	}
#else
	if (gpio_base == NULL) {
		gpio_base = ioremap(RPI_GPIO_BASE, RPI_GPIO_SIZE);
	}

	if (pwm_base == NULL) {
		pwm_base = ioremap(RPI_PWM_BASE, RPI_PWM_SIZE);
	}

	if (clk_base == NULL) {
		clk_base = ioremap(RPI_CLK_BASE, RPI_CLK_SIZE);
	}
#endif

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

/*
 * dev_init_module - register driver module
 * called by module_init(dev_init_module)
 */
static int dev_init_module(void)
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
		registered_devices += 2 * NUM_DEV[ID_DEV_CNT];
	} else {
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
	for (i = 0; i < ID_DEV_SIZE - 1; i++) {
		retval = register_dev(i);
		if (retval != 0) {
			printk(KERN_ALERT "%s: %s register failed.\n",
			       DRIVER_NAME, NAME_DEV[i]);
			return retval;
		}
	}

	retval = mcp3204_init();
	if (retval != 0) {
		printk(KERN_ALERT
		       "%s: optical sensor driver register failed.\n",
		       DRIVER_NAME);
		return retval;
	}

	printk(KERN_INFO "%s: %d devices loaded.\n", DRIVER_NAME,
	       registered_devices + NUM_DEV_TOTAL);

	printk(KERN_INFO "%s: module installed at %lu\n", DRIVER_NAME, jiffies);
	return 0;
}

/*
 * dev_cleanup_module - cleanup driver module
 * called by module_exit(dev_cleanup_module)
 */
static void cleanup_each_dev(int id_dev)
{
	int i;
	dev_t devno;
	dev_t devno_top;

	devno_top = MKDEV(_major_dev[id_dev], _minor_dev[id_dev]);
	for (i = 0; i < NUM_DEV[id_dev]; i++) {
		devno = MKDEV(_major_dev[id_dev], _minor_dev[id_dev] + i);
		device_destroy(class_dev[id_dev], devno);
	}
	unregister_chrdev_region(devno_top, NUM_DEV[id_dev]);
}

static void dev_cleanup_module(void)
{
	int i;

	/* --- remove char device --- */
	for (i = 0; i < NUM_DEV_TOTAL; i++) {
		cdev_del(&(cdev_array[i]));
	}

	/* --- free device num. and remove device --- */
	for (i = 0; i < ID_DEV_SIZE - 1; i++) {
		cleanup_each_dev(i);
	}

	/* --- remove device node --- */
	for (i = 0; i < ID_DEV_SIZE - 1; i++) {
		class_destroy(class_dev[i]);
	}

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
