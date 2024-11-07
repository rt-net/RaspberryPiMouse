/*
 *
 * rtmouse_main.c
 * Raspberry Pi Mouse device driver main
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

/* --- Device Numbers --- */
static const unsigned int NUM_DEV[ID_DEV_SIZE] = {
    [ID_DEV_LED] = 4,	  [ID_DEV_SWITCH] = 3,	  [ID_DEV_SENSOR] = 1,
    [ID_DEV_BUZZER] = 1,  [ID_DEV_MOTORRAWR] = 1, [ID_DEV_MOTORRAWL] = 1,
    [ID_DEV_MOTOREN] = 1, [ID_DEV_MOTOR] = 1,	  [ID_DEV_CNT] = 2};

/* --- Device Names --- */
static const char *NAME_DEV[ID_DEV_SIZE] = {
    [ID_DEV_LED] = "rtled",
    [ID_DEV_SWITCH] = "rtswitch",
    [ID_DEV_SENSOR] = "rtlightsensor",
    [ID_DEV_BUZZER] = "rtbuzzer",
    [ID_DEV_MOTORRAWR] = "rtmotor_raw_r",
    [ID_DEV_MOTORRAWL] = "rtmotor_raw_l",
    [ID_DEV_MOTOREN] = "rtmotoren",
    [ID_DEV_MOTOR] = "rtmotor"};

static const char *NAME_DEV_U[ID_DEV_SIZE] = {
    [ID_DEV_LED] = "rtled%u",
    [ID_DEV_SWITCH] = "rtswitch%u",
    [ID_DEV_SENSOR] = "rtlightsensor%u",
    [ID_DEV_BUZZER] = "rtbuzzer%u",
    [ID_DEV_MOTORRAWR] = "rtmotor_raw_r%u",
    [ID_DEV_MOTORRAWL] = "rtmotor_raw_l%u",
    [ID_DEV_MOTOREN] = "rtmotoren%u",
    [ID_DEV_MOTOR] = "rtmotor%u"};

// used in by register_dev() and cleanup_each_dev()
static int _major_dev[ID_DEV_SIZE] = {
    [ID_DEV_LED] = DEV_MAJOR,	    [ID_DEV_SWITCH] = DEV_MAJOR,
    [ID_DEV_SENSOR] = DEV_MAJOR,    [ID_DEV_BUZZER] = DEV_MAJOR,
    [ID_DEV_MOTORRAWR] = DEV_MAJOR, [ID_DEV_MOTORRAWL] = DEV_MAJOR,
    [ID_DEV_MOTOREN] = DEV_MAJOR,   [ID_DEV_MOTOR] = DEV_MAJOR};

// used in register_dev() and cleanup_each_dev()
static int _minor_dev[ID_DEV_SIZE] = {
    [ID_DEV_LED] = DEV_MINOR,	    [ID_DEV_SWITCH] = DEV_MINOR,
    [ID_DEV_SENSOR] = DEV_MINOR,    [ID_DEV_BUZZER] = DEV_MINOR,
    [ID_DEV_MOTORRAWR] = DEV_MINOR, [ID_DEV_MOTORRAWL] = DEV_MINOR,
    [ID_DEV_MOTOREN] = DEV_MINOR,   [ID_DEV_MOTOR] = DEV_MINOR};

/* --- General Options --- */
static struct cdev *cdev_array = NULL;
static struct class *class_dev[ID_DEV_SIZE] = {
    [ID_DEV_LED] = NULL,       [ID_DEV_SWITCH] = NULL,
    [ID_DEV_SENSOR] = NULL,    [ID_DEV_BUZZER] = NULL,
    [ID_DEV_MOTORRAWR] = NULL, [ID_DEV_MOTORRAWL] = NULL,
    [ID_DEV_MOTOREN] = NULL,   [ID_DEV_MOTOR] = NULL};

static volatile void __iomem *clk_base;
static volatile int cdev_index = 0;

// used in rtmouse_dev_fops.c
volatile void __iomem *pwm_base;
volatile uint32_t *gpio_base;
struct mutex lock;

/* --- Function Declarations --- */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 2, 0)
static int rtcnt_i2c_probe(struct i2c_client *client,
			   const struct i2c_device_id *id);
#else
static int rtcnt_i2c_probe(struct i2c_client *client);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
static int rtcnt_i2c_remove(struct i2c_client *client);
#else
static void rtcnt_i2c_remove(struct i2c_client *client);
#endif

/* --- Static variables --- */
// used in rtmouse_dev_fops.c
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0)
struct device *mcp320x_dev;
#endif

static struct i2c_client *i2c_client_r = NULL;
static struct i2c_client *i2c_client_l = NULL;

/* I2C Device ID */
static struct i2c_device_id i2c_counter_id[] = {
    {DEVNAME_CNTL, 0},
    {DEVNAME_CNTR, 1},
    {},
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
MODULE_DEVICE_TABLE(i2c, i2c_counter_id);

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

	// printk(KERN_DEBUG "%s: rpi_pwm_ctrl:%08X\n", DRIVER_NAME,
	// ioread32(pwm_base + RPI_PWM_CTRL));

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

/* --- Device Driver Registration and Device File Creation --- */
static int register_dev(int id_dev)
{
	int retval;
	dev_t dev;
	dev_t devno;
	int i;

	/* 空いているメジャー番号を使ってメジャー&
	   マイナー番号をカーネルに登録する */
	retval =
	    alloc_chrdev_region(&dev, /* 結果を格納するdev_t構造体 */
				DEV_MINOR, /* ベースマイナー番号 */
				NUM_DEV[id_dev], /* デバイスの数 */
				NAME_DEV[id_dev] /* デバイスドライバの名前 */
	    );

	if (retval < 0) {
		printk(KERN_ERR "alloc_chrdev_region failed.\n");
		return retval;
	}
	_major_dev[id_dev] = MAJOR(dev);

	/* デバイスクラスを作成する */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
	class_dev[id_dev] = class_create(THIS_MODULE, NAME_DEV[id_dev]);
#else
	class_dev[id_dev] = class_create(NAME_DEV[id_dev]);
#endif

	if (IS_ERR(class_dev[id_dev])) {
		return PTR_ERR(class_dev[id_dev]);
	}

	for (i = 0; i < NUM_DEV[id_dev]; i++) {
		/* デバイスの数だけキャラクタデバイスを登録する */
		devno = MKDEV(_major_dev[id_dev], _minor_dev[id_dev] + i);

		/* キャラクタデバイスとしてこのモジュールをカーネルに登録する */
		cdev_init(&(cdev_array[cdev_index]), &dev_fops[id_dev]);
		cdev_array[cdev_index].owner = THIS_MODULE;
		if (cdev_add(&(cdev_array[cdev_index]), devno, 1) < 0) {
			/* 登録に失敗した */
			printk(KERN_ERR "cdev_add failed minor = %d\n",
			       _minor_dev[id_dev] + i);
		} else {
			/* デバイスノードの作成 */
			struct device *dev_ret;
			dev_ret = device_create(class_dev[id_dev], NULL, devno,
						NULL, NAME_DEV_U[id_dev],
						_minor_dev[id_dev] + i);

			/* デバイスファイル作成の可否を判定 */
			if (IS_ERR(dev_ret)) {
				/* デバイスファイルの作成に失敗した */
				printk(KERN_ERR
				       "device_create failed minor = %d\n",
				       _minor_dev[id_dev] + i);
				/* リソースリークを避けるために登録された状態cdevを削除する
				 */
				cdev_del(&(cdev_array[cdev_index]));
				return PTR_ERR(dev_ret);
			}
		}
		cdev_index++;
	}
	return 0;
}

static int rtcntr_i2c_create_cdev(struct rtcnt_device_info *dev_info)
{
	int minor;
	int alloc_ret = 0;
	int cdev_err = 0;
	dev_t dev;

	/* 空いているメジャー番号を確保する */
	alloc_ret = alloc_chrdev_region(&dev, DEV_MINOR, NUM_DEV[ID_DEV_CNT],
					DEVNAME_CNTR);
	if (alloc_ret != 0) {
		printk(KERN_ERR "alloc_chrdev_region = %d\n", alloc_ret);
		return -1;
	}

	/* 取得したdev( = メジャー番号 +  マイナー番号)
	 * からメジャー番号を取得して保持しておく */
	dev_info->device_major = MAJOR(dev);
	dev = MKDEV(dev_info->device_major, DEV_MINOR);

	/* cdev構造体の初期化とシステムコールハンドラテーブルの登録 */
	cdev_init(&dev_info->cdev, &dev_fops[ID_DEV_CNT]);
	dev_info->cdev.owner = THIS_MODULE;

	/* このデバイスドライバ(cdev)をカーネルに登録する */
	cdev_err = cdev_add(&dev_info->cdev, dev, NUM_DEV[ID_DEV_CNT]);
	if (cdev_err != 0) {
		printk(KERN_ERR "cdev_add = %d\n", alloc_ret);
		unregister_chrdev_region(dev, NUM_DEV[ID_DEV_CNT]);
		return -1;
	}

	/* このデバイスのクラス登録をする(/sys/class/mydevice/ を作る) */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
	dev_info->device_class = class_create(THIS_MODULE, DEVNAME_CNTR);
#else
	dev_info->device_class = class_create(DEVNAME_CNTR);
#endif

	if (IS_ERR(dev_info->device_class)) {
		printk(KERN_ERR "class_create\n");
		cdev_del(&dev_info->cdev);
		unregister_chrdev_region(dev, NUM_DEV[ID_DEV_CNT]);
		return -1;
	}

	for (minor = DEV_MINOR; minor < DEV_MINOR + NUM_DEV[ID_DEV_CNT];
	     minor++) {

		struct device *dev_ret;
		dev_ret = device_create(dev_info->device_class, NULL,
					MKDEV(dev_info->device_major, minor),
					NULL, "rtcounter_r%d", minor);

		/* デバイスファイル作成の可否を判定 */
		if (IS_ERR(dev_ret)) {
			/* デバイスファイルの作成に失敗した */
			printk(KERN_ERR "device_create failed minor = %d\n",
			       minor);
			/* リソースリークを避けるために登録された状態cdevを削除する
			 */
			cdev_del(&(cdev_array[cdev_index]));
			return PTR_ERR(dev_ret);
		}
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
	alloc_ret = alloc_chrdev_region(&dev, DEV_MINOR, NUM_DEV[ID_DEV_CNT],
					DEVNAME_CNTL);
	if (alloc_ret != 0) {
		printk(KERN_ERR "alloc_chrdev_region = %d\n", alloc_ret);
		return -1;
	}

	/* 取得したdev( = メジャー番号 + マイナー番号)
	 * からメジャー番号を取得して保持しておく */
	dev_info->device_major = MAJOR(dev);
	dev = MKDEV(dev_info->device_major, DEV_MINOR);

	/* cdev構造体の初期化とシステムコールハンドラテーブルの登録 */
	cdev_init(&dev_info->cdev, &dev_fops[ID_DEV_CNT]);
	dev_info->cdev.owner = THIS_MODULE;

	/* このデバイスドライバ(cdev)をカーネルに登録する */
	cdev_err = cdev_add(&dev_info->cdev, dev, NUM_DEV[ID_DEV_CNT]);
	if (cdev_err != 0) {
		printk(KERN_ERR "cdev_add = %d\n", alloc_ret);
		unregister_chrdev_region(dev, NUM_DEV[ID_DEV_CNT]);
		return -1;
	}

/* このデバイスのクラス登録をする(/sys/class/mydevice/ を作る) */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
	dev_info->device_class = class_create(THIS_MODULE, DEVNAME_CNTL);
#else
	dev_info->device_class = class_create(DEVNAME_CNTL);
#endif

	if (IS_ERR(dev_info->device_class)) {
		printk(KERN_ERR "class_create\n");
		cdev_del(&dev_info->cdev);
		unregister_chrdev_region(dev, NUM_DEV[ID_DEV_CNT]);
		return -1;
	}

	/* /sys/class/mydevice/mydevice* を作る */
	for (minor = DEV_MINOR; minor < DEV_MINOR + NUM_DEV[ID_DEV_CNT];
	     minor++) {

		struct device *dev_ret;
		dev_ret = device_create(dev_info->device_class, NULL,
					MKDEV(dev_info->device_major, minor),
					NULL, "rtcounter_l%d", minor);

		/* デバイスファイル作成の可否を判定 */
		if (IS_ERR(dev_ret)) {
			/* デバイスファイルの作成に失敗した */
			printk(KERN_ERR "device_create failed minor = %d\n",
			       minor);
			/* リソースリークを避けるために登録された状態cdevを削除する
			 */
			cdev_del(&(cdev_array[cdev_index]));
			return PTR_ERR(dev_ret);
		}
	}

	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 2, 0)
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
#else
static int rtcnt_i2c_probe(struct i2c_client *client)
{
	const struct i2c_device_id *id = i2c_client_get_device_id(client);
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
#endif

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
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 0, 0)
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
#else
	// printk(KERN_DEBUG "%s: adding i2c device", __func__);
	i2c_adap_l = i2c_get_adapter(1);
	i2c_client_l = i2c_new_client_device(i2c_adap_l, &i2c_board_info_l);
	i2c_put_adapter(i2c_adap_l);
	// printk(KERN_DEBUG "%s: added i2c device rtcntl", __func__);

	// printk(KERN_DEBUG "%s: adding i2c device", __func__);
	i2c_adap_r = i2c_get_adapter(1);
	i2c_client_r = i2c_new_client_device(i2c_adap_r, &i2c_board_info_r);
	i2c_put_adapter(i2c_adap_r);
	// printk(KERN_DEBUG "%s: added i2c device rtcntr", __func__);
#endif

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
	for (minor = DEV_MINOR; minor < DEV_MINOR + NUM_DEV[ID_DEV_CNT];
	     minor++) {
		device_destroy(dev_info->device_class,
			       MKDEV(dev_info->device_major, minor));
	}
	/* このデバイスのクラス登録を取り除く(/sys/class/mydevice/を削除する) */
	class_destroy(dev_info->device_class);
	/* このデバイスドライバ(cdev)をカーネルから取り除く */
	cdev_del(&dev_info->cdev);
	/* このデバイスドライバで使用していたメジャー番号の登録を取り除く */
	unregister_chrdev_region(dev, NUM_DEV[ID_DEV_CNT]);
}

/*
 * i2c_counter_remove - I2C pulse counter
 * called when I2C pulse counter removed
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
static int rtcnt_i2c_remove(struct i2c_client *client)
{
	struct rtcnt_device_info *dev_info;
	// printk(KERN_DEBUG "%s: removing i2c device 0x%x\n", __func__,
	// client->addr);
	dev_info = i2c_get_clientdata(client);
	rtcnt_i2c_delete_cdev(dev_info);
	printk(KERN_INFO "%s: i2c device 0x%x removed\n", DRIVER_NAME,
	       client->addr);
	return 0;
}
#else
static void rtcnt_i2c_remove(struct i2c_client *client)
{
	struct rtcnt_device_info *dev_info;
	// printk(KERN_DEBUG "%s: removing i2c device 0x%x\n", __func__,
	// client->addr);
	dev_info = i2c_get_clientdata(client);
	rtcnt_i2c_delete_cdev(dev_info);
	printk(KERN_INFO "%s: i2c device 0x%x removed\n", DRIVER_NAME,
	       client->addr);
}
#endif

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
