/*
 *
 * rtmouse_i2c.c
 * I2C driver
 *
 * Copyright (C) 2024 RT Corporation <shop@rt-net.jp>
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

// used in i2c_counter_init() and i2c_counter_exit()
static struct i2c_client *i2c_client_r = NULL;
static struct i2c_client *i2c_client_l = NULL;

/*
 * I2C Device ID
 * used in i2c_counter_driver
 */
static struct i2c_device_id i2c_counter_id[] = {
    {DEVNAME_CNTL, 0},
    {DEVNAME_CNTR, 1},
    {},
};

/*
 * I2C Dirver Info
 * used in i2c_counter_init() and i2c_counter_exit()
 */
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

// called by rtcnt_i2c_probe()
static int rtcnt_i2c_create_cdev(struct rtcnt_device_info *dev_info,
				 const int dev_side)
{
	int minor;
	int alloc_ret = 0;
	int cdev_err = 0;
	dev_t dev;

	/* 空いているメジャー番号を確保する */
	if (dev_side == DEV_LEFT) {
		alloc_ret = alloc_chrdev_region(
		    &dev, DEV_MINOR, NUM_DEV[ID_DEV_CNT], DEVNAME_CNTL);
	} else if (dev_side == DEV_RIGHT) {
		alloc_ret = alloc_chrdev_region(
		    &dev, DEV_MINOR, NUM_DEV[ID_DEV_CNT], DEVNAME_CNTR);
	}
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
	if (dev_side == DEV_LEFT) {
		dev_info->device_class =
		    class_create(THIS_MODULE, DEVNAME_CNTL);
	} else if (dev_side == DEV_RIGHT) {
		dev_info->device_class =
		    class_create(THIS_MODULE, DEVNAME_CNTR);
	}
#else
	if (dev_side == DEV_LEFT) {
		dev_info->device_class = class_create(DEVNAME_CNTL);
	} else if (dev_side == DEV_RIGHT) {
		dev_info->device_class = class_create(DEVNAME_CNTR);
	}
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
		if (dev_side == DEV_LEFT) {
			dev_ret =
			    device_create(dev_info->device_class, NULL,
					  MKDEV(dev_info->device_major, minor),
					  NULL, "rtcounter_l%d", minor);
		} else if (dev_side == DEV_RIGHT) {
			dev_ret =
			    device_create(dev_info->device_class, NULL,
					  MKDEV(dev_info->device_major, minor),
					  NULL, "rtcounter_r%d", minor);
		}

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

// called by rtcnt_i2c_remove()
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

// called by i2c_counter_driver()
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
		if (rtcnt_i2c_create_cdev(dev_info, DEVNAME_CNTL))
			return -ENOMEM;
	} else if ((int)(id->driver_data) == 1) {
		if (rtcnt_i2c_create_cdev(dev_info, DEVNAME_CNTR))
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
		if (rtcnt_i2c_create_cdev(dev_info, DEVNAME_CNTL))
			return -ENOMEM;
	} else if ((int)(id->driver_data) == 1) {
		if (rtcnt_i2c_create_cdev(dev_info, DEVNAME_CNTR))
			return -ENOMEM;
	}

	return 0;
}
#endif

/*
 * i2c_counter_remove - I2C pulse counter
 * called when I2C pulse counter removed
 * called by i2c_counter_driver()
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
 * i2c_counter_init - initialize I2C counter
 * called by dev_init_module()
 */
int i2c_counter_init(void)
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
void i2c_counter_exit(void)
{
	/* delete I2C driver */
	i2c_del_driver(&i2c_counter_driver);
	/* free memory */
	if (i2c_client_r)
		i2c_unregister_device(i2c_client_r);
	if (i2c_client_l)
		i2c_unregister_device(i2c_client_l);
}
