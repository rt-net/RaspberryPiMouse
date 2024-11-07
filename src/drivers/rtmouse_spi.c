#include "rtmouse.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0)
static int mcp3204_remove(struct spi_device *spi);
#else
static void mcp3204_remove(struct spi_device *spi);
#endif
static int mcp3204_probe(struct spi_device *spi);

/*
 * SPI device ID
 * used in mcp3204_driver
 */
static struct spi_device_id mcp3204_id[] = {
    {"mcp3204", 0},
    {},
};

/*
 * SPI Info
 * used in mcp3204_probe(), mcp3204_init(), mcp3204_exit()
 * and mcp3204_get_value()
 */
struct spi_board_info mcp3204_info = {
    .modalias = "mcp3204",
    .max_speed_hz = 100000,
    .bus_num = 0,
    .chip_select = 0,
    .mode = SPI_MODE_3,
};

/*
 * SPI Dirver Info
 * used in mcp3204_init() and mcp3204_exit()
 */
static struct spi_driver mcp3204_driver = {
    .driver =
	{
	    .name = DEVNAME_SENSOR,
	    .owner = THIS_MODULE,
	},
    .id_table = mcp3204_id,
    .probe = mcp3204_probe,
    .remove = mcp3204_remove,
};

/* -- Device Addition -- */
MODULE_DEVICE_TABLE(spi, mcp3204_id);

/*
 * mcp3204_remove - remove function lined with spi_dirver
 * used in mcp3204_driver and mcp3204_exit()
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0)
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
#else
static void mcp3204_remove(struct spi_device *spi)
{
	struct mcp3204_drvdata *data;
	/* get drvdata */
	data = (struct mcp3204_drvdata *)spi_get_drvdata(spi);
	/* free kernel memory */
	kfree(data);
	printk(KERN_INFO "%s: mcp3204 removed\n", DRIVER_NAME);
}
#endif

/*
 * mcp3204_probe - probe function lined with spi_dirver
 * used in mcp3204_driver and __callback_find_mcp3204()
 */
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
	data->xfer.speed_hz = 100000;

	spi_message_init_with_transfers(&data->msg, &data->xfer, 1);

	/* set drvdata */
	spi_set_drvdata(spi, data);

	printk(KERN_INFO "%s: mcp3204 probed", DRIVER_NAME);

	return 0;
}

/*
 * spi_remove_device - remove SPI device
 * called by mcp3204_init() and mcp3204_exit()
 */
static void spi_remove_device(struct spi_master *master, unsigned int cs)
{
	struct device *dev;
	char str[128];

	snprintf(str, sizeof(str), "%s.%u", dev_name(&master->dev), cs);

	dev = bus_find_device_by_name(&spi_bus_type, NULL, str);
	// ここを参考にspi_deviceを取得するプログラムを作成する
	if (dev) {
		device_del(dev);
	}
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0)
/*
 * spiをサーチする関数
 * used in mcp3204_init()
 */
static int __callback_find_mcp3204(struct device *dev, void *data)
{
	printk(KERN_INFO "    device_name: %s\n", dev->driver->name);
	if (mcp320x_dev == NULL && strcmp(dev->driver->name, "mcp320x") == 0) {
		mcp320x_dev = dev;
		mcp3204_probe(to_spi_device(dev));
	}
	return 0;
}
#endif

/*
 * mcp3204_init - initialize MCP3204
 * called by dev_init_module()
 */
int mcp3204_init(void)
{

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0)
	bus_for_each_dev(&spi_bus_type, NULL, NULL, __callback_find_mcp3204);
#else
	struct spi_master *master;
	struct spi_device *spi_device;

	spi_register_driver(&mcp3204_driver);

	mcp3204_info.bus_num = SPI_BUS_NUM;
	mcp3204_info.chip_select = SPI_CHIP_SELECT;

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
#endif

	return 0;
}

/*
 * mcp3204_exit - cleanup MCP3204
 * called by dev_cleanup_module()
 */
void mcp3204_exit(void)
{

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0)
	printk(KERN_INFO "   mcp3204_exit\n");
	if (mcp320x_dev) {
		mcp3204_remove(to_spi_device(mcp320x_dev));
	}
#else
	struct spi_master *master;
	master = spi_busnum_to_master(mcp3204_info.bus_num);

	if (master) {
		spi_remove_device(master, mcp3204_info.chip_select);
	} else {
		printk(KERN_ERR "mcp3204 remove error\n");
	}

	spi_unregister_driver(&mcp3204_driver);
#endif
}
