
#include <linux/spi/spi.h>



struct spi_controller *spi_busnum_to_master(int bus_num)
{
	// struct device		*dev;
	struct spi_controller	*ctlr = NULL;

	// dev = class_find_device(&spi_master_class, NULL, &bus_num,
	// 			__spi_controller_match);
	// if (dev)
	// 	ctlr = container_of(dev, struct spi_controller, dev);
	// /* reference got in class_find_device */
	return ctlr;
}
