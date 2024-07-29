#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/version.h>


#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 1, 6)

/**
 * struct spi_statistics - statistics for spi transfers
 * @lock:          lock protecting this structure
 *
 * @messages:      number of spi-messages handled
 * @transfers:     number of spi_transfers handled
 * @errors:        number of errors during spi_transfer
 * @timedout:      number of timeouts during spi_transfer
 *
 * @spi_sync:      number of times spi_sync is used
 * @spi_sync_immediate:
 *                 number of times spi_sync is executed immediately
 *                 in calling context without queuing and scheduling
 * @spi_async:     number of times spi_async is used
 *
 * @bytes:         number of bytes transferred to/from device
 * @bytes_tx:      number of bytes sent to device
 * @bytes_rx:      number of bytes received from device
 *
 * @transfer_bytes_histo:
 *                 transfer bytes histogramm
 *
 * @transfers_split_maxsize:
 *                 number of transfers that have been split because of
 *                 maxsize limit
 */
struct spi_statistics {
	spin_t lock; /* lock for the whole structure */

	unsigned long messages;
	unsigned long transfers;
	unsigned long errors;
	unsigned long timedout;

	unsigned long spi_sync;
	unsigned long spi_sync_immediate;
	unsigned long spi_async;

	unsigned long long bytes;
	unsigned long long bytes_rx;
	unsigned long long bytes_tx;

#define SPI_STATISTICS_HISTO_SIZE 17
	unsigned long transfer_bytes_histo[SPI_STATISTICS_HISTO_SIZE];

	unsigned long transfers_split_maxsize;
};

static void spi_controller_release(struct device *dev);

#define SPI_STATISTICS_ATTRS(field, file)                                      \
	static ssize_t spi_controller_##field##_show(                          \
	    struct device *dev, struct device_attribute *attr, char *buf)      \
	{                                                                      \
		struct spi_controller *ctlr =                                  \
		    container_of(dev, struct spi_controller, dev);             \
		return spi_statistics_##field##_show(&ctlr->statistics, buf);  \
	}                                                                      \
	static struct device_attribute dev_attr_spi_controller_##field = {     \
	    .attr = {.name = file, .mode = 0444},                              \
	    .show = spi_controller_##field##_show,                             \
	};                                                                     \
	static ssize_t spi_device_##field##_show(                              \
	    struct device *dev, struct device_attribute *attr, char *buf)      \
	{                                                                      \
		struct spi_device *spi = to_spi_device(dev);                   \
		return spi_statistics_##field##_show(&spi->statistics, buf);   \
	}                                                                      \
	static struct device_attribute dev_attr_spi_device_##field = {         \
	    .attr = {.name = file, .mode = 0444},                              \
	    .show = spi_device_##field##_show,                                 \
	}

#define SPI_STATISTICS_SHOW_NAME(name, file, field, format_string)             \
	static ssize_t spi_statistics_##name##_show(                           \
	    struct spi_statistics *stat, char *buf)                            \
	{                                                                      \
		unsigned long flags;                                           \
		ssize_t len;                                                   \
		spin_lock_irqsave(&stat->lock, flags);                         \
		len = sprintf(buf, format_string, stat->field);                \
		spin_unlock_irqrestore(&stat->lock, flags);                    \
		return len;                                                    \
	}                                                                      \
	SPI_STATISTICS_ATTRS(name, file)

#define SPI_STATISTICS_SHOW(field, format_string)                              \
	SPI_STATISTICS_SHOW_NAME(field, __stringify(field), field,             \
				 format_string)

SPI_STATISTICS_SHOW(messages, "%lu");
SPI_STATISTICS_SHOW(transfers, "%lu");
SPI_STATISTICS_SHOW(errors, "%lu");
SPI_STATISTICS_SHOW(timedout, "%lu");

SPI_STATISTICS_SHOW(spi_sync, "%lu");
SPI_STATISTICS_SHOW(spi_sync_immediate, "%lu");
SPI_STATISTICS_SHOW(spi_async, "%lu");

SPI_STATISTICS_SHOW(bytes, "%llu");
SPI_STATISTICS_SHOW(bytes_rx, "%llu");
SPI_STATISTICS_SHOW(bytes_tx, "%llu");

#define SPI_STATISTICS_TRANSFER_BYTES_HISTO(index, number)                     \
	SPI_STATISTICS_SHOW_NAME(transfer_bytes_histo##index,                  \
				 "transfer_bytes_histo_" number,               \
				 transfer_bytes_histo[index], "%lu")
SPI_STATISTICS_TRANSFER_BYTES_HISTO(0, "0-1");
SPI_STATISTICS_TRANSFER_BYTES_HISTO(1, "2-3");
SPI_STATISTICS_TRANSFER_BYTES_HISTO(2, "4-7");
SPI_STATISTICS_TRANSFER_BYTES_HISTO(3, "8-15");
SPI_STATISTICS_TRANSFER_BYTES_HISTO(4, "16-31");
SPI_STATISTICS_TRANSFER_BYTES_HISTO(5, "32-63");
SPI_STATISTICS_TRANSFER_BYTES_HISTO(6, "64-127");
SPI_STATISTICS_TRANSFER_BYTES_HISTO(7, "128-255");
SPI_STATISTICS_TRANSFER_BYTES_HISTO(8, "256-511");
SPI_STATISTICS_TRANSFER_BYTES_HISTO(9, "512-1023");
SPI_STATISTICS_TRANSFER_BYTES_HISTO(10, "1024-2047");
SPI_STATISTICS_TRANSFER_BYTES_HISTO(11, "2048-4095");
SPI_STATISTICS_TRANSFER_BYTES_HISTO(12, "4096-8191");
SPI_STATISTICS_TRANSFER_BYTES_HISTO(13, "8192-16383");
SPI_STATISTICS_TRANSFER_BYTES_HISTO(14, "16384-32767");
SPI_STATISTICS_TRANSFER_BYTES_HISTO(15, "32768-65535");
SPI_STATISTICS_TRANSFER_BYTES_HISTO(16, "65536+");

SPI_STATISTICS_SHOW(transfers_split_maxsize, "%lu");

static struct attribute *spi_controller_statistics_attrs[] = {
    &dev_attr_spi_controller_messages.attr,
    &dev_attr_spi_controller_transfers.attr,
    &dev_attr_spi_controller_errors.attr,
    &dev_attr_spi_controller_timedout.attr,
    &dev_attr_spi_controller_spi_sync.attr,
    &dev_attr_spi_controller_spi_sync_immediate.attr,
    &dev_attr_spi_controller_spi_async.attr,
    &dev_attr_spi_controller_bytes.attr,
    &dev_attr_spi_controller_bytes_rx.attr,
    &dev_attr_spi_controller_bytes_tx.attr,
    &dev_attr_spi_controller_transfer_bytes_histo0.attr,
    &dev_attr_spi_controller_transfer_bytes_histo1.attr,
    &dev_attr_spi_controller_transfer_bytes_histo2.attr,
    &dev_attr_spi_controller_transfer_bytes_histo3.attr,
    &dev_attr_spi_controller_transfer_bytes_histo4.attr,
    &dev_attr_spi_controller_transfer_bytes_histo5.attr,
    &dev_attr_spi_controller_transfer_bytes_histo6.attr,
    &dev_attr_spi_controller_transfer_bytes_histo7.attr,
    &dev_attr_spi_controller_transfer_bytes_histo8.attr,
    &dev_attr_spi_controller_transfer_bytes_histo9.attr,
    &dev_attr_spi_controller_transfer_bytes_histo10.attr,
    &dev_attr_spi_controller_transfer_bytes_histo11.attr,
    &dev_attr_spi_controller_transfer_bytes_histo12.attr,
    &dev_attr_spi_controller_transfer_bytes_histo13.attr,
    &dev_attr_spi_controller_transfer_bytes_histo14.attr,
    &dev_attr_spi_controller_transfer_bytes_histo15.attr,
    &dev_attr_spi_controller_transfer_bytes_histo16.attr,
    &dev_attr_spi_controller_transfers_split_maxsize.attr,
    NULL,
};

static const struct attribute_group spi_controller_statistics_group = {
    .name = "statistics",
    .attrs = spi_controller_statistics_attrs,
};

static const struct attribute_group *spi_master_groups[] = {
    &spi_controller_statistics_group,
    NULL,
};

static struct class spi_master_class = {
    .name = "spi_master",
    .dev_release = spi_controller_release,
    .dev_groups = spi_master_groups,
};

struct spi_controller *spi_busnum_to_master_alt(u16 bus_num)
{
	struct device *dev;
	struct spi_controller *ctlr = NULL;

	// dev = class_find_device(&spi_master_class, NULL, &bus_num,
	// 			__spi_controller_match);
	// if (dev)
	// 	ctlr = container_of(dev, struct spi_controller, dev);
	// /* reference got in class_find_device */
	return ctlr;
}

static void spi_controller_release(struct device *dev)
{
	struct spi_controller *ctlr;

	ctlr = container_of(dev, struct spi_controller, dev);
	kfree(ctlr);
}

#endif
