#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0)

#undef spi_master  // <linux/spi/spi.h>の"spi_controller"の定義を解く
#define spi_master spi_controller_alt  // 代替構造体と紐付け

/**
 * struct spi_statistics_alt - statistics for spi transfers
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
struct spi_statistics_alt {
	spinlock_t		lock; /* lock for the whole structure */

	unsigned long		messages;
	unsigned long		transfers;
	unsigned long		errors;
	unsigned long		timedout;

	unsigned long		spi_sync;
	unsigned long		spi_sync_immediate;
	unsigned long		spi_async;

	unsigned long long	bytes;
	unsigned long long	bytes_rx;
	unsigned long long	bytes_tx;

#define SPI_STATISTICS_HISTO_SIZE 17
	unsigned long transfer_bytes_histo[SPI_STATISTICS_HISTO_SIZE];

	unsigned long transfers_split_maxsize;
};



/**
 * struct spi_controller_alt - interface to SPI master or slave controller
 * @dev: device interface to this driver
 * @list: link with the global spi_controller_alt list
 * @bus_num: board-specific (and often SOC-specific) identifier for a
 *	given SPI controller.
 * @num_chipselect: chipselects are used to distinguish individual
 *	SPI slaves, and are numbered from zero to num_chipselects.
 *	each slave has a chipselect signal, but it's common that not
 *	every chipselect is connected to a slave.
 * @dma_alignment: SPI controller constraint on DMA buffers alignment.
 * @mode_bits: flags understood by this controller driver
 * @buswidth_override_bits: flags to override for this controller driver
 * @bits_per_word_mask: A mask indicating which values of bits_per_word are
 *	supported by the driver. Bit n indicates that a bits_per_word n+1 is
 *	supported. If set, the SPI core will reject any transfer with an
 *	unsupported bits_per_word. If not set, this value is simply ignored,
 *	and it's up to the individual driver to perform any validation.
 * @min_speed_hz: Lowest supported transfer speed
 * @max_speed_hz: Highest supported transfer speed
 * @flags: other constraints relevant to this driver
 * @slave: indicates that this is an SPI slave controller
 * @devm_allocated: whether the allocation of this struct is devres-managed
 * @max_transfer_size: function that returns the max transfer size for
 *	a &spi_device; may be %NULL, so the default %SIZE_MAX will be used.
 * @max_message_size: function that returns the max message size for
 *	a &spi_device; may be %NULL, so the default %SIZE_MAX will be used.
 * @io_mutex: mutex for physical bus access
 * @bus_lock_spinlock: spinlock for SPI bus locking
 * @bus_lock_mutex: mutex for exclusion of multiple callers
 * @bus_lock_flag: indicates that the SPI bus is locked for exclusive use
 * @setup: updates the device mode and clocking records used by a
 *	device's SPI controller; protocol code may call this.  This
 *	must fail if an unrecognized or unsupported mode is requested.
 *	It's always safe to call this unless transfers are pending on
 *	the device whose settings are being modified.
 * @set_cs_timing: optional hook for SPI devices to request SPI master
 * controller for configuring specific CS setup time, hold time and inactive
 * delay interms of clock counts
 * @transfer: adds a message to the controller's transfer queue.
 * @cleanup: frees controller-specific state
 * @can_dma: determine whether this controller supports DMA
 * @queued: whether this controller is providing an internal message queue
 * @kworker: pointer to thread struct for message pump
 * @pump_messages: work struct for scheduling work to the message pump
 * @queue_lock: spinlock to syncronise access to message queue
 * @queue: message queue
 * @idling: the device is entering idle state
 * @cur_msg: the currently in-flight message
 * @cur_msg_prepared: spi_prepare_message was called for the currently
 *                    in-flight message
 * @cur_msg_mapped: message has been mapped for DMA
 * @last_cs_enable: was enable true on the last call to set_cs.
 * @last_cs_mode_high: was (mode & SPI_CS_HIGH) true on the last call to set_cs.
 * @xfer_completion: used by core transfer_one_message()
 * @busy: message pump is busy
 * @running: message pump is running
 * @rt: whether this queue is set to run as a realtime task
 * @auto_runtime_pm: the core should ensure a runtime PM reference is held
 *                   while the hardware is prepared, using the parent
 *                   device for the spidev
 * @max_dma_len: Maximum length of a DMA transfer for the device.
 * @prepare_transfer_hardware: a message will soon arrive from the queue
 *	so the subsystem requests the driver to prepare the transfer hardware
 *	by issuing this call
 * @transfer_one_message: the subsystem calls the driver to transfer a single
 *	message while queuing transfers that arrive in the meantime. When the
 *	driver is finished with this message, it must call
 *	spi_finalize_current_message() so the subsystem can issue the next
 *	message
 * @unprepare_transfer_hardware: there are currently no more messages on the
 *	queue so the subsystem notifies the driver that it may relax the
 *	hardware by issuing this call
 *
 * @set_cs: set the logic level of the chip select line.  May be called
 *          from interrupt context.
 * @prepare_message: set up the controller to transfer a single message,
 *                   for example doing DMA mapping.  Called from threaded
 *                   context.
 * @transfer_one: transfer a single spi_transfer.
 *
 *                  - return 0 if the transfer is finished,
 *                  - return 1 if the transfer is still in progress. When
 *                    the driver is finished with this transfer it must
 *                    call spi_finalize_current_transfer() so the subsystem
 *                    can issue the next transfer. Note: transfer_one and
 *                    transfer_one_message are mutually exclusive; when both
 *                    are set, the generic subsystem does not call your
 *                    transfer_one callback.
 * @handle_err: the subsystem calls the driver to handle an error that occurs
 *		in the generic implementation of transfer_one_message().
 * @mem_ops: optimized/dedicated operations for interactions with SPI memory.
 *	     This field is optional and should only be implemented if the
 *	     controller has native support for memory like operations.
 * @unprepare_message: undo any work done by prepare_message().
 * @slave_abort: abort the ongoing transfer request on an SPI slave controller
 * @cs_gpios: LEGACY: array of GPIO descs to use as chip select lines; one per
 *	CS number. Any individual value may be -ENOENT for CS lines that
 *	are not GPIOs (driven by the SPI controller itself). Use the cs_gpiods
 *	in new drivers.
 * @cs_gpiods: Array of GPIO descs to use as chip select lines; one per CS
 *	number. Any individual value may be NULL for CS lines that
 *	are not GPIOs (driven by the SPI controller itself).
 * @use_gpio_descriptors: Turns on the code in the SPI core to parse and grab
 *	GPIO descriptors rather than using global GPIO numbers grabbed by the
 *	driver. This will fill in @cs_gpiods and @cs_gpios should not be used,
 *	and SPI devices will have the cs_gpiod assigned rather than cs_gpio.
 * @unused_native_cs: When cs_gpiods is used, spi_register_controller() will
 *	fill in this field with the first unused native CS, to be used by SPI
 *	controller drivers that need to drive a native CS when using GPIO CS.
 * @max_native_cs: When cs_gpiods is used, and this field is filled in,
 *	spi_register_controller() will validate all native CS (including the
 *	unused native CS) against this value.
 * @statistics: statistics for the spi_controller_alt
 * @dma_tx: DMA transmit channel
 * @dma_rx: DMA receive channel
 * @dummy_rx: dummy receive buffer for full-duplex devices
 * @dummy_tx: dummy transmit buffer for full-duplex devices
 * @fw_translate_cs: If the boot firmware uses different numbering scheme
 *	what Linux expects, this optional hook can be used to translate
 *	between the two.
 * @ptp_sts_supported: If the driver sets this to true, it must provide a
 *	time snapshot in @spi_transfer->ptp_sts as close as possible to the
 *	moment in time when @spi_transfer->ptp_sts_word_pre and
 *	@spi_transfer->ptp_sts_word_post were transmitted.
 *	If the driver does not set this, the SPI core takes the snapshot as
 *	close to the driver hand-over as possible.
 * @irq_flags: Interrupt enable state during PTP system timestamping
 * @fallback: fallback to pio if dma transfer return failure with
 *	SPI_TRANS_FAIL_NO_START.
 *
 * Each SPI controller can communicate with one or more @spi_device
 * children.  These make a small bus, sharing MOSI, MISO and SCK signals
 * but not chip select signals.  Each device may be configured to use a
 * different clock rate, since those shared signals are ignored unless
 * the chip is selected.
 *
 * The driver for an SPI controller manages access to those devices through
 * a queue of spi_message transactions, copying data between CPU memory and
 * an SPI slave device.  For each such message it queues, it calls the
 * message's completion function when the transaction completes.
 */
struct spi_controller_alt {
	struct device	dev;

	struct list_head list;

	/* other than negative (== assign one dynamically), bus_num is fully
	 * board-specific.  usually that simplifies to being SOC-specific.
	 * example:  one SOC has three SPI controllers, numbered 0..2,
	 * and one board's schematics might show it using SPI-2.  software
	 * would normally use bus_num=2 for that controller.
	 */
	s16			bus_num;

	/* chipselects will be integral to many controllers; some others
	 * might use board-specific GPIOs.
	 */
	u16			num_chipselect;

	/* some SPI controllers pose alignment requirements on DMAable
	 * buffers; let protocol drivers know about these requirements.
	 */
	u16			dma_alignment;

	/* spi_device.mode flags understood by this controller driver */
	u32			mode_bits;

	/* spi_device.mode flags override flags for this controller */
	u32			buswidth_override_bits;

	/* bitmask of supported bits_per_word for transfers */
	u32			bits_per_word_mask;
#define SPI_BPW_MASK(bits) BIT((bits) - 1)
#define SPI_BPW_RANGE_MASK(min, max) GENMASK((max) - 1, (min) - 1)

	/* limits on transfer speed */
	u32			min_speed_hz;
	u32			max_speed_hz;

	/* other constraints relevant to this driver */
	u16			flags;
#define SPI_CONTROLLER_HALF_DUPLEX	BIT(0)	/* can't do full duplex */
#define SPI_CONTROLLER_NO_RX		BIT(1)	/* can't do buffer read */
#define SPI_CONTROLLER_NO_TX		BIT(2)	/* can't do buffer write */
#define SPI_CONTROLLER_MUST_RX		BIT(3)	/* requires rx */
#define SPI_CONTROLLER_MUST_TX		BIT(4)	/* requires tx */

#define SPI_MASTER_GPIO_SS		BIT(5)	/* GPIO CS must select slave */

	/* flag indicating if the allocation of this struct is devres-managed */
	bool			devm_allocated;

	/* flag indicating this is an SPI slave controller */
	bool			slave;

	/*
	 * on some hardware transfer / message size may be constrained
	 * the limit may depend on device transfer settings
	 */
	size_t (*max_transfer_size)(struct spi_device *spi);
	size_t (*max_message_size)(struct spi_device *spi);

	/* I/O mutex */
	struct mutex		io_mutex;

	/* Used to avoid adding the same CS twice */
	struct mutex		add_lock;

	/* lock and mutex for SPI bus locking */
	spinlock_t		bus_lock_spinlock;
	struct mutex		bus_lock_mutex;

	/* flag indicating that the SPI bus is locked for exclusive use */
	bool			bus_lock_flag;

	/* Setup mode and clock, etc (spi driver may call many times).
	 *
	 * IMPORTANT:  this may be called when transfers to another
	 * device are active.  DO NOT UPDATE SHARED REGISTERS in ways
	 * which could break those transfers.
	 */
	int			(*setup)(struct spi_device *spi);

	/*
	 * set_cs_timing() method is for SPI controllers that supports
	 * configuring CS timing.
	 *
	 * This hook allows SPI client drivers to request SPI controllers
	 * to configure specific CS timing through spi_set_cs_timing() after
	 * spi_setup().
	 */
	int (*set_cs_timing)(struct spi_device *spi);

	/* bidirectional bulk transfers
	 *
	 * + The transfer() method may not sleep; its main role is
	 *   just to add the message to the queue.
	 * + For now there's no remove-from-queue operation, or
	 *   any other request management
	 * + To a given spi_device, message queueing is pure fifo
	 *
	 * + The controller's main job is to process its message queue,
	 *   selecting a chip (for masters), then transferring data
	 * + If there are multiple spi_device children, the i/o queue
	 *   arbitration algorithm is unspecified (round robin, fifo,
	 *   priority, reservations, preemption, etc)
	 *
	 * + Chipselect stays active during the entire message
	 *   (unless modified by spi_transfer.cs_change != 0).
	 * + The message transfers use clock and SPI mode parameters
	 *   previously established by setup() for this device
	 */
	int			(*transfer)(struct spi_device *spi,
						struct spi_message *mesg);

	/* called on release() to free memory provided by spi_controller_alt */
	void			(*cleanup)(struct spi_device *spi);

	/*
	 * Used to enable core support for DMA handling, if can_dma()
	 * exists and returns true then the transfer will be mapped
	 * prior to transfer_one() being called.  The driver should
	 * not modify or store xfer and dma_tx and dma_rx must be set
	 * while the device is prepared.
	 */
	bool			(*can_dma)(struct spi_controller_alt *ctlr,
					   struct spi_device *spi,
					   struct spi_transfer *xfer);
	struct device *dma_map_dev;

	/*
	 * These hooks are for drivers that want to use the generic
	 * controller transfer queueing mechanism. If these are used, the
	 * transfer() function above must NOT be specified by the driver.
	 * Over time we expect SPI drivers to be phased over to this API.
	 */
	bool				queued;
	struct kthread_worker		*kworker;
	struct kthread_work		pump_messages;
	spinlock_t			queue_lock;
	struct list_head		queue;
	struct spi_message		*cur_msg;
	bool				idling;
	bool				busy;
	bool				running;
	bool				rt;
	bool				auto_runtime_pm;
	bool                            cur_msg_prepared;
	bool				cur_msg_mapped;
	bool				last_cs_enable;
	bool				last_cs_mode_high;
	bool                            fallback;
	struct completion               xfer_completion;
	size_t				max_dma_len;

	int (*prepare_transfer_hardware)(struct spi_controller_alt *ctlr);
	int (*transfer_one_message)(struct spi_controller_alt *ctlr,
				    struct spi_message *mesg);
	int (*unprepare_transfer_hardware)(struct spi_controller_alt *ctlr);
	int (*prepare_message)(struct spi_controller_alt *ctlr,
			       struct spi_message *message);
	int (*unprepare_message)(struct spi_controller_alt *ctlr,
				 struct spi_message *message);
	int (*slave_abort)(struct spi_controller_alt *ctlr);

	/*
	 * These hooks are for drivers that use a generic implementation
	 * of transfer_one_message() provided by the core.
	 */
	void (*set_cs)(struct spi_device *spi, bool enable);
	int (*transfer_one)(struct spi_controller_alt *ctlr, struct spi_device *spi,
			    struct spi_transfer *transfer);
	void (*handle_err)(struct spi_controller_alt *ctlr,
			   struct spi_message *message);

	/* Optimized handlers for SPI memory-like operations. */
	const struct spi_controller_mem_ops *mem_ops;

	/* gpio chip select */
	int			*cs_gpios;
	struct gpio_desc	**cs_gpiods;
	bool			use_gpio_descriptors;
	s8			unused_native_cs;
	s8			max_native_cs;

	/* statistics */
	struct spi_statistics	statistics;

	/* DMA channels for use with core dmaengine helpers */
	struct dma_chan		*dma_tx;
	struct dma_chan		*dma_rx;

	/* dummy data for full duplex devices */
	void			*dummy_rx;
	void			*dummy_tx;

	int (*fw_translate_cs)(struct spi_controller_alt *ctlr, unsigned cs);

	/*
	 * Driver sets this field to indicate it is able to snapshot SPI
	 * transfers (needed e.g. for reading the time of POSIX clocks)
	 */
	bool			ptp_sts_supported;

	/* Interrupt enable state during PTP system timestamping */
	unsigned long		irq_flags;
};


/* --- Declare External Functions --- */

static void spi_controller_release(struct device *dev);

struct spi_controller_alt *spi_busnum_to_master_alt(u16 bus_num);

struct spi_device *spi_new_device_alt(struct spi_controller_alt *ctlr, struct spi_board_info *chip);

#endif