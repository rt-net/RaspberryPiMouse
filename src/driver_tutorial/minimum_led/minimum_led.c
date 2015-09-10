#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/stat.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <asm/uaccess.h>

#define	RASPBERRYPI2

MODULE_AUTHOR("RT Corporation");
MODULE_LICENSE("GPL");
MODULE_VERSION("1:1.0") ;
MODULE_DESCRIPTION("Raspberry pi MicroMouse device driver");

#define NUM_DEV_LED	4
#define NUM_DEV_TOTAL	(NUM_DEV_LED)
#define DEVNAME_LED	"rtled"

#define	DEV_MAJOR 0
#define	DEV_MINOR 0

static int _major_led = DEV_MAJOR;
static int _minor_led = DEV_MINOR;

static struct cdev *cdev_array = NULL;
static struct class *class_led = NULL;

static volatile void __iomem *pwm_base;
static volatile void __iomem *clk_base;
static volatile uint32_t *gpio_base;

static volatile int cdev_index = 0;
static volatile int open_counter = 0;

#define LED0_BASE 	25 
#define LED1_BASE 	24 
#define LED2_BASE 	23 
#define LED3_BASE 	18 

/* レジスタアドレス */
#define RPI_REG_BASE	0x3f000000

//gpio addr
#define RPI_GPIO_OFFSET	0x200000
#define RPI_GPIO_SIZE	0xC0
#define RPI_GPIO_BASE	(RPI_REG_BASE + RPI_GPIO_OFFSET)
#define	REG_GPIO_NAME	"RPi mouse GPIO"

//pwm addr
#define RPI_PWM_OFFSET	0x20C000
#define RPI_PWM_SIZE	0xC0
#define RPI_PWM_BASE	(RPI_REG_BASE + RPI_PWM_OFFSET)
#define	REG_PWM_NAME	"RPi mouse PWM"

//clock addr
#define RPI_CLK_OFFSET	0x101000
#define RPI_CLK_SIZE	0x100
#define RPI_CLK_BASE	(RPI_REG_BASE + RPI_CLK_OFFSET)
#define	REG_CLK_NAME	"RPi mouse CLK"

//clock offsets
#define CLK_PWM_INDEX		0xa0
#define CLK_PWMDIV_INDEX	0xa4

/* GPIO Functions	*/
#define	RPI_GPF_INPUT	0x00
#define	RPI_GPF_OUTPUT	0x01
#define	RPI_GPF_ALT0	0x04
#define	RPI_GPF_ALT5	0x02

/* GPIOレジスタインデックス */
#define RPI_GPFSEL0_INDEX	0

#define RPI_GPSET0_INDEX	7
#define RPI_GPCLR0_INDEX	10

#define RPI_GPIO_P2MASK (uint32_t)0xffffffff

static int rpi_gpio_function_set(int pin, uint32_t func)
{
	int index = RPI_GPFSEL0_INDEX + pin / 10;
	uint32_t mask = ~(0x7 << ((pin % 10) * 3));
	gpio_base[index] = (gpio_base[index] & mask) | ((func & 0x7) << ((pin % 10) * 3));
	
	return 1;
}

static void rpi_gpio_set32( uint32_t mask, uint32_t val )
{
	gpio_base[RPI_GPSET0_INDEX] = val & mask;
}

static void rpi_gpio_clear32( uint32_t mask, uint32_t val )
{
	gpio_base[RPI_GPCLR0_INDEX] = val & mask;
}

static int led_put(int ledno)
{
	switch(ledno)
	{
		case 0:
			rpi_gpio_set32( RPI_GPIO_P2MASK, 1 << LED0_BASE);
			break;
		case 1:
			rpi_gpio_set32( RPI_GPIO_P2MASK, 1 << LED1_BASE);
			break;
		case 2:
			rpi_gpio_set32( RPI_GPIO_P2MASK, 1 << LED2_BASE);
			break;
		case 3:
			rpi_gpio_set32( RPI_GPIO_P2MASK, 1 << LED3_BASE);
			break;
	}
	
	return 0;
}
static int led_del(int ledno)
{
	switch(ledno)
	{
		case 0:
			rpi_gpio_clear32( RPI_GPIO_P2MASK, 1 << LED0_BASE);
			break;
		case 1:
			rpi_gpio_clear32( RPI_GPIO_P2MASK, 1 << LED1_BASE);
			break;
		case 2:
			rpi_gpio_clear32( RPI_GPIO_P2MASK, 1 << LED2_BASE);
			break;
		case 3:
			rpi_gpio_clear32( RPI_GPIO_P2MASK, 1 << LED3_BASE);
			break;
	}
	
	return 0;
}

static int led_gpio_map(void)
{
	static int clk_status = 1;
	
	if(gpio_base == NULL)
	{
		gpio_base = ioremap_nocache(RPI_GPIO_BASE, RPI_GPIO_SIZE);
	}
	
	if(pwm_base == NULL)
	{
		pwm_base = ioremap_nocache(RPI_PWM_BASE, RPI_PWM_SIZE);
	}
	
	if(clk_base == NULL)
	{
		clk_base = ioremap_nocache(RPI_CLK_BASE, RPI_CLK_SIZE);
	}

	//kill
	if(clk_status == 1)
	{
		iowrite32(0x5a000000 | (1 << 5), clk_base + CLK_PWM_INDEX);
		udelay(1000);

		//clk set
		iowrite32(0x5a000000 | (2 << 12), clk_base + CLK_PWMDIV_INDEX);
		iowrite32(0x5a000011, clk_base + CLK_PWM_INDEX);

		udelay(1000);	//1mS wait
		
		clk_status = 0;
	}

	return 0;
}

static int dev_open(struct inode *inode, struct file *filep)
{
	int retval;
	int *minor = (int *)kmalloc(sizeof(int),GFP_KERNEL);
	int major = MAJOR(inode->i_rdev);
	*minor = MINOR(inode->i_rdev);

	printk(KERN_INFO "open request major:%d minor: %d \n", major, *minor);
	
	filep->private_data = (void *)minor;

	retval = led_gpio_map();
	if( retval != 0 ) {
		printk(KERN_ERR "Can not open led.\n" );
		return retval;
	}
	
	open_counter++;
	return 0;
}

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

static int dev_release(struct inode *inode, struct file *filep)
{
	kfree(filep->private_data);
	
	open_counter--;
	if(open_counter <= 0)
	{
		gpio_unmap();
	}
	return 0;
}

static ssize_t led_write( struct file *filep, const char __user *buf, size_t count, loff_t *f_pos)
{
	char cval;
	int ret;
	int minor =  *((int *)filep->private_data);
	
	if(count>0){
		if(copy_from_user(&cval, buf, sizeof(char))){
			return -EFAULT;
		}
		switch(cval){
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

static struct file_operations led_fops = {
	.open      = dev_open,
	.release   = dev_release,
	.write     = led_write,
};

static int led_register_dev(void)
{
	int retval;
	dev_t dev;
	int i;
	
	/* 空いているメジャー番号を使ってメジャー&
	   マイナー番号をカーネルに登録する */
	retval =  alloc_chrdev_region(
		&dev,				/* 結果を格納するdev_t構造体 */
		DEV_MINOR,		/* ベースマイナー番号 */
		NUM_DEV_LED,	/* デバイスの数 */
		DEVNAME_LED		/* デバイスドライバの名前 */
	);
	
	if( retval < 0 ) {
		printk(KERN_ERR "alloc_chrdev_region failed.\n" );
		return retval;
	}
	_major_led = MAJOR(dev);
	
	/* デバイスクラスを作成する */
	class_led = class_create(THIS_MODULE,DEVNAME_LED);
	if(IS_ERR(class_led))
		return PTR_ERR(class_led);
	
	for(i = 0; i < 4; i++)
	{
		/* デバイスの数だけキャラクタデバイスを登録する */
		dev_t devno = MKDEV(_major_led, _minor_led + i);
		/* キャラクタデバイスとしてこのモジュールをカーネルに登録する */
		cdev_init(&(cdev_array[cdev_index]), &led_fops);
		cdev_array[cdev_index].owner = THIS_MODULE;
		if( cdev_add( &(cdev_array[cdev_index]), devno, 1) < 0 ) {
			/* 登録に失敗した */
			printk(KERN_ERR "cdev_add failed minor = %d\n", _minor_led + i);
		}
		else {
			/* デバイスノードの作成 */
			device_create(
					class_led,
					NULL,
					devno,
					NULL,
					DEVNAME_LED"%u",_minor_led+i
			);
		}
		cdev_index ++;
	}
	
	return 0;
}

int dev_init_module(void)
{
	int retval;
	size_t size;
	
	/* 開始のメッセージ */
	printk(KERN_INFO "%s loading...\n", DEVNAME_LED );

	/* GPIOレジスタがマップ可能か調べる */
	retval = led_gpio_map();
	if( retval != 0 ) {
		printk( KERN_ALERT "Can not use GPIO registers.\n");
		return -EBUSY;
	}
	/* GPIO初期化 */
	rpi_gpio_function_set( LED0_BASE, RPI_GPF_OUTPUT );
	rpi_gpio_function_set( LED1_BASE, RPI_GPF_OUTPUT );
	rpi_gpio_function_set( LED2_BASE, RPI_GPF_OUTPUT );
	rpi_gpio_function_set( LED3_BASE, RPI_GPF_OUTPUT );
		
	/* cdev構造体の用意 */
	size = sizeof(struct cdev) * NUM_DEV_TOTAL;
	cdev_array =  (struct cdev*)kmalloc(size, GFP_KERNEL);
	
	/* デバイスドライバをカーネルに登録 */
	retval = led_register_dev();
	if( retval != 0 ) {
		printk( KERN_ALERT " led driver register failed.\n");
		return retval;
	}
	
	/* GPIOレジスタのアンマップ */
	gpio_unmap();

	printk("module being installed at %lu\n",jiffies);
	return 0;
}

void dev_cleanup_module(void)
{
	int i;
	dev_t devno;
		
	/* キャラクタデバイスの登録解除 */
	for(i = 0; i < NUM_DEV_TOTAL; i++)
	{
		cdev_del(&(cdev_array[i]));
	}
	
	
	for(i = 0; i < NUM_DEV_LED; i++)
	{
		devno = MKDEV(_major_led,_minor_led+i);
		device_destroy(class_led, devno);
	}
	devno = MKDEV(_major_led, _minor_led);
	unregister_chrdev_region(devno, NUM_DEV_LED);
	
	/* デバイスノードを取り除く */
	class_destroy( class_led );
	
	kfree(cdev_array);
	printk("module being removed at %lu\n",jiffies);
	
}

module_init(dev_init_module);
module_exit(dev_cleanup_module);
