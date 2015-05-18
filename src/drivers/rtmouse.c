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
#include <asm/delay.h>

//Raspberry pi version setting

//
//for Raspberry pi version 1 
//#define RASPBERRYPI1
//#undef	RASPBERRYPI2
//

//for Raspberry pi version 2 
#undef	RASPBERRYPI1
#define RASPBERRYPI2

MODULE_AUTHOR("RT Corporation");
MODULE_LICENSE("GPL");
MODULE_VERSION("1:1.0") ;
MODULE_DESCRIPTION("Raspberry pi MicroMouse device driver");


#define NUM_DEV_LED	4
#define NUM_DEV_SWITCH	3
#define NUM_DEV_BUZZER	1

#define NUM_DEV_TOTAL (NUM_DEV_LED + NUM_DEV_SWITCH + NUM_DEV_BUZZER)

#define DEVNAME_LED "rtled" //device name 
#define DEVNAME_SWITCH "rtswitch" //device name 
#define DEVNAME_BUZZER "rtbuzzer" //device name 
#define	DEV_MAJOR 0 //autosetting 0
#define	DEV_MINOR 0

static int _major_led = DEV_MAJOR;
static int _minor_led = DEV_MINOR;

static int _major_switch = DEV_MAJOR;
static int _minor_switch = DEV_MINOR;

static int _major_buzzer = DEV_MAJOR;
static int _minor_buzzer = DEV_MINOR;

static struct cdev *cdev_array = NULL;
static struct class *class_led = NULL;
static struct class *class_buzzer = NULL;
static struct class *class_switch = NULL;

static volatile void __iomem *pwm_base;
static volatile void __iomem *clk_base;
static volatile uint32_t *gpio_base;

static volatile int cdev_index = 0;
static volatile int open_counter = 0;


#define LED0_BASE 	25 
#define LED1_BASE 	24 
#define LED2_BASE 	23 
#define LED3_BASE 	18 

#define SW1_PIN		20 
#define SW2_PIN		26 
#define SW3_PIN		21 

#define BUZZER_BASE	19

#define MOTCLK_L_BASE	12
#define MOTDIR_L_BASE	16
#define MOTEN_BASE	13

#define PWM_ORG0_BASE	40
#define PWM_ORG1_BASE	45


/* レジスタアドレス */
//base addr
#ifdef RASPBERRYPI1
	#define RPI_REG_BASE	0x20000000
#else
	#define RPI_REG_BASE	0x3f000000
#endif

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


/* GPIO PUPD select */
#define GPIO_PULLNONE 0x0
#define GPIO_PULLDOWN 0x1
#define GPIO_PULLUP   0x2

/* GPIO Functions	*/
#define	RPI_GPF_INPUT	0x00
#define	RPI_GPF_OUTPUT	0x01
#define	RPI_GPF_ALT0	0x04
#define	RPI_GPF_ALT5	0x02

/* GPIOレジスタインデックス */
#define RPI_GPFSEL0_INDEX	0
#define RPI_GPFSEL1_INDEX	1
#define RPI_GPFSEL2_INDEX	2
#define RPI_GPFSEL3_INDEX	3

#define RPI_GPSET0_INDEX	7
#define RPI_GPCLR0_INDEX	10

//PWM インデックス
#define RPI_PWM_CTRL	0x0
#define RPI_PWM_STA	0x4
#define RPI_PWM_DMAC	0x8
#define RPI_PWM_RNG1	0x10
#define RPI_PWM_DAT1	0x14
#define RPI_PWM_FIF1	0x18
#define RPI_PWM_RNG2	0x20
#define RPI_PWM_DAT2	0x24

#define PWM_BASECLK	9600000

/* GPIO Mask */
#define RPI_GPIO_P1MASK	(uint32_t) ((0x01<<2) | (0x01<<3) | (0x01<<4) | \
									(0x01<<7) | (0x01<<8) | (0x01<<9) | \
									(0x01<<10)| (0x01<<11)| (0x01<<14)| \
									(0x01<<15)| (0x01<<17)| (0x01<<18)| \
									(0x01<<22)| (0x01<<23)| (0x01<<24)| \
									(0x01<<25)| (0x01<<27)\
								   )
#define RPI_GPIO_P2MASK (uint32_t)0xffffffff

// 内部バッファ
#define MAX_BUFLEN 64
unsigned char sw_buf[ MAX_BUFLEN ];
static int buflen = 0;

typedef struct
{
	int major;
	int minor;
	void *ptr;
}t_malloc_pool;


t_malloc_pool malloc_pool[64] = {0};
static const dev_t devinit;


static void init_malloc_pool_one(t_malloc_pool *t)
{
	t->major = -1;
	t->minor = -1;
}

static void init_malloc_pool(void)
{
	int i;
	for(i = 0; i < 64; i++)
	{
		init_malloc_pool_one(malloc_pool + i);
	}
}

static void push_malloc_pool(dev_t key, void *ptr)
{
	int i;
	for(i = 0; i < 64; i++)
	{
		if((malloc_pool[i].major == -1) && (malloc_pool[i].minor == -1))
		{
			malloc_pool[i].major = MAJOR(key);
			malloc_pool[i].minor = MINOR(key);
			malloc_pool[i].ptr = ptr;
		}
	}
}

static void delete_malloc_pool(dev_t key)
{
	int i;
	int minor = MINOR(key);
	int major = MAJOR(key);
	
	for(i = 0; i < 64; i++)
	{
		if((malloc_pool[i].major == MAJOR(key)) && (malloc_pool[i].minor == MINOR(key)))
		{
			kfree(malloc_pool[i].ptr);
			malloc_pool[i].major = -1;
			malloc_pool[i].minor = -1;
		}
	}
}

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

static void rpi_pwm_write32(uint32_t offset, uint32_t val)
{
	iowrite32(val, pwm_base + offset);
}



//ここでreturn 0はデバイスクローズなのでやってはいけな��
static ssize_t sw_read(struct file *filep, char __user *buf, size_t count, loff_t *f_pos)
{
	
	unsigned int ret=0;
	int len;
	int index;
	unsigned int pin=SW1_PIN;
	uint32_t mask;
	int _eof=-1;
	int minor =  *((int *)filep->private_data);

	switch(minor)
	{
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
	
	if(*f_pos > 0) return 0; /* End of file */

		//  プルモード (2bit)を書き込む NONE/DOWN/UP
		gpio_base[37] = GPIO_PULLUP & 0x3;  //  GPPUD
		//  ピンにクロックを供給（前後にウェイト��	
		msleep(1);
		gpio_base[38] = 0x1 << pin;      //  GPPUDCLK0
		msleep(1);
		//  プルモード・クロック状態をクリアして終��
		gpio_base[37] = 0;
		gpio_base[38] = 0;


	index= RPI_GPFSEL0_INDEX + pin / 10;
	mask= ~(0x7 << ((pin % 10) * 3));
	printk( KERN_INFO "sw_read mask %d\n", mask);

	ret = ((gpio_base[13] & (0x01 << pin))!=0);
	sprintf(sw_buf, "%d\n", ret);
	buflen = strlen(sw_buf);
	count=buflen+1;
	len=buflen;
	printk( KERN_INFO "sw_read ret %d\n", ret);
	
	if(copy_to_user((void *)buf, &sw_buf, count))
	{
	printk(KERN_INFO "err read buffer from ret  %d\n", ret);
	printk(KERN_INFO "err read buffer from %s\n", sw_buf);
	printk(KERN_INFO "err sample_char_read size(%d)\n", count);
		printk(KERN_INFO "sample_char_read size err(%d)\n", -EFAULT);
		return 0;
	}
	*f_pos += count;
	buflen=0;

	printk(KERN_INFO "read buffer from ret  %d\n", ret);
	printk(KERN_INFO "read buffer from sw_buf %s\n", sw_buf);
	printk(KERN_INFO "read buffer from len  %d\n", len);
	printk(KERN_INFO "sample_char_read size(%d)\n", count);

	//msleep(1000);
	//gpio_unmap();

	return count;
	//return 0;
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



static int buzzer_init(void)
{

	rpi_gpio_function_set(BUZZER_BASE, RPI_GPF_OUTPUT);	//io is pwm out
	rpi_pwm_write32(RPI_PWM_CTRL, 0x00000000);
	udelay(1000);
	rpi_pwm_write32(RPI_PWM_CTRL, 0x00008181);		//PWM1,2 enable

	printk(KERN_INFO "rpi_pwm_ctrl:%08X\n", ioread32(pwm_base + RPI_PWM_CTRL));
	
	return 0;
}

static int led_gpio_map(void)
{
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
	iowrite32(0x5a000000 | (1 << 5), clk_base + CLK_PWM_INDEX);
	udelay(1000);

	//clk set
	iowrite32(0x5a000000 | (2 << 12), clk_base + CLK_PWMDIV_INDEX);
	iowrite32(0x5a000011, clk_base + CLK_PWM_INDEX);

	udelay(1000);	//1mS wait

	return 0;
}

static int dev_open(struct inode *inode, struct file *filep)
{
		int retval;
	int *minor = (int *)kmalloc(sizeof(int),GFP_KERNEL);
	int major = MAJOR(inode->i_rdev);
	*minor = MINOR(inode->i_rdev);
	
	
	
	//push_malloc_pool(inode->i_rdev, minor);
	
	printk(KERN_INFO "open request major:%d minor: %d \n", major, *minor);
	
	filep->private_data = (void *)minor;
	
	/*
		if( gpio_base != NULL ) {
				printk(KERN_ERR "led is already open.\n" );
				return -EIO;
		}
	*/
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
	
	//delete_malloc_pool(inode->i_rdev);
	
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

static ssize_t buzzer_write( struct file *filep, const char __user *buf, size_t count, loff_t *f_pos)
{
	char cval;
	size_t readcount = count;
	char *newbuf = kmalloc(sizeof(char)*count, GFP_KERNEL);
	int error = 0, i = 0, tmp, bufcnt = 0;
	int freq, dat;
	
	while(readcount > 0){
		if(copy_from_user(&cval, buf + i, sizeof(char))){
			return -EFAULT;
		}
		
		if(cval < '0' || cval > '9')
		{
			newbuf[bufcnt] = 'e';
			error = 1;
		}
		else
		{
			newbuf[bufcnt] = cval;
			
		}
		
		i++;
		bufcnt++;
				readcount--;
	}

	freq = 0;
	for(i = 0, tmp = 1; i < bufcnt; i++)
	{
		char c = newbuf[bufcnt - i - 1];
		
		if( c >= '0' && c <= '9')
		{
			freq += (newbuf[bufcnt - i - 1] - '0') * tmp;
			tmp *= 10;
		}
	}
	
	if(freq != 0)
	{
		rpi_gpio_function_set(BUZZER_BASE, RPI_GPF_ALT5);	//io is pwm out
		dat = PWM_BASECLK / freq;
		rpi_pwm_write32(RPI_PWM_RNG2, dat);
		rpi_pwm_write32(RPI_PWM_DAT2, dat >> 1);
		
		printk(KERN_INFO "pwmdata:%d\n",dat);
	}
	else
	{
		rpi_gpio_function_set(BUZZER_BASE, RPI_GPF_OUTPUT);	//io is pwm out
	}
	
	kfree(newbuf);
	
	return count;
}

struct file_operations led_fops = {
		.open      = dev_open,
		.release   = dev_release,
		.write     = led_write,
};

struct file_operations buzzer_fops = {
		.open      = dev_open,
		.release   = dev_release,
		.write     = buzzer_write,
};


struct file_operations fops = {
		.open      = dev_open,
		.read      = sw_read,
		.release   = dev_release,
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

static int buzzer_register_dev(void)
{
	int retval;
	dev_t dev;
	int i;
	
	/* 空いているメジャー番号を使ってメジャー&
	   マイナー番号をカーネルに登録する */
	retval =  alloc_chrdev_region(
		&dev,				/* 結果を格納するdev_t構造体 */
		DEV_MINOR,			/* ベースマイナー番号 */
		NUM_DEV_BUZZER,			/* デバイスの数 */
		DEVNAME_BUZZER		/* デバイスドライバの名前 */
	);
	
	if( retval < 0 ) {
		printk(KERN_ERR "alloc_chrdev_region failed.\n" );
		return retval;
	}
	_major_buzzer = MAJOR(dev);
	
	/* デバイスクラスを作成する */
	class_buzzer = class_create(THIS_MODULE,DEVNAME_BUZZER);
	if(IS_ERR(class_buzzer))
		return PTR_ERR(class_buzzer);

	dev_t devno = MKDEV(_major_buzzer, _minor_buzzer);
	/* キャラクタデバイスとしてこのモジュールをカーネルに登録する */
	cdev_init(&(cdev_array[cdev_index]), &buzzer_fops);
	cdev_array[cdev_index].owner = THIS_MODULE;
	if( cdev_add( &(cdev_array[cdev_index]), devno, 1) < 0 ) {
		/* 登録に失敗した */
		printk(KERN_ERR "cdev_add failed minor = %d\n", _minor_buzzer );
	}
	else {
		/* デバイスノードの作成 */
		device_create(
				class_buzzer,
				NULL,
				devno,
				NULL,
				DEVNAME_BUZZER"%u",_minor_buzzer
		);
	}
	
	cdev_index++;

	return 0;
}

static int switch_register_dev(void)
{
	int retval;
	dev_t dev;
	size_t size;
	int i;
		
	/* 空いているメジャー番号を使ってメジャー&マイナー番号をカーネルに登録する */
	retval =  alloc_chrdev_region(
		&dev,				/* 結果を格納するdev_t構造体 */
		DEV_MINOR,		/* ベースマイナー番号 */
		NUM_DEV_SWITCH,	/* デバイスの数 */
		DEVNAME_SWITCH		/* デバイスドライバの名前 */
	);
	
	if( retval < 0 ) {
		printk(KERN_ERR "alloc_chrdev_region failed.\n" );
		return retval;
	}
	_major_switch = MAJOR(dev);
	
	/* デバイスクラスを作成する */
	class_switch = class_create(THIS_MODULE,DEVNAME_SWITCH);
	if(IS_ERR(class_switch))
	return PTR_ERR(class_switch);

	/* デバイスの数だけキャラクタデバイスを登録する */
	for( i = 0; i < 3; i++ ) {
		dev_t devno = MKDEV(_major_switch, _minor_switch + i);
		/* キャラクタデバイスとしてこのモジュールをカーネルに登録する */
		cdev_init(&(cdev_array[cdev_index]), &fops);
		cdev_array[i].owner = THIS_MODULE;
		if( cdev_add( &(cdev_array[cdev_index]), devno, 1) < 0 ) {
			/* 登録に失敗した */
			printk(KERN_ERR "cdev_add failed minor = %d\n", _minor_switch+i );
		}
		else {
			/* デバイスノードの作成 */
			device_create(
					class_switch,
					NULL,
					devno,
					NULL,
					DEVNAME_SWITCH"%u",_minor_switch+i
			);
		}
		cdev_index++;
	}
	return 0;
}



int dev_init_module(void)
{
		int retval, i;
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
		//led_gpio_setup();
	rpi_gpio_function_set( LED0_BASE, RPI_GPF_OUTPUT );
	rpi_gpio_function_set( LED1_BASE, RPI_GPF_OUTPUT );
	rpi_gpio_function_set( LED2_BASE, RPI_GPF_OUTPUT );
	rpi_gpio_function_set( LED3_BASE, RPI_GPF_OUTPUT );
	
	rpi_gpio_function_set( BUZZER_BASE, RPI_GPF_OUTPUT );
	rpi_gpio_function_set( MOTDIR_L_BASE, RPI_GPF_OUTPUT );
	rpi_gpio_function_set( MOTEN_BASE, RPI_GPF_OUTPUT );
	rpi_gpio_function_set( MOTCLK_L_BASE, RPI_GPF_ALT0 );
	
	rpi_gpio_function_set( SW1_PIN, RPI_GPF_INPUT );
	rpi_gpio_function_set( SW2_PIN, RPI_GPF_INPUT );
	rpi_gpio_function_set( SW3_PIN, RPI_GPF_INPUT );
	
	
	rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << MOTEN_BASE);
	rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << MOTDIR_L_BASE);

	for(i = 0; i < 100; i++)
	{
		rpi_gpio_set32( RPI_GPIO_P2MASK, 1 << BUZZER_BASE);
		udelay(500);
		rpi_gpio_clear32( RPI_GPIO_P2MASK, 1 << BUZZER_BASE);
		udelay(500);
	}
	buzzer_init();

	/* cdev構造体の用意 */
	size = sizeof(struct cdev) * NUM_DEV_TOTAL;
	cdev_array =  (struct cdev*)kmalloc(size, GFP_KERNEL);
	
		/* デバイスドライバをカーネルに登録 */
		retval = led_register_dev();
		if( retval != 0 ) {
			   printk( KERN_ALERT " led driver register failed.\n");
			 return retval;
		}
	
	retval = buzzer_register_dev();
		if( retval != 0 ) {
			   printk( KERN_ALERT " led driver register failed.\n");
			 return retval;
		}
	
	retval = switch_register_dev();
		if( retval != 0 ) {
			   printk( KERN_ALERT " led driver register failed.\n");
			 return retval;
		}
	
		printk( KERN_INFO "led driver register sccessed.\n");

		/* GPIOレジスタのアンマップ */
	gpio_unmap();

	init_malloc_pool();

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
	
	
	for( i = 0; i < 3; i++ ) {
		devno = MKDEV(_major_switch, _minor_switch + i);
		device_destroy(class_switch, devno);
	}
	devno = MKDEV(_major_switch, _minor_switch);
	unregister_chrdev_region(devno, NUM_DEV_SWITCH);
	
	devno = MKDEV(_major_buzzer,_minor_buzzer); 
	device_destroy(class_buzzer, devno);
	unregister_chrdev_region(devno, NUM_DEV_BUZZER);
	
	/* デバイスノードを取り除く */
	class_destroy( class_led );
	class_destroy( class_switch );
	class_destroy( class_buzzer );
	
	
	kfree(cdev_array);
	printk("module being removed at %lu\n",jiffies);
}



module_init(dev_init_module);
module_exit(dev_cleanup_module);



