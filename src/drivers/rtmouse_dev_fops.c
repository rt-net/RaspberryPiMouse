/*
 *
 * rtmouse_dev.c
 * Raspberry Pi Mouse device driver
 *
 * Version: 3.3.2
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

/*
 * update_signed_count - update signed pulse count of dev_info
 * called by rtcnt_read()
 */
void update_signed_count(struct rtcnt_device_info *dev_info, int rtcnt_count)
{
	int diff_count = rtcnt_count - dev_info->raw_pulse_count;

	// カウントがMAX_PULSE_COUNTから0に変わる場合の処理
	// ただし、それ以外でもdiffが負の値になることがある
	// そのため、diffが十分に大きな負の値の場合に処理する
	// if(diff_count < 0) では正常に動作しない
	if (diff_count < -SIGNED_COUNT_SIZE) {
		diff_count += MAX_PULSE_COUNT;
	}

	if (dev_info->client->addr == DEV_ADDR_CNTL) {
		if (motor_l_freq_is_positive) {
			dev_info->signed_pulse_count += diff_count;
		} else {
			dev_info->signed_pulse_count -= diff_count;
		}
	} else {
		if (motor_r_freq_is_positive) {
			dev_info->signed_pulse_count += diff_count;
		} else {
			dev_info->signed_pulse_count -= diff_count;
		}
	}

	if (dev_info->signed_pulse_count > SIGNED_COUNT_SIZE ||
	    dev_info->signed_pulse_count < -SIGNED_COUNT_SIZE) {
		dev_info->signed_pulse_count = 0;
	}
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
 * reset_signed_count - reset signed pulse count of dev_info
 * called by rtcnt_write()
 */
void reset_signed_count(struct rtcnt_device_info *dev_info, int rtcnt_count)
{
	int raw_count;

	if (rtcnt_count > SIGNED_COUNT_SIZE) {
		rtcnt_count = SIGNED_COUNT_SIZE;
	} else if (rtcnt_count < -SIGNED_COUNT_SIZE) {
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

	if (dev_info->device_minor == 1) {
		update_signed_count(dev_info, rtcnt_count);
		dev_info->raw_pulse_count = rtcnt_count;
		rtcnt_count = dev_info->signed_pulse_count;
	} else {
		dev_info->raw_pulse_count = rtcnt_count;
	}

	/* set sensor data to rw_buf(static buffer) */
	sprintf(rw_buf, "%d\n", rtcnt_count);
	buflen = strlen(rw_buf);
	count = buflen;

	/* copy data to user area */
	if (copy_to_user((void *)buf, &rw_buf, count)) {
		printk(KERN_INFO "err read buffer from %s\n", rw_buf);
		printk(KERN_INFO "err sample_char_read size(%zu)\n", count);
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

	int rtcnt_count = 0;
	int ret;

	ret = kstrtoint_from_user(buf, count, 10, &rtcnt_count);
	if (ret) {
		printk(KERN_ERR "%s: error parsing string to int in %s()\n",
		       DRIVER_NAME, __func__);
		return ret;
	}

	i2c_counter_set(dev_info, rtcnt_count);

	if (dev_info->device_minor == 1) {
		reset_signed_count(dev_info, rtcnt_count);
	}

	printk(KERN_INFO "%s: set pulse counter value %d\n", DRIVER_NAME,
	       rtcnt_count);
	return count;
}

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
		printk(KERN_INFO "err sample_char_read size(%zu)\n", count);
		printk(KERN_INFO "sample_char_read size err(%d)\n", -EFAULT);
		return 0;
	}
	*f_pos += count;

	return count;
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

	unsigned int r = 0;
	unsigned char c = channel & 0x03;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0)

	if (mcp320x_dev == NULL)
		return 0;
	dev = mcp320x_dev;

#else
	struct spi_master *master;
	master = spi_busnum_to_master(mcp3204_info.bus_num);
	snprintf(str, sizeof(str), "%s.%u", dev_name(&master->dev),
		 mcp3204_info.chip_select);
	dev = bus_find_device_by_name(&spi_bus_type, NULL, str);
#endif

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
		printk(KERN_INFO "err sample_char_read size(%zu)\n", count);
		printk(KERN_INFO "sample_char_read size err(%d)\n", -EFAULT);
		return 0;
	}

	*f_pos += count;

	return count;
}

/* pwm set function */
static void rpi_pwm_write32(uint32_t offset, uint32_t val)
{
	iowrite32(val, pwm_base + offset);
}

/*
 * Initialize buzzer
 * return 0 : device close
 */
int buzzer_init(void)
{

	rpi_gpio_function_set(BUZZER_BASE, RPI_GPF_OUTPUT); // io is pwm out
	rpi_pwm_write32(RPI_PWM_CTRL, 0x00000000);
	udelay(1000);
	rpi_pwm_write32(RPI_PWM_CTRL, 0x00008181); // PWM1,2 enable

	// printk(KERN_DEBUG "%s: rpi_pwm_ctrl:%08X\n", DRIVER_NAME,
	// ioread32(pwm_base + RPI_PWM_CTRL));

	return 0;
}

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

/* left motor function */
static void set_motor_l_freq(int freq)
{
	int dat;

	rpi_gpio_function_set(BUZZER_BASE, RPI_GPF_OUTPUT);

	// Reset uncontrollable frequency to zero.
	if (abs(freq) < MOTOR_UNCONTROLLABLE_FREQ) {
		freq = 0;
	}

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

	// Reset uncontrollable frequency to zero.
	if (abs(freq) < MOTOR_UNCONTROLLABLE_FREQ) {
		freq = 0;
	}

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

/*set mask and value */
void rpi_gpio_set32(uint32_t mask, uint32_t val)
{
	gpio_base[RPI_GPSET0_INDEX] = val & mask;
}

/* clear mask and value */
void rpi_gpio_clear32(uint32_t mask, uint32_t val)
{
	gpio_base[RPI_GPCLR0_INDEX] = val & mask;
}

/* set function */
int rpi_gpio_function_set(int pin, uint32_t func)
{
	int index = RPI_GPFSEL0_INDEX + pin / 10;
	uint32_t mask = ~(0x7 << ((pin % 10) * 3));

	gpio_base[index] =
	    (gpio_base[index] & mask) | ((func & 0x7) << ((pin % 10) * 3));

	return 1;
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

/* --- Device File Operation --- */
/* Open Device */
int dev_open(struct inode *inode, struct file *filep)
{
	int *minor = (int *)kmalloc(sizeof(int), GFP_KERNEL);
	int major = MAJOR(inode->i_rdev);
	*minor = MINOR(inode->i_rdev);

	filep->private_data = (void *)minor;

	const char *dev_name = filep->f_path.dentry->d_name.name;
	printk(KERN_INFO "Device opened: %s, Major: %d\n", dev_name, major);

	return 0;
}

/* Close device */
int dev_release(struct inode *inode, struct file *filep)
{
	kfree(filep->private_data);
	return 0;
}

int i2c_dev_open(struct inode *inode, struct file *filep)
{
	struct rtcnt_device_info *dev_info;
	dev_info = container_of(inode->i_cdev, struct rtcnt_device_info, cdev);
	if (dev_info == NULL || dev_info->client == NULL) {
		printk(KERN_ERR "%s: i2c dev_open failed.\n", DRIVER_NAME);
	}
	dev_info->device_minor = MINOR(inode->i_rdev);
	filep->private_data = dev_info;
	return 0;
}

int i2c_dev_release(struct inode *inode, struct file *filep) { return 0; }

/*
 * led_write - Trun ON/OFF LEDs
 * Write function of /dev/rtled
 */
ssize_t led_write(struct file *filep, const char __user *buf, size_t count,
		  loff_t *f_pos)
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
ssize_t buzzer_write(struct file *filep, const char __user *buf, size_t count,
		     loff_t *f_pos)
{
	int ret;
	int freq, dat;

	ret = kstrtoint_from_user(buf, count, 10, &freq);
	if (ret) {
		printk(KERN_ERR "%s: error parsing string to int in %s()\n",
		       DRIVER_NAME, __func__);
		return ret;
	}

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

	return count;
}

/*
 *  rawmotor_l_write - Output frequency to the left motor
 *  Write function of /dev/rtmotor_raw_l
 */
ssize_t rawmotor_l_write(struct file *filep, const char __user *buf,
			 size_t count, loff_t *f_pos)
{
	int freq, ret;

	ret = kstrtoint_from_user(buf, count, 10, &freq);
	if (ret) {
		printk(KERN_ERR "%s: error parsing string to int in %s()\n",
		       DRIVER_NAME, __func__);
		return ret;
	}
	set_motor_l_freq(freq);

	return count;
}

/*
 *  rawmotor_r_write - Output frequency to the right motor
 *  Write function of /dev/rtmotor_raw_r
 */
ssize_t rawmotor_r_write(struct file *filep, const char __user *buf,
			 size_t count, loff_t *f_pos)
{
	int freq, ret;

	ret = kstrtoint_from_user(buf, count, 10, &freq);
	if (ret) {
		printk(KERN_ERR "%s: error parsing string to int in %s()\n",
		       DRIVER_NAME, __func__);
		return ret;
	}

	set_motor_r_freq(freq);

	return count;
}

/*
 * motoren_write - Turn ON/OFF SteppingMotor Power
 * Write function of /dev/rtmotoren
 */
ssize_t motoren_write(struct file *filep, const char __user *buf, size_t count,
		      loff_t *f_pos)
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
ssize_t motor_write(struct file *filep, const char __user *buf, size_t count,
		    loff_t *f_pos)
{
	int tmp;
	int bufcnt;
	bufcnt = parseMotorCmd(buf, count, &tmp);

	return bufcnt;
}

/* --- Device File Operations --- */
struct file_operations dev_fops[ID_DEV_SIZE] = {
    [ID_DEV_LED].open = dev_open,
    [ID_DEV_LED].release = dev_release,
    [ID_DEV_LED].write = led_write,
    [ID_DEV_SWITCH].open = dev_open,
    [ID_DEV_SWITCH].read = sw_read,
    [ID_DEV_SWITCH].release = dev_release,
    [ID_DEV_SENSOR].open = dev_open,
    [ID_DEV_SENSOR].read = sensor_read,
    [ID_DEV_SENSOR].release = dev_release,
    [ID_DEV_BUZZER].open = dev_open,
    [ID_DEV_BUZZER].release = dev_release,
    [ID_DEV_BUZZER].write = buzzer_write,
    [ID_DEV_MOTORRAWR].open = dev_open,
    [ID_DEV_MOTORRAWR].release = dev_release,
    [ID_DEV_MOTORRAWR].write = rawmotor_r_write,
    [ID_DEV_MOTORRAWL].open = dev_open,
    [ID_DEV_MOTORRAWL].release = dev_release,
    [ID_DEV_MOTORRAWL].write = rawmotor_l_write,
    [ID_DEV_MOTOREN].open = dev_open,
    [ID_DEV_MOTOREN].release = dev_release,
    [ID_DEV_MOTOREN].write = motoren_write,
    [ID_DEV_MOTOR].open = dev_open,
    [ID_DEV_MOTOR].release = dev_release,
    [ID_DEV_MOTOR].write = motor_write,
    [ID_DEV_CNT].open = i2c_dev_open,
    [ID_DEV_CNT].release = i2c_dev_release,
    [ID_DEV_CNT].read = rtcnt_read,
    [ID_DEV_CNT].write = rtcnt_write};
