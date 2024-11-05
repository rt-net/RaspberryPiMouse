/*
 *
 * rtmouse.h
 * Raspberry Pi Mouse device driver header
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

#ifndef RTMOUSE_H
#define RTMOUSE_H

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kdev_t.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/stat.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>

// define the Raspberry Pi version here
// Raspberry Pi 1 B/A/B+/A+: 1
// Raspberry Pi 2 B        : 2
// Raspberry Pi 3 B/A+/B+  : 2
// Raspberry Pi 4 B        : 4
#define RASPBERRYPI 4

/* --- Device ID --- */
#define ID_DEV_LED 0
#define ID_DEV_SWITCH 1
#define ID_DEV_SENSOR 2
#define ID_DEV_BUZZER 3
#define ID_DEV_MOTORRAWR 4
#define ID_DEV_MOTORRAWL 5
#define ID_DEV_MOTOREN 6
#define ID_DEV_MOTOR 7
#define ID_DEV_CNT 8
#define ID_DEV_SIZE 9

#define NUM_DEV_TOTAL                                                          \
	(NUM_DEV[ID_DEV_LED] + NUM_DEV[ID_DEV_SWITCH] +                        \
	 NUM_DEV[ID_DEV_SENSOR] + NUM_DEV[ID_DEV_BUZZER] +                     \
	 NUM_DEV[ID_DEV_MOTORRAWR] + NUM_DEV[ID_DEV_MOTORRAWL] +               \
	 NUM_DEV[ID_DEV_MOTOREN] + NUM_DEV[ID_DEV_MOTOR])

#define DEVNAME_SENSOR "rtlightsensor"
#define DEVNAME_CNTR "rtcounter_r"
#define DEVNAME_CNTL "rtcounter_l"
#define DRIVER_NAME "rtmouse"

/* SPI */
#define SPI_BUS_NUM 0
#define SPI_CHIP_SELECT 0

/* --- Device Major and Minor Numbers --- */
#define DEV_MAJOR 0
#define DEV_MINOR 0

/* --- GPIO Pin Definitions --- */
#define R_AD_CH 3
#define L_AD_CH 0
#define RF_AD_CH 2
#define LF_AD_CH 1

#define R_LED_BASE 22
#define L_LED_BASE 4
#define RF_LED_BASE 27
#define LF_LED_BASE 17

#define LED0_BASE 25
#define LED1_BASE 24
#define LED2_BASE 23
#define LED3_BASE 18

#define SW1_PIN 20
#define SW2_PIN 26
#define SW3_PIN 21

#define BUZZER_BASE 19

#define MOTCLK_L_BASE 12
#define MOTDIR_L_BASE 16

#define MOTCLK_R_BASE 13
#define MOTDIR_R_BASE 6

#define MOTEN_BASE 5

#define PWM_ORG0_BASE 40
#define PWM_ORG1_BASE 45

/* --- Register Address --- */
/* Base Addr */
#if RASPBERRYPI == 1
#define RPI_REG_BASE 0x20000000
#elif RASPBERRYPI == 2
#define RPI_REG_BASE 0x3f000000
#elif RASPBERRYPI == 4
#define RPI_REG_BASE 0xfe000000
/* 2711 has a different mechanism for pin pull-up/down/enable  */
#define GPPUPPDN0 57 /* Pin pull-up/down for pins 15:0  */
#define GPPUPPDN1 58 /* Pin pull-up/down for pins 31:16 */
#define GPPUPPDN2 59 /* Pin pull-up/down for pins 47:32 */
#define GPPUPPDN3 60 /* Pin pull-up/down for pins 57:48 */
#endif

/* GPIO Addr */
#define RPI_GPIO_OFFSET 0x200000
#define RPI_GPIO_SIZE 0xC0
#define RPI_GPIO_BASE (RPI_REG_BASE + RPI_GPIO_OFFSET)
#define REG_GPIO_NAME "RPi mouse GPIO"

/* Pwm Addr */
#define RPI_PWM_OFFSET 0x20C000
#define RPI_PWM_SIZE 0xC0
#define RPI_PWM_BASE (RPI_REG_BASE + RPI_PWM_OFFSET)
#define REG_PWM_NAME "RPi mouse PWM"

/* Clock Addr */
#define RPI_CLK_OFFSET 0x101000
#define RPI_CLK_SIZE 0x100
#define RPI_CLK_BASE (RPI_REG_BASE + RPI_CLK_OFFSET)
#define REG_CLK_NAME "RPi mouse CLK"

/* --- General Options --- */
/* Clock Offset */
#define CLK_PWM_INDEX 0xa0
#define CLK_PWMDIV_INDEX 0xa4

/* GPIO PUPD select */
#if RASPBERRYPI == 4
#define GPIO_PULLNONE 0x0
#define GPIO_PULLUP 0x1
#define GPIO_PULLDOWN 0x2
#else
#define GPIO_PULLNONE 0x0
#define GPIO_PULLDOWN 0x1
#define GPIO_PULLUP 0x2
#endif

/* GPIO Function */
#define RPI_GPF_INPUT 0x00
#define RPI_GPF_OUTPUT 0x01
#define RPI_GPF_ALT0 0x04
#define RPI_GPF_ALT5 0x02

/* GPIO Register Index */
#define RPI_GPFSEL0_INDEX 0
#define RPI_GPFSEL1_INDEX 1
#define RPI_GPFSEL2_INDEX 2
#define RPI_GPFSEL3_INDEX 3

#define RPI_GPSET0_INDEX 7
#define RPI_GPCLR0_INDEX 10

/* GPIO Mask */
#define RPI_GPIO_P1MASK                                                        \
	(uint32_t)((0x01 << 2) | (0x01 << 3) | (0x01 << 4) | (0x01 << 7) |     \
		   (0x01 << 8) | (0x01 << 9) | (0x01 << 10) | (0x01 << 11) |   \
		   (0x01 << 14) | (0x01 << 15) | (0x01 << 17) | (0x01 << 18) | \
		   (0x01 << 22) | (0x01 << 23) | (0x01 << 24) | (0x01 << 25) | \
		   (0x01 << 27))
#define RPI_GPIO_P2MASK (uint32_t)0xffffffff

/* PWM Index */
#define RPI_PWM_CTRL 0x0
#define RPI_PWM_STA 0x4
#define RPI_PWM_DMAC 0x8
#define RPI_PWM_RNG1 0x10
#define RPI_PWM_DAT1 0x14
#define RPI_PWM_FIF1 0x18
#define RPI_PWM_RNG2 0x20
#define RPI_PWM_DAT2 0x24

#if RASPBERRYPI == 4
#define PWM_BASECLK 27000000
#else
#define PWM_BASECLK 9600000
#endif

/* A/D Parameter */
#define MCP320X_PACKET_SIZE 3
#define MCP320X_DIFF 0
#define MCP320X_SINGLE 1
#define MCP3204_CHANNELS 4

/* I2C Parameter */
#define DEV_ADDR_CNTL 0x10
#define DEV_ADDR_CNTR 0x11
#define CNT_ADDR_MSB 0x10
#define CNT_ADDR_LSB 0x11

/* Motor Parameter */
#define MOTOR_UNCONTROLLABLE_FREQ 5

/* I2C */
#define SIGNED_COUNT_SIZE 32767
#define MAX_PULSE_COUNT 65535

/* --- Buffer --- */
#define MAX_BUFLEN 64

/* --- extern --- */
extern unsigned int NUM_DEV[ID_DEV_SIZE];
extern char *NAME_DEV[ID_DEV_SIZE];
extern char *NAME_DEV_U[ID_DEV_SIZE];
extern int _major_dev[ID_DEV_SIZE];
extern int _minor_dev[ID_DEV_SIZE];
extern struct cdev *cdev_array;
extern struct class *class_dev[ID_DEV_SIZE];
extern volatile void __iomem *pwm_base;
extern volatile void __iomem *clk_base;
extern volatile uint32_t *gpio_base;
extern volatile int cdev_index;
extern struct mutex lock;
extern struct spi_device_id mcp3204_id[];
extern struct spi_board_info mcp3204_info;
extern struct spi_driver mcp3204_driver;
extern struct i2c_client *i2c_client_r;
extern struct i2c_client *i2c_client_l;
extern unsigned int motor_l_freq_is_positive;
extern unsigned int motor_r_freq_is_positive;
extern struct i2c_device_id i2c_counter_id[];
extern struct i2c_driver i2c_counter_driver;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0)
extern struct device *mcp320x_dev;
#endif

void tmp_func(void);

#endif // RTMOUSE_H
