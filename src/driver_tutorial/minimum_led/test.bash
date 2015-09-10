#!/bin/bash -xv

sudo insmod minimum_led.ko
sudo chmod 666 /dev/rt*
echo 1 > /dev/rtled3
sleep 3
echo 0 > /dev/rtled3
sudo rmmod minimum_led
