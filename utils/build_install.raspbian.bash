#!/bin/bash -vxe

dir=$(dirname $0)

cd $dir/src/drivers/
rm Makefile
ln -s Makefile.raspbian Makefile
make clean
make 
sudo insmod rtmouse.ko
sleep 1
sudo chmod 666 /dev/rt*
echo 0 > /dev/rtmotoren0
