#!/bin/bash -vxe

dir=$(dirname $0)/../
[ ! -e /usr/src/linux-headers-$(uname -r) ] && { bash -e $dir/utils/print_env.bash "No kernel header files found."; exit 1; }
cd $dir/src/drivers/
rm Makefile
ln -s Makefile.ubuntu14 Makefile
make clean
make
sudo insmod rtmouse.ko
sleep 1
sudo chmod 666 /dev/rt*
echo 0 > /dev/rtmotoren0
