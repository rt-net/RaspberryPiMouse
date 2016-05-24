#!/bin/bash -vxe

dir=$(dirname $0)/../

cd $dir/src/drivers/
rm Makefile
ln -s Makefile.ubuntu14 Makefile
make clean
make 
sudo insmod rtmouse.ko
