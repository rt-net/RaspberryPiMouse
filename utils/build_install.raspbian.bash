#!/bin/bash -eu

SRC_DIR=$(cd $(dirname ${BASH_SOURCE:-$0})/../; pwd)
KERNEL_SRC=/usr/src/linux-headers-$(uname -r)

# check kernel headers
[ ! -e $KERNEL_SRC ] && { bash -e $SRC_DIR/utils/print_env.bash "No kernel header files found."; exit 1; }

# build and install the driver
cd $SRC_DIR/src/drivers/
make clean
make 
sudo make install

# initialize the driver
sleep 1
sudo chmod 666 /dev/rt*
echo 0 > /dev/rtmotoren0
