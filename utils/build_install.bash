#!/usr/bin/env bash
set -eu

SRC_DIR=$(cd $(dirname ${BASH_SOURCE:-$0})/../; pwd)

if [ -e /usr/src/linux ]; then
	# Raspbian/Raspberry Pi OS
	$SRC_DIR/utils/build_install.raspbian.bash && exit 0
elif [ "$(ls /usr/src/linux-* 2> /dev/null)" != '' ]; then
	   # Ubuntu
	   if grep -q "Raspberry Pi 4" /proc/cpuinfo; then
			   $SRC_DIR/utils/build_install.raspi4ubuntu.bash && exit 0
	   else
			   $SRC_DIR/utils/build_install.ubuntu14.bash && exit 0
	   fi
else
	# Error
	bash -e $SRC_DIR/utils/print_env.bash "No kernel header files found."
fi

