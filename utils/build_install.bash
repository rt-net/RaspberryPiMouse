#!/usr/bin/env bash
set -eu

SRC_DIR=$(cd $(dirname ${BASH_SOURCE:-$0})/../; pwd)

if [ -e /usr/src/linux ]; then
	# build with linux headers installed from source
	if grep -q "Raspberry Pi 4" /proc/cpuinfo; then
		$SRC_DIR/utils/build_install_header_from_source_raspi4.bash && exit 0
	else
		$SRC_DIR/utils/build_install_header_from_source_raspi2.bash && exit 0
	fi
elif [ "$(ls /usr/src/linux-* 2> /dev/null)" != '' ]; then
	# build with linux headers installed with apt
	if grep -q "Raspberry Pi 4" /proc/cpuinfo; then
		$SRC_DIR/utils/build_install_header_from_apt_raspi4.bash && exit 0
	else
		$SRC_DIR/utils/build_install_header_from_apt_raspi2.bash && exit 0
	fi
else
	# Error
	bash -e $SRC_DIR/utils/print_env.bash "No kernel header files found."
fi
