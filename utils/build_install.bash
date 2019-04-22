#!/usr/bin/env bash

SRC_DIR=$(cd $(dirname ${BASH_SOURCE:-$0})/../; pwd)

if [ -e /usr/src/linux ]; then
	$SRC_DIR/utils/build_install.raspbian.bash && exit 0
elif [ "$(ls /usr/src/linux-* 2> /dev/null)" != '' ]; then
	$SRC_DIR/utils/build_install.ubuntu14.bash && exit 0
else
	bash -e $SRC_DIR/utils/print_env.bash "No kernel header files found."
fi

