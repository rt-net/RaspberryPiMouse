#!/usr/bin/env bash
set -eu

install_kernel_headers () {
    RPI_LINUX_VER="${RPI_LINUX_VER:-$1}"
    CONFIG_FILE="${CONFIG_FILE:-$2}"
    RPI_LINUX_COMMIT_HASH="${RPI_LINUX_COMMIT_HASH:-$3}"
    echo "==========================================="
    echo "build raspberrypi/linux"
    echo "==========================================="
    if [ -e ${HOME}/repo/raspberrypi/linux ]; then
        (cd ${HOME}/repo/raspberrypi/linux && git fetch origin && git checkout ${RPI_LINUX_VER} && git pull)
    else
        (cd ${HOME}/repo/raspberrypi && git clone -b ${RPI_LINUX_VER} https://github.com/raspberrypi/linux.git)
    fi
    cd ${HOME}/repo/raspberrypi/linux && make clean && git reset --hard HEAD && git checkout ${RPI_LINUX_COMMIT_HASH}
    cp "${SRC_DIR}/.test/${CONFIG_FILE}" ${HOME}/repo/raspberrypi/linux/.config
    export KERNEL=kernel7
    make CROSS_COMPILE=${CROSS_COMPILE_PATH} ARCH=arm oldconfig
    make CROSS_COMPILE=${CROSS_COMPILE_PATH} ARCH=arm -j 8
}
