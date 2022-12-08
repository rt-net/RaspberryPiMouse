#!/usr/bin/env bash
set -eu

prepare_cross_compiler () {
    GCC_VER="${GCC_VER:-$1}"
    mkdir -p ${HOME}/repo/raspberrypi

    echo "==========================================="
    echo "prepare cross compiler"
    echo "==========================================="

    dpkg -l | grep libgmp3-dev || sudo apt-get install libgmp3-dev
    dpkg -l | grep libmpc-dev || sudo apt-get install libmpc-dev

    if [ "${GCC_VER}" = "4.8" ]; then
        if [ -e ${HOME}/repo/raspberrypi/tools ]; then
            (cd ${HOME}/repo/raspberrypi/tools && git pull)
        else
            (cd ${HOME}/repo/raspberrypi && git clone --depth 1 https://github.com/raspberrypi/tools.git)
        fi
        export CROSS_COMPILE_PATH="${HOME}/repo/raspberrypi/tools/arm-bcm2708/arm-rpi-4.9.3-linux-gnueabihf/bin/arm-linux-gnueabihf-"
    fi
    if [ "${GCC_VER}" = "6.3" ]; then
        if [ ! -e ${HOME}/repo/raspberrypi/cross-pi-gcc-6.3.0-2 ]; then
            cd ${HOME}/repo/raspberrypi/
            if [ ! -e ${HOME}/repo/raspberrypi/cross-gcc-6.3.0-pi_3+.tar.gz ]; then
                wget --content-disposition "https://sourceforge.net/projects/raspberry-pi-cross-compilers/files/Raspberry%20Pi%20GCC%20Cross-Compiler%20Toolchains/Stretch/GCC%206.3.0/Raspberry%20Pi%203A%2B%2C%203B%2B%2C%204/cross-gcc-6.3.0-pi_3%2B.tar.gz"
            fi
            tar xvf cross-gcc-6.3.0-pi_3+.tar.gz
        fi
        export CROSS_COMPILE_PATH="${HOME}/repo/raspberrypi/cross-pi-gcc-6.3.0-2/bin/arm-linux-gnueabihf-"
    fi
    if [ "${GCC_VER}" = "8.3" ]; then
        if [ ! -e ${HOME}/repo/raspberrypi/cross-pi-gcc-8.3.0-2 ]; then
            cd ${HOME}/repo/raspberrypi/
            if [ ! -e ${HOME}/repo/raspberrypi/cross-gcc-8.3.0-pi_3+.tar.gz ]; then
                wget --content-disposition "https://sourceforge.net/projects/raspberry-pi-cross-compilers/files/Raspberry%20Pi%20GCC%20Cross-Compiler%20Toolchains/Buster/GCC%208.3.0/Raspberry%20Pi%203A%2B%2C%203B%2B%2C%204/cross-gcc-8.3.0-pi_3%2B.tar.gz"
            fi
            tar xvf cross-gcc-8.3.0-pi_3+.tar.gz
        fi
        export CROSS_COMPILE_PATH="${HOME}/repo/raspberrypi/cross-pi-gcc-8.3.0-2/bin/arm-linux-gnueabihf-"
    fi
    if [ "${GCC_VER}" = "10.2" ]; then
        if [ ! -e ${HOME}/repo/raspberrypi/cross-pi-gcc-10.2.0-2 ]; then
            cd ${HOME}/repo/raspberrypi/
            if [ ! -e ${HOME}/repo/raspberrypi/cross-gcc-10.2.0-pi_3+.tar.gz ]; then
                wget --content-disposition "https://jaist.dl.sourceforge.net/project/raspberry-pi-cross-compilers/Raspberry%20Pi%20GCC%20Cross-Compiler%20Toolchains/Bullseye/GCC%2010.2.0/Raspberry%20Pi%203A%2B%2C%203B%2B%2C%204/cross-gcc-10.2.0-pi_3%2B.tar.gz"
            fi
            tar xvf cross-gcc-10.2.0-pi_3+.tar.gz
        fi
        export CROSS_COMPILE_PATH="${HOME}/repo/raspberrypi/cross-pi-gcc-10.2.0-2/bin/arm-linux-gnueabihf-"
    fi
    if [ "${GCC_VER}" = "10.3" ]; then
        if [ ! -e ${HOME}/repo/raspberrypi/cross-pi-gcc-10.3.0-2 ]; then
            cd ${HOME}/repo/raspberrypi/
            if [ ! -e ${HOME}/repo/raspberrypi/cross-gcc-10.3.0-pi_3+.tar.gz ]; then
                wget --content-disposition "https://jaist.dl.sourceforge.net/project/raspberry-pi-cross-compilers/Raspberry%20Pi%20GCC%20Cross-Compiler%20Toolchains/Bullseye/GCC%2010.3.0/Raspberry%20Pi%203A%2B%2C%203B%2B%2C%204/cross-gcc-10.3.0-pi_3%2B.tar.gz"
            fi
            tar xvf cross-gcc-10.3.0-pi_3+.tar.gz
        fi
        export CROSS_COMPILE_PATH="${HOME}/repo/raspberrypi/cross-pi-gcc-10.3.0-2/bin/arm-linux-gnueabihf-"
    fi
}
