#!/usr/bin/env bash
set -eu

prepare_cross_compiler () {
    GCC_VER="${GCC_VER:-$1}"
    TAR_NAME="cross-gcc.tar.gz"
    mkdir -p ${HOME}/repo/raspberrypi

    echo "==========================================="
    echo "prepare cross compiler"
    echo "==========================================="

    dpkg -l | grep libgmp3-dev || sudo apt-get install libgmp3-dev
    dpkg -l | grep libmpc-dev || sudo apt-get install libmpc-dev

    if [ ! -e ${HOME}/repo/raspberrypi/cross-pi-gcc-${GCC_VER}.0-2 ]; then
        cd ${HOME}/repo/raspberrypi/
        if [ ! -e ${HOME}/repo/raspberrypi/cross-gcc-${GCC_VER}.0-pi_3+.tar.gz ]; then
            case "${GCC_VER}" in
                "6.3")
                    wget -O ${TAR_NAME} "https://sourceforge.net/projects/raspberry-pi-cross-compilers/files/Raspberry%20Pi%20GCC%20Cross-Compiler%20Toolchains/Stretch/GCC%206.3.0/Raspberry%20Pi%203A%2B%2C%203B%2B%2C%204/cross-gcc-6.3.0-pi_3%2B.tar.gz"
                    ;;
                "8.3")
                    wget -O ${TAR_NAME} "https://sourceforge.net/projects/raspberry-pi-cross-compilers/files/Raspberry%20Pi%20GCC%20Cross-Compiler%20Toolchains/Buster/GCC%208.3.0/Raspberry%20Pi%203A%2B%2C%203B%2B%2C%204/cross-gcc-8.3.0-pi_3%2B.tar.gz"
                    ;;
                "10.2")
                    wget -O ${TAR_NAME}  "https://jaist.dl.sourceforge.net/project/raspberry-pi-cross-compilers/Raspberry%20Pi%20GCC%20Cross-Compiler%20Toolchains/Bullseye/GCC%2010.2.0/Raspberry%20Pi%203A%2B%2C%203B%2B%2C%204/cross-gcc-10.2.0-pi_3%2B.tar.gz"
                    ;;
                "10.3")
                    wget -O ${TAR_NAME}  "https://jaist.dl.sourceforge.net/project/raspberry-pi-cross-compilers/Raspberry%20Pi%20GCC%20Cross-Compiler%20Toolchains/Bullseye/GCC%2010.3.0/Raspberry%20Pi%203A%2B%2C%203B%2B%2C%204/cross-gcc-10.3.0-pi_3%2B.tar.gz"
                    ;;
            esac
        fi
        tar xvf ${TAR_NAME}
    fi
    export CROSS_COMPILE_PATH="${HOME}/repo/raspberrypi/cross-pi-gcc-${GCC_VER}.0-2/bin/arm-linux-gnueabihf-"
}
