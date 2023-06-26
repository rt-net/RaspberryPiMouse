#!/usr/bin/env bash
set -eu

SRC_DIR=$(cd $(dirname ${BASH_SOURCE:-$0}); cd ../; pwd)

source ./step0.sh
source ./step1.sh
source ./step2.sh

# modify here
# ======================
# RPI_LINUX_VER=rpi-5.4.y
# CONFIG_FILE=config-5.4.83-v7+
# RPI_LINUX_COMMIT_HASH=76c49e6
# GCC_VER=8.3

# RPI_LINUX_VER=rpi-5.10.y
# CONFIG_FILE=config-5.10.11-v7+
# RPI_LINUX_COMMIT_HASH=6af8ae3
# GCC_VER=8.3

# RPI_LINUX_VER=rpi-5.15.y
# CONFIG_FILE=config-5.15.61-v7l+
# RPI_LINUX_COMMIT_HASH=5b775d7
# GCC_VER=10.2

RPI_LINUX_VER=rpi-5.15.y
CONFIG_FILE=config-5.15.76-v7l+
RPI_LINUX_COMMIT_HASH=45d339389bb85588b8045dd40a00c54d01e2e711
GCC_VER=10.2
# ======================

prepare_cross_compiler $GCC_VER
install_kernel_headers $RPI_LINUX_VER $CONFIG_FILE $RPI_LINUX_COMMIT_HASH
build_kernel_module
