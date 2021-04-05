#!/usr/bin/env bash
set -eu

SRC_DIR=$(cd $(dirname ${BASH_SOURCE:-$0}); cd ../; pwd)

source ./step0.sh
source ./step1.sh
source ./step2.sh

# modify here
# ======================
RPI_LINUX_VER=rpi-5.10.y
CONFIG_FILE=config-5.10.11-v7+
RPI_LINUX_COMMIT_HASH=6af8ae3
GCC_VER=8.3
# ======================

prepare_cross_compiler $GCC_VER
install_kernel_headers $RPI_LINUX_VER $CONFIG_FILE $RPI_LINUX_COMMIT_HASH
build_kernel_module
