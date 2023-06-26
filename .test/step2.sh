#!/usr/bin/env bash
set -eu

build_kernel_module () {
    echo "==========================================="
    echo "build kernel module"
    echo "==========================================="
    cd ${SRC_DIR}/src/drivers
    make CROSS_COMPILE=${CROSS_COMPILE_PATH} -f ${SRC_DIR}/.test/Makefile.crosscompile clean
    echo "==========================================="
    echo "==========================================="
    make CROSS_COMPILE=${CROSS_COMPILE_PATH} -f ${SRC_DIR}/.test/Makefile.crosscompile
    modinfo rtmouse.ko
    echo "==========================================="
    echo "==========================================="
    make CROSS_COMPILE=${CROSS_COMPILE_PATH} -f ${SRC_DIR}/.test/Makefile.crosscompile clean
}
