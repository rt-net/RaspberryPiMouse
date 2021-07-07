#!/usr/bin/env bash
set -eu

SRC_DIR=$(cd $(dirname ${BASH_SOURCE:-$0}); cd ../; pwd)


lint_driver () {
    pushd $SRC_DIR/src/drivers
        python3 $SRC_DIR/.test/bin/run-clang-format.py rtmouse.c
    popd
}

check_driver_version () {
    MOD_VER=$(grep MODULE_VERSION $SRC_DIR/src/drivers/rtmouse.c | sed -E 's/MODULE_VERSION\("([0-9.]*)"\);/\1/g')
    COMMENT_VER=$(grep '* Version' $SRC_DIR/src/drivers/rtmouse.c | sed -E 's/.*Version: ([0-9.]*)/\1/g')
    if [[ "$MOD_VER" != "$COMMENT_VER" ]]; then
        echo "The versions do not match."
        echo "Module Version: "$MOD_VER
        echo "Comment Version: "$COMMENT_VER
        exit 1
    else
        echo "Module Version and Comment Version matched."
    fi
}


lint_driver
check_driver_version