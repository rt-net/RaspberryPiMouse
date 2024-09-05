#!/usr/bin/env bash
set -eu

# dtoverlay-setting
DTOVERLAY='dtoverlay=anyspi:spi0-0,dev="microchip,mcp3204",speed=1000000'
DTPARAM='dtparam=i2c_baudrate=62500'

# config-file PATH
CONFIG_FILE='/boot/firmware/config.txt'

# kernel version
KERNEL_VERSION=$(uname -r | cut -d'.' -f1,2)

# add dtparam-setting for "/boot/firmware/config.txt"
if ! grep -qxF "$DTPARAM" "$CONFIG_FILE"; then
    echo "$DTPARAM" | sudo tee -a "$CONFIG_FILE" >> /dev/null
    echo "Add  \"$DTPARAM\"  > $CONFIG_FILE"
fi

# use device-tree-overlay when the kernel is 5.16 or higher
if (( $(echo "$KERNEL_VERSION >= 5.16" | bc -l) )); then

    # add dtoverlay-setting for "/boot/firmware/config.txt"
    if ! grep -qxF "$DTOVERLAY" "$CONFIG_FILE"; then
        echo "$DTOVERLAY" | sudo tee -a "$CONFIG_FILE" >> /dev/null
        echo "Add  \"$DTOVERLAY\"  > $CONFIG_FILE"
    fi
fi