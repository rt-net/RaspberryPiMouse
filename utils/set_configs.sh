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

# replace "dtparam=i2c_arm=off" with "dtparam=i2c_arm=on"
if grep -qxF 'dtparam=i2c_arm=off' "$CONFIG_FILE"; then
    sudo sed -i 's/dtparam=i2c_arm=off/dtparam=i2c_arm=on/' "$CONFIG_FILE"
    echo "changed \"dtparam=i2c_arm=off\" to \"dtparam=i2c_arm=on\" in $CONFIG_FILE"
fi

# replace "dtparam=spi=off" with "dtparam=spi=on"
if grep -qxF 'dtparam=spi=off' "$CONFIG_FILE"; then
    sudo sed -i 's/dtparam=spi=off/dtparam=spi=on/' "$CONFIG_FILE"
    echo "changed \"dtparam=spi=off\" to \"dtparam=spi=on\" in $CONFIG_FILE"
fi