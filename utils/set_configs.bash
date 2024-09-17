#!/usr/bin/env bash
set -eu

# settings comment
SETTING_COMMENT='# Raspberry Pi Mouse V3 settings'

# OS architecture (32-bit or 64-bit)
ARCHITECTURE=$(getconf LONG_BIT)

# dtoverlay setting
DTOVERLAY='dtoverlay=anyspi:spi0-0,dev="microchip,mcp3204",speed=1000000'

# i2c_baudrate-setting
DTPARAM='dtparam=i2c_baudrate=62500'

# config-file PATH
CONFIG_FILE='/boot/firmware/config.txt'

# compare kernel versions as integers for accurate version comparison (excluding minor versions).
GET_KERNEL_VERSION_INT() {
    # 0-padding is used to avoid minor versions being compared by their first digit.
    echo "$1" | awk -F. '{ printf "%d%02d", $1, $2 }'
}

# kernel version
KERNEL_VERSION=$(uname -r | cut -d'-' -f1)
KERNEL_VERSION_INT=$(GET_KERNEL_VERSION_INT "$KERNEL_VERSION")

# add "Raspberry Pi Mouse v3" settings
if ! grep -qxF "$SETTING_COMMENT" "$CONFIG_FILE"; then
    echo "$SETTING_COMMENT" | sudo tee -a "$CONFIG_FILE" > /dev/null
fi

# check if the OS is running in 32-bit mode
if [[ "$ARCHITECTURE" == "32" ]]; then
    if ! grep -qxF "arm_64bit=0" "$CONFIG_FILE"; then
        echo "arm_64bit=0" | sudo tee -a "$CONFIG_FILE"
        echo "Add \"arm_64bit=0\"  > $CONFIG_FILE"
    fi
elif [[ "$ARCHITECTURE" == "64" ]]; then
    # remove arm_64bit=0 if present in a 64-bit environment
    if grep -qxF "arm_64bit=0" "$CONFIG_FILE"; then
        sudo sed -i '/arm_64bit=0/d' "$CONFIG_FILE"
        echo "Removed \"arm_64bit=0\" from $CONFIG_FILE"
    fi
fi

# add dtparam-setting for "/boot/firmware/config.txt"
if ! grep -qxF "$DTPARAM" "$CONFIG_FILE"; then
    echo "$DTPARAM" | sudo tee -a "$CONFIG_FILE" > /dev/null
    echo "Add  \"$DTPARAM\"  > $CONFIG_FILE"
fi

# use device-tree-overlay when the kernel is 5.16 or higher
if (( KERNEL_VERSION_INT >= $(GET_KERNEL_VERSION_INT 5.16) )); then
    # add dtoverlay-setting for "/boot/firmware/config.txt"
    if ! grep -qxF "$DTOVERLAY" "$CONFIG_FILE"; then
        echo "$DTOVERLAY" | sudo tee -a "$CONFIG_FILE" > /dev/null
        echo "Add  \"$DTOVERLAY\"  > $CONFIG_FILE"
    fi
else
    # remove dtoverlay-setting if kernel is less than 5.16
    if grep -qxF "$DTOVERLAY" "$CONFIG_FILE"; then
        sudo sed -i "/$DTOVERLAY/d" "$CONFIG_FILE"
        echo "Removed \"$DTOVERLAY\" from $CONFIG_FILE"
    fi
fi

# replace "dtparam=i2c_arm=off" with "dtparam=i2c_arm=on"
if grep -qxF 'dtparam=i2c_arm=off' "$CONFIG_FILE"; then
    sudo sed -i 's/dtparam=i2c_arm=off/dtparam=i2c_arm=on/' "$CONFIG_FILE"
    echo "Changed \"dtparam=i2c_arm=off\" to \"dtparam=i2c_arm=on\" in $CONFIG_FILE"
fi

# replace "dtparam=spi=off" with "dtparam=spi=on"
if grep -qxF 'dtparam=spi=off' "$CONFIG_FILE"; then
    sudo sed -i 's/dtparam=spi=off/dtparam=spi=on/' "$CONFIG_FILE"
    echo "Changed \"dtparam=spi=off\" to \"dtparam=spi=on\" in $CONFIG_FILE"
fi

# uncomment "dtparam=i2c_arm=on" if it is commented
if grep -qxF '#dtparam=i2c_arm=on' "$CONFIG_FILE"; then
    sudo sed -i 's/#dtparam=i2c_arm=on/dtparam=i2c_arm=on/' "$CONFIG_FILE"
    echo "Uncommented \"dtparam=i2c_arm=on\" in $CONFIG_FILE"
fi

# uncomment "dtparam=spi=on" if it is commented
if grep -qxF '#dtparam=spi=on' "$CONFIG_FILE"; then
    sudo sed -i 's/#dtparam=spi=on/dtparam=spi=on/' "$CONFIG_FILE"
    echo "Uncommented \"dtparam=spi=on\" in $CONFIG_FILE"
fi
