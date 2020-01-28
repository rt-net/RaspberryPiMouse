#!/bin/bash

PROC_FILE=/proc/modules
GREP=/bin/grep
MODPROBE=/sbin/modprobe
MODULE_NAME=rtmouse
DEP_MODULE_NAME=mcp320x

[ -f $PROC_FILE ] || exit 0
[ -x $GREP ] || exit 0
[ -x $MODPROBE ] || exit 0
RES=`$GREP $MODULE_NAME $PROC_FILE`

if [ "$RES" = "" ]; then
    echo "Module '$MODULE_NAME' isn't installed yet."
else
    $MODPROBE -r $MODULE_NAME
    $MODPROBE -r $DEP_MODULE_NAME
    echo "Module '$MODULE_NAME' is rmoved."
 fi

/bin/rm /etc/udev/rules.d/50-rtmouse.rules
/bin/rm /etc/init.d/rtmouse.sh
/bin/rm /etc/systemd/system/rtmouse.service
/bin/rm /lib/modules/`uname -r`/kernel/drivers/misc/rtmouse.ko
