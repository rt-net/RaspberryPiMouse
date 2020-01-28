#!/bin/bash
/bin/cp rtmouse.ko /lib/modules/`uname -r`/kernel/drivers/misc
/sbin/depmod
/bin/cp ../scripts/50-rtmouse.rules /etc/udev/rules.d/
/bin/cp ../scripts/rtmouse.sh /etc/init.d/
/bin/cp ../scripts/rtmouse.service /etc/systemd/system
