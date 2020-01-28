#!/bin/bash
#
#
### BEGIN INIT INFO
# Provides:          rtmouse
# Required-Start:    $all
# Required-Stop:
# Default-Start:     2 3 4 5
# Default-Stop:
# Short-Description: RT_Mouse_Driver
# Description:       RaspPiMouse Driver
### END INIT INFO
SCRIPTNAME=rtmouse.sh
PROC_FILE=/proc/modules
GREP=/bin/grep
MODPROBE=/sbin/modprobe
MODULE_NAME=rtmouse
DEP_MODULE_NAME=mcp320x
[ -f $PROC_FILE ] || exit 0
[ -x $GREP ] || exit 0
[ -x $MODPROBE ] || exit 0
RES=`$GREP $MODULE_NAME $PROC_FILE`

install_rtmouse(){
  if [ "$RES" = "" ]; then
    $MODPROBE $MODULE_NAME
    echo "Module Install $MODULE_NAME"
  else
    echo "Module '$MODULE_NAME' is already installed"
  fi
}
remove_rtmouse(){
  if [ "$RES" = "" ]; then
    echo "Module '$MODULE_NAME' isn't installed yet."
  else
    $MODPROBE -r $MODULE_NAME
    $MODPROBE -r $DEP_MODULE_NAME
    echo "Module '$MODULE_NAME' is rmoved."
  fi
}

case "$1" in
  start)
  install_rtmouse
  sleep 1
  /bin/chmod a+rw /dev/rt*
  ;;
  stop)
  remove_rtmouse
  ;;
  restart)
  remove_rtmouse
  sleep 1
  RES=`$GREP $MODULE_NAME $PROC_FILE`
  install_rtmouse
  ;;
  status)
    if [ "$RES" = "" ]; then
      echo "Module '$MODULE_NAME' isn't installed yet."
      exit 0
    else
      echo "Module '$MODULE_NAME' is already installed"
      exit 0
    fi
  ;;
  *)
    echo "Usage: $SCRIPTNAME {start|stop|restart|status}" >&2
    exit 3
esac
exit 0
