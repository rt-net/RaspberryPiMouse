#!/bin/bash -xv

trap 2

sudo sixad -start &


set +xv
while true ; do
if grep -q 0 /dev/rtswitch0 ; then
   sleep 0.1
   while grep -q 0 /dev/rtswitch0 ; do
       sleep 0.1
   done
   sudo python /home/pi/bbb.py
   break
fi
if grep -q 0 /dev/rtswitch1 ; then
   sleep 0.1
   while grep -q 0 /dev/rtswitch1 ; do
       sleep 0.1
   done
   break;
fi
if grep -q 0 /dev/rtswitch2 ; then
   sleep 0.1
   while grep -q 0 /dev/rtswitch2 ; do
       sleep 0.1
   done
   break
fi

echo 1 | tee /dev/rtled?
sleep 0.5
echo 0 | tee /dev/rtled?
sleep 0.5
done
set -xv


