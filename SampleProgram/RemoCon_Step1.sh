#!/bin/bash -xv

# Copyright 2016-2020 RT Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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


