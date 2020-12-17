#!/bin/bash

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

state0=0
state1=0
state2=0

while true ; do
	if grep -q 0 /dev/rtswitch0 ; then
		sleep 0.1
		while grep -q 0 /dev/rtswitch0 ; do
			sleep 0.1
		done
		state0=`expr $state0 + 1`
		state0=`expr $state0 % 2`	
		echo $state0 > /dev/rtled3
	fi
        if grep -q 0 /dev/rtswitch1 ; then
                sleep 0.1
                while grep -q 0 /dev/rtswitch1 ; do
                        sleep 0.1
                done
                state1=`expr $state1 + 1`
                state1=`expr $state1 % 2`
                echo $state1 > /dev/rtled2
		echo $state1 > /dev/rtled1
        fi
        if grep -q 0 /dev/rtswitch2 ; then
                sleep 0.1
                while grep -q 0 /dev/rtswitch2 ; do
                        sleep 0.1
                done
                state2=`expr $state2 + 1`
                state2=`expr $state2 % 2`
                echo $state2 > /dev/rtled0
        fi
done



