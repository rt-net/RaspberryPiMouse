#!/bin/bash

# Copyright 2020 RT Corporation
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

MOTOR_EN=/dev/rtmotoren0
MOTOR_R=/dev/rtmotor_raw_r0
MOTOR_L=/dev/rtmotor_raw_l0
COUNTER_R=/dev/rtcounter_r0
COUNTER_L=/dev/rtcounter_l0

SPEED=400

function echo_counter () {
    SECONDS=0
    while [ $SECONDS -lt "$1" ]
    do
    echo "count_l:$(cat $COUNTER_L), count_r:$(cat $COUNTER_R)"
    done
}

function reset_counters_and_motors () {
    echo 0 | tee $MOTOR_L $MOTOR_R > /dev/null
    echo "Reset counter"
    echo 0 | tee $COUNTER_L $COUNTER_R > /dev/null
    echo "count_l:$(cat $COUNTER_L), count_r:$(cat $COUNTER_R)"
}

reset_counters_and_motors

echo "Motor On"
echo 1 > $MOTOR_EN

echo "Rotate left motor"
sleep 0.5
echo $SPEED > $MOTOR_L
echo_counter 2
reset_counters_and_motors

echo "Rotate right motor"
sleep 0.5
echo $SPEED > $MOTOR_R
echo_counter 2
reset_counters_and_motors

echo "Move forward"
sleep 0.5
echo $SPEED | tee $MOTOR_L $MOTOR_R > /dev/null
echo_counter 2
reset_counters_and_motors

echo "Move backward"
sleep 0.5
echo -$SPEED | tee $MOTOR_L $MOTOR_R > /dev/null
echo_counter 2
reset_counters_and_motors

echo "Motor Off"
echo 0 > $MOTOR_EN
