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
MOTOR=/dev/rtmotor0

SPEED=400
TIME_MS=500

echo "Motor On"
echo 1 > $MOTOR_EN
sleep 0.5

echo "Rotate counter-clockwise"
echo -$SPEED > $MOTOR_L
echo $SPEED > $MOTOR_R
sleep 0.5

echo 0 > $MOTOR_L
echo 0 > $MOTOR_R
sleep 0.5

echo "Rotate clockwise"
echo $SPEED > $MOTOR_L
echo -$SPEED > $MOTOR_R
sleep 0.5

echo 0 > $MOTOR_L
echo 0 > $MOTOR_R
sleep 0.5

echo "Rotate clockwise"
echo $SPEED -$SPEED $TIME_MS > $MOTOR
sleep 0.5

echo "Rotate counter-clockwise"
echo -$SPEED $SPEED $TIME_MS > $MOTOR

echo "Motor Off"
echo 0 > $MOTOR_EN
