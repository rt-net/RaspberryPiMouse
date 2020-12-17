#!/usr/bin/python

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

import time
import sys

filename_motoren = "/dev/rtmotoren0"
filename_motor_r = "/dev/rtmotor_raw_r0"
filename_motor_l = "/dev/rtmotor_raw_l0"
filename_motor = "/dev/rtmotor0"

SPEED = "400"
TIME_MS = "500"

def motor_drive(freq_l="0", freq_r="0"):
    with open(filename_motor_l, 'w') as f:
        f.write(freq_l)
    with open(filename_motor_r, 'w') as f:
        f.write(freq_r)

print("Motor On")
with open(filename_motoren, 'w') as f:
    f.write("1")
time.sleep(0.5)

print("Rotate counter-clockwise")
motor_drive("-"+SPEED, SPEED)
time.sleep(0.5)

motor_drive("0", "0")
time.sleep(0.5)

print("Rotate clockwise")
motor_drive(SPEED, "-"+SPEED)
time.sleep(0.5)

motor_drive("0", "0")
time.sleep(0.5)

print("Rotate clockwise")
with open(filename_motor, 'w') as f:
    f.write(SPEED+" -"+SPEED+" "+TIME_MS)

time.sleep(0.5)

print("Rotate counter-clockwise")
with open(filename_motor, 'w') as f:
    f.write("-"+SPEED+" "+SPEED+" "+TIME_MS)

print("Motor Off")
with open(filename_motoren, 'w') as f:
    f.write("0")
