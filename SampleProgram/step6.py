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
filename_count_r = "/dev/rtcounter_r0"
filename_count_l = "/dev/rtcounter_l0"

SPEED = "400"


def motor_drive(freq_l="0", freq_r="0"):
    with open(filename_motor_l, 'w') as f:
        f.write(freq_l)
    with open(filename_motor_r, 'w') as f:
        f.write(freq_r)


def print_counter(timeout=2.0):
    start = time.time()
    while time.time() - start < timeout:
        count_r = count_l = 0
        with open(filename_count_l, 'r') as f:
            count_l = f.read().strip()
        with open(filename_count_r, 'r') as f:
            count_r = f.read().strip()
        print("count_l:" + count_l + ", count_r:" + count_r)


def reset_counters_and_motors():
    motor_drive("0", "0")
    print("Reset counter")
    with open(filename_count_l, 'w') as f:
        f.write("0")
    with open(filename_count_r, 'w') as f:
        f.write("0")
    print_counter(0)


print("Motor On")
with open(filename_motoren, 'w') as f:
    f.write("1")

print("Rotate left motor")
time.sleep(0.5)
motor_drive(SPEED, "0")
print_counter(2.0)
reset_counters_and_motors()

print("Rotate right motor")
time.sleep(0.5)
motor_drive("0", SPEED)
print_counter(2.0)
reset_counters_and_motors()

print("Move forward")
time.sleep(0.5)
motor_drive(SPEED, SPEED)
print_counter(2.0)
reset_counters_and_motors()

print("Move backward")
time.sleep(0.5)
motor_drive("-"+SPEED, "-"+SPEED)
print_counter(2.0)
reset_counters_and_motors()

print("Motor Off")
with open(filename_motoren, 'w') as f:
    f.write("0")
