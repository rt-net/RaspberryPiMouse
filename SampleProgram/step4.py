#!/usr/bin/python
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
