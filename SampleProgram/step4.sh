#!/bin/bash

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
