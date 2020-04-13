#!/bin/bash

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
