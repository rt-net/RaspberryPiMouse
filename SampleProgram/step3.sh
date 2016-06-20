#!/bin/bash

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



