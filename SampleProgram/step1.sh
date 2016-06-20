#!/bin/bash

while true
do
    echo 1 | tee /dev/rtled?
    sleep 0.5
    echo 0 | tee /dev/rtled?
    sleep 0.5
done

