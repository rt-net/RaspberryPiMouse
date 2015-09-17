#!/bin/bash 
while true 
do
echo 1 | tee /dev/rtled? #点灯
sleep 0.5		#0.5秒待ち
echo 0 | tee /dev/rtled? #消灯
sleep 0.5		#0.5秒待ち
done


