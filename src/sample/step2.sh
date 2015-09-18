#!/bin/bash 

while read -N 1 b ; do
        awk -v "t=$b" '$3==t{print $2}' SCALE > /dev/rtbuzzer0
done



