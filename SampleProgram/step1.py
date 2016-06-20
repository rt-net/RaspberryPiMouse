#!/usr/bin/python
import time
import sys

files = ["/dev/rtled0","/dev/rtled1","/dev/rtled2","/dev/rtled3"]

while 1:
	for filename in files:
		with open(filename,'w') as f:
			f.write("1")
	time.sleep(0.5)
	for filename in files:
		with open(filename,'w') as f:
			f.write("0")
	time.sleep(0.5)

