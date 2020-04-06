#!/usr/bin/python
import time

while 1:
    with open("/dev/rtlightsensor0", 'r') as f:
        print(f.read().strip())
    time.sleep(0.5)

