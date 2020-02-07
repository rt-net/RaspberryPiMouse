#!/usr/bin/python
import time

state0=state1=state2=0

while 1 :
    with open("/dev/rtswitch0","r") as f:
        if f.readline() == "0\n" :
            time.sleep(0.1)
            while 1 :
                with open("/dev/rtswitch0","r") as f:
                    if f.readline() != "0\n" :
                        break
            time.sleep(0.1)
            state0 = (state0 + 1 ) & 1
            with open("/dev/rtled3","w") as f:
                f.write(str(state0))
    with open("/dev/rtswitch1","r") as f:
        if f.readline() == "0\n" :
            time.sleep(0.1)
            while 1 :
                with open("/dev/rtswitch1","r") as f:
                    if f.readline() != "0\n" :
                        break
            time.sleep(0.1)
            state1 = (state1 + 1 ) & 1
            with open("/dev/rtled2","w") as f:
                f.write(str(state1))
            with open("/dev/rtled1","w") as f:
                f.write(str(state1))
    with open("/dev/rtswitch2","r") as f:
        if f.readline() == "0\n" :
            time.sleep(0.1)
            while 1 :
                with open("/dev/rtswitch2","r") as f:
                     if f.readline() != "0\n" :
                            break
            time.sleep(0.1)
            state2 = (state2 + 1 ) & 1
            with open("/dev/rtled0","w") as f:
                f.write(str(state2))

