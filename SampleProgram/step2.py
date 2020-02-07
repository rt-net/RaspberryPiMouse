#!/usr/bin/python
import sys

class _Getch:
    def __init__(self):
        import tty,sys

    def __call__(self):
        import sys,tty,termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd,termios.TCSADRAIN,old_settings)
        return ch

dict= {"0":"0","a":"261","w":"277","s":"293","e":"311","d":"329","f":"349","t":"370","g":"392","y":"415","h":"440","u":"446","j":"493","k":"523"}

while 1:
    getch = _Getch()
    d = getch()
    if d == "c":
        break
    if d in dict:
        with open('/dev/rtbuzzer0','w') as f:
            f.write(dict[d])

