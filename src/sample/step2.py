#!/usr/bin/python
import sys
buzzer = open('/dev/rtbuzzer0','a')

class _Getch:
    """Gets a single character from standard input.  Does not echo to the screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()

dict= { "o":"0", "a":"261", "w":"277", "s":"293","e":"311","d":"329","f":"349","t":"370","g":"392","y":"415","h":"440","u":"446","j":"493","k":"523"}

while 1:
	getch = _Getch()
	d = getch()
	if d == "c" :
		break
	if d in dict :
		buzzer.write(dict[d])
	buzzer.flush()
