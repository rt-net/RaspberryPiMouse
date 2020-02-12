#!/usr/bin/python
import sys
import tty
import termios


class _Getch:
    def __init__(self):
        pass

    def __call__(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


freq_dict = {
        "0": "0", "a": "261", "w": "277", "s": "293", "e": "311", "d": "329",
        "f": "349", "t": "370", "g": "392", "y": "415", "h": "440", "u": "446",
        "j": "493", "k": "523"}

while 1:
    getch = _Getch()
    key = getch()
    if key == "c":
        break
    if key in freq_dict:
        with open('/dev/rtbuzzer0', 'w') as f:
            f.write(freq_dict[key])
