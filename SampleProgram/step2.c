#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int _Getch(void) {
    int ch;
    system("stty -echo -icanon min 1 time 0");
    ch = getchar();
    system("stty echo icanon");
    return ch;
}

int main(void) {
    int buzzer = open("/dev/rtbuzzer0", O_WRONLY);
    int c = 1;

    while (c) {
        switch (_Getch()) {
            case '0':  // off
                write(buzzer, "0", 2);
                break;
            case 'a':  // do
                write(buzzer, "261", 4);
                break;
            case 'w':  // do#
                write(buzzer, "277", 4);
                break;
            case 's':  // re
                write(buzzer, "293", 4);
                break;
            case 'e':  // re#
                write(buzzer, "311", 4);
                break;
            case 'd':  // mi
                write(buzzer, "329", 4);
                break;
            case 'f':  // fa
                write(buzzer, "349", 4);
                break;
            case 't':  // fa#
                write(buzzer, "370", 4);
                break;
            case 'g':  // so
                write(buzzer, "392", 4);
                break;
            case 'y':  // so#
                write(buzzer, "415", 4);
                break;
            case 'h':  // ra
                write(buzzer, "440", 4);
                break;
            case 'u':  // ra#
                write(buzzer, "446", 4);
                break;
            case 'j':  // shi
                write(buzzer, "493", 4);
                break;
            case 'k':  // do2
                write(buzzer, "523", 4);
                break;
            case 'c':
                write(buzzer, "0", 2);
                c = 0;
                break;
        }
    }
    close(buzzer);
    return 0;
    }
    