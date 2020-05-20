#include "fcntl.h"

char get_SW0(void) {
    char buf[2];
    int SW0;
    SW0 = open("/dev/rtswitch0", O_RDONLY);
    read(SW0, buf, 2);
    close(SW0);
    return buf[0];
}

char get_SW1(void) {
    char buf[2];
    int SW1;
    SW1 = open("/dev/rtswitch1", O_RDONLY);
    read(SW1, buf, 2);
    close(SW1);
    return buf[0];
}

char get_SW2(void) {
    char buf[2];
    int SW2;
    SW2 = open("/dev/rtswitch2", O_RDONLY);
    read(SW2, buf, 2);
    close(SW2);
    return buf[0];
}

void main(void) {
    int state0, state1, state2;
    int LED0, LED1, LED2, LED3;

    LED0 = open("/dev/rtled0", O_WRONLY);
    LED1 = open("/dev/rtled1", O_WRONLY);
    LED2 = open("/dev/rtled2", O_WRONLY);
    LED3 = open("/dev/rtled3", O_WRONLY);

    state0 = state1 = state2 = 0;

    while (1) {
        if (get_SW0() == '0') {
            usleep(10000);
            while (get_SW0() == '0')
                ;
            usleep(10000);
            state0 = (state0 + 1) & 0x01;
            if (state0 == 0) {
                write(LED3, "0", 1);
            } else {
                write(LED3, "1", 1);
            }
        }
        if (get_SW1() == '0') {
            usleep(10000);
            while (get_SW1() == '0')
                ;
            usleep(10000);
            state1 = (state1 + 1) & 0x01;
            if (state1 == 0) {
                write(LED2, "0", 1);
                write(LED1, "0", 1);
            } else {
                write(LED2, "1", 1);
                write(LED1, "1", 1);
            }
        }
        if (get_SW2() == '0') {
            usleep(10000);
            while (get_SW2() == '0')
                ;
            usleep(10000);
            state2 = (state2 + 1) & 0x01;
            if (state2 == 0) {
                write(LED0, "0", 1);
            } else {
                write(LED0, "1", 1);
            }
        }
    }
}
