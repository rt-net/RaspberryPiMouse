#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

int main(void) {
    int motoren = open("/dev/rtmotoren0", O_WRONLY);
    int motor_l = open("/dev/rtmotor_raw_l0", O_WRONLY);
    int motor_r = open("/dev/rtmotor_raw_r0", O_WRONLY);
    int motor = open("/dev/rtmotor0", O_WRONLY);

    printf("Motor On\n");
    write(motoren, "1", 1);
    usleep(500 * 1000);

    printf("Rotate counter-clockwise\n");
    write(motor_l, "-400", 5);
    write(motor_r, "400", 5);
    usleep(500 * 1000);

    write(motor_l, "0", 5);
    write(motor_r, "0", 5);
    usleep(500 * 1000);

    printf("Rotate clockwise\n");
    write(motor_l, "400", 5);
    write(motor_r, "-400", 5);
    usleep(500 * 1000);

    write(motor_l, "0", 5);
    write(motor_r, "0", 5);
    usleep(500 * 1000);

    printf("Rotate clockwise\n");
    write(motor, "400 -400 500", 15);
    usleep(500 * 1000);

    printf("Rotate counter-clockwise\n");
    write(motor, "-400 400 500", 15);

    printf("Motor Off\n");
    write(motoren, "0", 1);

    close(motoren);
    close(motor_l);
    close(motor_r);
    close(motor);
    return 0;
}
