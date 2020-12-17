// Copyright 2020 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define FILE_MOTOREN "/dev/rtmotoren0"
#define FILE_MOTOR_L "/dev/rtmotor_raw_l0"
#define FILE_MOTOR_R "/dev/rtmotor_raw_r0"
#define FILE_COUNT_L "/dev/rtcounter_l0"
#define FILE_COUNT_R "/dev/rtcounter_r0"
#define BUFF_SIZE 256

void motor_drive(char *freq_l, char *freq_r) {
    FILE *motor_l, *motor_r;
    if ((motor_l = fopen(FILE_MOTOR_L, "w")) != NULL &&
        (motor_r = fopen(FILE_MOTOR_R, "w")) != NULL) {
        fputs(freq_l, motor_l);
        fputs(freq_r, motor_r);
    }
    fclose(motor_l);
    fclose(motor_r);
}

void delete_newline(char *str) {
    char *p;

    if ((p = strchr(str, '\n')) != NULL) {
        *p = '\0';
    }
}

void print_counter(const int timeout) {
    FILE *count_l, *count_r;
    char buff_l[BUFF_SIZE];
    char buff_r[BUFF_SIZE];

    time_t start = time(NULL);
    int elapsed_time = 0;

    while (elapsed_time < timeout) {
        if ((count_l = fopen(FILE_COUNT_L, "r")) != NULL &&
            (count_r = fopen(FILE_COUNT_R, "r")) != NULL) {
            while (fgets(buff_l, BUFF_SIZE, count_l) != NULL) {
            }
            while (fgets(buff_r, BUFF_SIZE, count_r) != NULL) {
            }
            delete_newline(buff_l);
            delete_newline(buff_r);
            printf("count_l:%s, count_r:%s\n", buff_l, buff_r);
        }
        fclose(count_l);
        fclose(count_r);

        elapsed_time = (int)(time(NULL) - start);
    }
}

void reset_counters_and_motors(void) {
    FILE *count_l, *count_r;

    motor_drive("0", "0");

    printf("Reset counter\n");
    if ((count_l = fopen(FILE_COUNT_L, "w")) != NULL &&
        (count_r = fopen(FILE_COUNT_R, "w")) != NULL) {
        fputs("0", count_l);
        fputs("0", count_r);
    }
    fclose(count_l);
    fclose(count_r);
}

int main(void) {
    int motoren = open("/dev/rtmotoren0", O_WRONLY);
    // int motor_l = open("/dev/rtmotor_raw_l0",O_WRONLY);

    printf("Motor On\n");
    write(motoren, "1", 1);

    printf("Rotate left motor\n");
    usleep(500 * 1000);
    motor_drive("400", "0");
    print_counter(2);
    reset_counters_and_motors();

    printf("Rotate right motor\n");
    usleep(500 * 1000);
    motor_drive("0", "400");
    print_counter(2);
    reset_counters_and_motors();

    printf("Move forward\n");
    usleep(500 * 1000);
    motor_drive("400", "400");
    print_counter(2);
    reset_counters_and_motors();

    printf("Move backward\n");
    usleep(500 * 1000);
    motor_drive("-400", "-400");
    print_counter(2);
    reset_counters_and_motors();

    printf("Motor Off\n");
    write(motoren, "0", 1);

    close(motoren);
    return 0;
}
