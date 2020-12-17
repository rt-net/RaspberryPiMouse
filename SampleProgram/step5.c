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
#include <unistd.h>

int main(void) {
    int buff_size = 256;
    FILE *fp;

    char buff[buff_size];
    while (1) {
        if ((fp = fopen("/dev/rtlightsensor0", "r")) != NULL) {
            while (fgets(buff, buff_size, fp) != NULL) {
                printf("%s", buff);
            }
        }
        fclose(fp);
        usleep(500 * 1000);
    }
    return 0;
}
