#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

void main(void)
{
    int buff_size = 256;
    FILE *fp;

    char buff[buff_size];
    while(1){
        if ((fp = fopen("/dev/rtlightsensor0", "r")) != NULL){
            while(fgets(buff, buff_size, fp) != NULL){
                printf("%s", buff);
            }
        }
        fclose(fp);
        usleep(500*1000);
    }
}
