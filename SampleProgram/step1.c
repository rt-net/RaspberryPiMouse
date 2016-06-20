#include "fcntl.h"

void main(void)
{
    int led[4];
    int i;

    led[0]=open("/dev/rtled0",O_WRONLY);
    led[1]=open("/dev/rtled1",O_WRONLY);
    led[2]=open("/dev/rtled2",O_WRONLY);
    led[3]=open("/dev/rtled3",O_WRONLY);

    while(1)
    {
        for(i=0;i<4;i++)
        {
            write(led[i],"1",1);
        }
        usleep(500*1000);
        for(i=0;i<4;i++)
        {
            write(led[i],"0",1);
        }
        usleep(500*1000);
    }
    for(i=0;i<4;i++)
    {
        close(led[i]);
    }
}
