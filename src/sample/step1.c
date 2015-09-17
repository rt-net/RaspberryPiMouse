#include "fcntl.h"

void main(void){
int led0,led1,led2,led3;

led0 = open("/dev/rtled0" , O_WRONLY  );
led1 = open("/dev/rtled1" , O_WRONLY  );
led2 = open("/dev/rtled2" , O_WRONLY  );
led3 = open("/dev/rtled3" , O_WRONLY  );

while(1){
	write (led0,"1",2);
        write (led1,"1",2);
        write (led2,"1",2);
        write (led3,"1",2);
	usleep(500 * 1000);
	write (led0,"0",2);
        write (led1,"0",2);
        write (led2,"0",2);
        write (led3,"0",2);
	usleep(500 * 1000);
}

close(led0);
close(led1);
close(led2);
close(led3);
} 
