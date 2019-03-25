/**
 * main.cpp
 * @author yuta seya
 * @date 2019 3.25 
*/

#include <iostream>
#include <unistd.h>

#include "Switch.h"
#include "Led.h"
#include "Buzzer.h"
#include "Motor.h"


using namespace std;

int main()
{

  cout << "checkPushSw" << endl;

  Switch *sw = new Switch();
  Led *led = new Led();
  Buzzer *buzzer = new Buzzer();

  bool sw0,sw1,sw2;
  int mode_count = 0;

  while( 1 ){
    sw0 = sw->get0();
    sw1 = sw->get1();
    sw2 = sw->get2();

    if ( sw0 ){
      mode_count++;
      if ( mode_count > 15 ) mode_count = 0;
      led->illuminate( mode_count );
      buzzer->on( A, 300 );
    }

    if ( sw1 ){
      mode_count--;
      if ( mode_count < 0 ) mode_count = 15;
      led->illuminate( mode_count );
      buzzer->on( C, 300 );
    }

    if ( sw2 ) {
      led->illuminate( 0x00 );
      break;
    }
    
  }

  return 0;
}