#include "definitions.hpp"

//const int BUTTON_PIN = 7; // the number of the pushbutton pin


//MIGHT NOT NEED SERVO LIBRARY, IF WE USE ANALOG WRITE WE WOULD NEED H-BRIDGE TO REVERSE DIRECTION


void release()
{
  int releasepos = 170;
  lever_servo.write(releasepos);
  //delay(5000);
}

void clamp()
{
  int clamppos = 5;
  lever_servo.write(clamppos);
  //delay(5000);
}


