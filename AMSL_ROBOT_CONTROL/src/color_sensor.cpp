#include "definitions.hpp"
///////////// Init /////////////
void init_func()
{
 
    // Color sensor
    sei();
    PCICR = 0b00000001;  // enable PCINT0 as interrupt
    PCMSK0 = 0b00000001; // enable PCINT0
    TCCR1B = 0b00000001; //  timer - is 1
}
 
///////// COLOR sensor ////////
// get which color we are on
int getColor()
{
  PCMSK0 = 0b00000001;
  _delay_ms(10);
  PCMSK0 = 0b00000000;
  period = period / 16;
 
  // Keep checking if color switches
  if ((curr = 1))
  {
 
    if ((period < low_bound) && (period > high_bound)) // check if within bounds
    {
      opp = 1;
    }
// Pick new bounds if opposite is set to 1
    if ((opp = 1))
    {
 
      if ((period > yellowBound[0]) && (period < yellowBound[1])) // if yellow
      {
        high_bound = yellowBound[1];
        low_bound = yellowBound[0];
        curr = 1;
        opp = 0;
        yellow = 1;
        blue = 0;
        black = 0;
      }
      else if ((period > blueBound[0]) && (period < blueBound[1])) // if blue
      {
        high_bound = blueBound[1];
        low_bound = blueBound[0];
        curr = 1;
        opp = 0;
        yellow = 0;
        blue = 1;
        black = 0;
      }
      else if ((period > blackBound[0]) && (period < blackBound[1])) // if black
      {
        high_bound = blackBound[1];
        low_bound = blackBound[0];
        opp = 0;
        yellow = 0;
        blue = 0;
        black = 1;
      }
    }
  }
  return period;
}