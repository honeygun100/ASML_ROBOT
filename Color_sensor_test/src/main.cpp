#include <Arduino.h>

#include <stdio.h>

 

#define sensorFL 8

#define sensorRB 9

 

// Color sensor

int curr_low_bound = 0;

int curr_high_bound = 0;

 

int yellowBound[2] = {0, 20};  // 55
int blueBound[2] = {20, 81};   // 450
int blackBound[2] = {81, 1000}; // 800

 

struct States

{

  int curr_low_bound, curr_high_bound, curr, opp, yellow, blue, black, period;

} stateFL, stateRB;

 

void setup()

{

  pinMode(sensorFL, INPUT);

  pinMode(sensorRB, INPUT);

 

  // Setting frequency-scaling to 20%

 

  Serial.begin(9600);

}

 

///////// COLOR sensor ////////

 

void checkColor(States state)

{

 

  int period = state.period;

  // Keep checking if color switches

  if (state.curr == 1)

  {

    if ((period < state.curr_low_bound) && (period > state.curr_high_bound)) // check if within bounds

    {

      state.opp = 1;

    }

 

    // Pick new bounds if opposite is set to 1

    if ((state.opp == 1))

    {

 

      if ((period > yellowBound[0]) && (period < yellowBound[1])) // if yellow

      {

        state.curr_high_bound = yellowBound[1];

        state.curr_low_bound = yellowBound[0];

        state.curr = 1;

        state.opp = 0;

        state.yellow = 1;

        state.blue = 0;

        state.black = 0;

      }

      else if ((period > blueBound[0]) && (period < blueBound[1])) // if blue

      {

        state.curr_high_bound = blueBound[1];

        state.curr_low_bound = blueBound[0];

        state.curr = 1;

        state.opp = 0;

        state.yellow = 0;

        state.blue = 1;

        state.black = 0;

      }

      else if ((period > blackBound[0]) && (period < blackBound[1])) // if black

      {

        state.curr_high_bound = blackBound[1];

        state.curr_low_bound = blackBound[0];

        state.opp = 0;

        state.yellow = 0;

        state.blue = 0;

        state.black = 1;

      }

    }

  }

}

 

// get which color we are on

void getColor()

{

  stateFL.period = pulseIn(sensorFL, LOW);

  checkColor(stateFL);

 

  stateRB.period = pulseIn(sensorRB, LOW);

  checkColor(stateRB);

}

 

void loop()

{

  getColor();

 

  while (1)

  {

    getColor();

    Serial.print("FL:   | ");

    Serial.print("Curr: ");

    Serial.print(stateFL.curr);

    Serial.print(" | ");

    Serial.print("Opp: ");

    Serial.print(stateFL.opp);

    Serial.print(" | ");

    Serial.print("Blue: ");

    Serial.print(stateFL.blue);

    Serial.print(" | ");

    Serial.print("Yellow: ");

    Serial.print(stateFL.yellow);

    Serial.print(" | ");

    Serial.print("Black: ");

    Serial.print(stateFL.black);

    Serial.print(" | ");

    Serial.print("Period: ");

    Serial.print(stateFL.period);

    Serial.print(" | ");

 

    Serial.print("------- RB:   | ");

    Serial.print("Curr: ");

    Serial.print(stateRB.curr);

    Serial.print(" | ");

    Serial.print("Opp: ");

    Serial.print(stateRB.opp);

    Serial.print(" | ");

    Serial.print("Blue: ");

    Serial.print(stateRB.blue);

    Serial.print(" | ");

    Serial.print("Yellow: ");

    Serial.print(stateRB.yellow);

    Serial.print(" | ");

    Serial.print("Black: ");

    Serial.print(stateRB.black);

    Serial.print(" | ");

    Serial.print("Period: ");

    Serial.print(stateRB.period);

    Serial.print(" | ");

    Serial.println();

  }

}