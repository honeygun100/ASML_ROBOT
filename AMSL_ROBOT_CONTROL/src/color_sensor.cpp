#include "definitions.hpp"

// Color sensor
States checkColor(States state){

  int period = state.period;

  // Keep checking if color switches
  if (state.curr == 1){
    if ((period < state.curr_low_bound) || (period > state.curr_high_bound)){ // check if within bounds
      state.opp = 1;
      state.curr = 0;
    }
    else{
      state.opp = 0;
    }
  }


  // Pick new bounds if opposite is set to 1
  if ((state.opp == 1)){
    if ((period > yellowBound[0]) && (period < yellowBound[1])){ // if yellow
      state.curr_high_bound = yellowBound[1];
      state.curr_low_bound = yellowBound[0];
      state.curr = 1;
      state.yellow = 1;
      state.blue = 0;
      state.black = 0;
    
    }else if ((period > blueBound[0]) && (period < blueBound[1])){ // if blue
      state.curr_high_bound = blueBound[1];
      state.curr_low_bound = blueBound[0];
      state.curr = 1;
      state.yellow = 0;
      state.blue = 1;
      state.black = 0;
    
    }else if ((period > blackBound[0]) && (period < blackBound[1])){ // if black
      state.curr_high_bound = blackBound[1];
      state.curr_low_bound = blackBound[0];
      state.yellow = 0;
      state.blue = 0;
      state.black = 1;
    }
  }
  return state;
}

// get which color we are on

void getColor(){
  stateFL.period = pulseIn(sensorFL, LOW);
  stateFL = checkColor(stateFL);
  stateRB.period = pulseIn(sensorRB, LOW);
  stateRB = checkColor(stateRB);
}


