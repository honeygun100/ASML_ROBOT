#include "definitions.hpp"

// Color sensor
States checkColor(States state){
  if(state.am_i_left == 1){
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
      if ((period > yellowBound_FL[0]) && (period < yellowBound_FL[1])){ // if yellow
        state.curr_high_bound = yellowBound_FL[1];
        state.curr_low_bound = yellowBound_FL[0];
        state.curr = 1;
        state.yellow = 1;
        state.blue = 0;
        state.black = 0;
      
      }else if ((period > blueBound_FL[0]) && (period < blueBound_FL[1])){ // if blue
        state.curr_high_bound = blueBound_FL[1];
        state.curr_low_bound = blueBound_FL[0];
        state.curr = 1;
        state.yellow = 0;
        state.blue = 1;
        state.black = 0;
      
      }else if ((period > blackBound_FL[0]) && (period < blackBound_FL[1])){ // if black
        state.curr_high_bound = blackBound_FL[1];
        state.curr_low_bound = blackBound_FL[0];
        state.yellow = 0;
        state.blue = 0;
        state.black = 1;
      }
    }
  }else{
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
      if ((period > yellowBound_RB[0]) && (period < yellowBound_RB[1])){ // if yellow
        state.curr_high_bound = yellowBound_RB[1];
        state.curr_low_bound = yellowBound_RB[0];
        state.curr = 1;
        state.yellow = 1;
        state.blue = 0;
        state.black = 0;
      
      }else if ((period > blueBound_RB[0]) && (period < blueBound_RB[1])){ // if blue
        state.curr_high_bound = blueBound_RB[1];
        state.curr_low_bound = blueBound_RB[0];
        state.curr = 1;
        state.yellow = 0;
        state.blue = 1;
        state.black = 0;
      
      }else if ((period > blackBound_RB[0]) && (period < blackBound_RB[1])){ // if black
        state.curr_high_bound = blackBound_RB[1];
        state.curr_low_bound = blackBound_RB[0];
        state.yellow = 0;
        state.blue = 0;
        state.black = 1;
      }
    }
  }
  return state;
}

// get which color we are on

void getColor(){
  stateRB.period = pulseIn(sensorRB, LOW);
  stateRB = checkColor(stateRB);

  stateFL.period = pulseIn(sensorFL, LOW);
  stateFL = checkColor(stateFL);
}


