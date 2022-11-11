#include "definitions.hpp"



void choose_direction_and_move(){
    //UPDATE_GYRO_VALUES
    /*
    current_time = millis();
    if(current_time - gyro_update_loop_timer > .000000000000001){ //100hz
        MPU_6050_update_Gyro_values(); 
        gyro_update_loop_timer = current_time;
        }
    */

    
    /*
    current_time = millis();
    if(current_time - gyro_update_loop_timer > 1000){ //100hz
        BNO005_update_Gyro_values(); 
        gyro_update_loop_timer = current_time;
    }
    */

    
    BNO005_update_Gyro_values(); 
    //MPU_6050_update_Gyro_values();
    
    current_time = millis();
    //GYRO_PID_LOOP
    if(current_time - gyro_PID_loop_timer > 20){ //50hz
        gyro_PID_loop();
        gyro_PID_loop_timer = current_time;
    }

    //go to wheel and write pwm wave depending on direciton
    motor_move();







}









void motor_move(){



}












