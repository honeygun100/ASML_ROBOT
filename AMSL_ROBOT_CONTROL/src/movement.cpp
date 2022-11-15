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

    current_time = millis();
    BNO005_update_Gyro_values(); 
    //MPU_6050_update_Gyro_values();

    //GYRO_PID_LOOP
    if(current_time - gyro_PID_loop_timer > 20){ //50hz
        gyro_PID_loop();
        gyro_PID_loop_timer = current_time;
    }

    //go to wheel and write pwm wave depending on direciton
    //motor_move();







}








// SOME OF THESE ...max and ...min NEED TO CHANGE DEPENDING ON HOW MOTORS ARE MOUNTED
void motor_move(){
    if(current_direction == forward){
        if(gyro_PID_out >= 0){ 
            // this would mean the robot is leaning left, speed left wheel up and right wheel down
            // we need motor 1 to go counter-clockwise and motor 3 to go clockwise
            control_move_forward(whlpair1_micro_p_in_max + gyro_PID_out, whlpair1_micro_p_in_min + gyro_PID_out);

        }else{
            // this would mean the robot is leaning right, speed right wheel up and left wheel down
            // we need motor 1 to go counter-clockwise and motor 3 to go clockwise
            gyro_PID_out = gyro_PID_out * (-1.00);
            control_move_forward(whlpair1_micro_p_in_max - gyro_PID_out, whlpair1_micro_p_in_min - gyro_PID_out);
        }

    }else if(current_direction == backward){
        if(gyro_PID_out >= 0){ 
            // this would mean the robot is leaning left, speed right wheel up and left wheel down
            // we need motor 1 to go clockwise and motor 3 to go counter-clockwise
            control_move_backward(whlpair1_micro_p_in_min + gyro_PID_out, whlpair1_micro_p_in_max + gyro_PID_out);
        }else{
            // this would mean the robot is leaning right, speed right wheel down and left wheel up
            // we need motor 1 to go clockwise and motor 3 to go counter-clockwise
            gyro_PID_out = gyro_PID_out * (-1.00);
            control_move_backward(whlpair1_micro_p_in_min - gyro_PID_out, whlpair1_micro_p_in_max - gyro_PID_out);
        }


    }else if(current_direction == left){ 
        if(gyro_PID_out >= 0){ 
            // this would mean the robot is leaning left, speed front wheel down and back wheel up
            // we need motor 2 to go clockwise and motor 4 to go counter-clockwise
            control_move_left(whlpair2_micro_p_in_min + gyro_PID_out, whlpair2_micro_p_in_max + gyro_PID_out);
        }else{
            // this would mean the robot is leaning right, speed front wheel up and back wheel down
            // we need motor 2 to go clockwise and motor 4 to go counter-clockwise
            gyro_PID_out = gyro_PID_out * (-1.00);
            control_move_left(whlpair2_micro_p_in_min - gyro_PID_out, whlpair2_micro_p_in_max - gyro_PID_out);
        }

    }else if(current_direction == right){
        if(gyro_PID_out >= 0){ 
            // this would mean the robot is leaning left, speed front wheel up and back wheel down
            // we need motor 2 to go counter-clockwise and motor 4 to go clockwise
            control_move_right(whlpair2_micro_p_in_max + gyro_PID_out, whlpair2_micro_p_in_min + gyro_PID_out);
        }else{
            // this would mean the robot is leaning right, speed front wheel down and back wheel up
            // we need motor 2 to go counter-clockwise and motor 4 to go clockwise
            gyro_PID_out = gyro_PID_out * (-1.00);
            control_move_right(whlpair2_micro_p_in_max - gyro_PID_out, whlpair2_micro_p_in_min - gyro_PID_out);
        }

    }else{
        Serial.println("Out of control, direction not defined");
    }






}






void control_move_forward(float leftwheel_p_in, float rightwheel_p_in){
    //turn off not needed wheels
    myservo2.writeMicroseconds(micros_p_in); // front wheel
    myservo4.writeMicroseconds(micros_p_in); // back wheel
    //turn on appropriate wheels
    myservo1.writeMicroseconds(leftwheel_p_in); // left wheel
    myservo3.writeMicroseconds(rightwheel_p_in); // right wheel
}

void control_move_backward(float leftwheel_p_in, float rightwheel_p_in){
    //turn off not needed wheels
    myservo2.writeMicroseconds(micros_p_in); // front wheel
    myservo4.writeMicroseconds(micros_p_in); // back wheel
    //turn on appropriate wheels
    myservo1.writeMicroseconds(leftwheel_p_in); // left wheel
    myservo3.writeMicroseconds(rightwheel_p_in); // right wheel
}

void control_move_left(float frontwheel_p_in, float backwheel_p_in){
    //turn off not needed wheels
    myservo1.writeMicroseconds(micros_p_in); // left wheel
    myservo3.writeMicroseconds(micros_p_in); // right wheel
    //turn on appropriate wheels
    myservo2.writeMicroseconds(frontwheel_p_in); // front wheel
    myservo4.writeMicroseconds(backwheel_p_in); // back wheel
}

void control_move_right(float frontwheel_p_in, float backwheel_p_in){
    //turn off not needed wheels
    myservo1.writeMicroseconds(micros_p_in); // left wheel
    myservo3.writeMicroseconds(micros_p_in); // right wheel
    //turn on appropriate wheels
    myservo2.writeMicroseconds(frontwheel_p_in); // front wheel
    myservo4.writeMicroseconds(backwheel_p_in); // back wheel
}









// leftwheel_p_in and rightwheel_p_in will be whlpair1_micro_p_in_max and whlpair1_micro_p_in_min depending on orientation of motor mounting
void move_forward(float leftwheel_p_in, float rightwheel_p_in){
    //turn off not needed wheels
    myservo2.writeMicroseconds(micros_p_in); // front wheel
    myservo4.writeMicroseconds(micros_p_in); // back wheel
    //turn on appropriate wheels
    myservo1.writeMicroseconds(leftwheel_p_in); // left wheel
    myservo3.writeMicroseconds(rightwheel_p_in); // right wheel
}

void move_backward(float leftwheel_p_in, float rightwheel_p_in){
    //turn off not needed wheels
    myservo2.writeMicroseconds(micros_p_in); // front wheel
    myservo4.writeMicroseconds(micros_p_in); // back wheel
    //turn on appropriate wheels
    myservo1.writeMicroseconds(leftwheel_p_in); // left wheel
    myservo3.writeMicroseconds(rightwheel_p_in); // right wheel
}

void move_left(float frontwheel_p_in, float backwheel_p_in){
    //turn off not needed wheels
    myservo1.writeMicroseconds(micros_p_in); // left wheel
    myservo3.writeMicroseconds(micros_p_in); // right wheel
    //turn on appropriate wheels
    myservo2.writeMicroseconds(frontwheel_p_in); // front wheel
    myservo4.writeMicroseconds(backwheel_p_in); // back wheel
}

void move_right(float frontwheel_p_in, float backwheel_p_in){
    //turn off not needed wheels
    myservo1.writeMicroseconds(micros_p_in); // left wheel
    myservo3.writeMicroseconds(micros_p_in); // right wheel
    //turn on appropriate wheels
    myservo2.writeMicroseconds(frontwheel_p_in); // front wheel
    myservo4.writeMicroseconds(backwheel_p_in); // back wheel
}
