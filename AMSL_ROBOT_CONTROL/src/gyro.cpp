#include "definitions.hpp"


void update_Gyro_values(){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    //Serial.print("Temperature:");
    //Serial.print(temp.temperature);
    //Serial.print("x-acceleration:");
    //Serial.print(a.acceleration.x);
    //Serial.print("\ty-acceleration:");
    //Serial.print(a.acceleration.y);
    //Serial.print("\tz-acceleration:");
    //Serial.print(a.acceleration.z);
    //Serial.print("\tx-gyro:");
    //Serial.print(g.gyro.x);
    //Serial.print("\ty-gyro:");
    //Serial.print(g.gyro.y);
    Serial.print("\tz-gyro:");
    Serial.print(g.gyro.z,6);


    //////////////////////////////
    //This CODE IS TO ONLY FIND THE STANDING ERROR OF GYRO
    find_mean += g.gyro.z;
    find_mean_counter+= 1.00;
    float standing_gyro_mean = find_mean/find_mean_counter;// 0.005418 is a good number 0.005863 0.005873 0.005894 0.005581
    Serial.print("  \tstanding_gyro_mean:");
    Serial.print(standing_gyro_mean, 6);
    /////////////////////////


    gyro_degrees += ((float)current_time - (float)gyro_update_loop_timer) / 1000.00 * (g.gyro.z-0.005581) * 180.00/PI;
    Serial.print("  \tgyro degrees:");
    Serial.println(gyro_degrees);
}






void gyro_PID_loop(){
    //ERROR
    gyro_PID_error = gyro_degrees; // If this is positive then it is leaning left and left wheel speed up 

    //PROPORTIONAL
    gyro_PID_P = gyro_PID_error * gyro_PID_KP;

    //INTEGRAL
    gyro_PID_I += (gyro_PID_error) * (float)(current_time - gyro_PID_loop_timer) * gyro_PID_KI;
    //if(gyro_PID_I > 255) line_follow_PID_I = 255;
    //if(gyro_PID_I < -255) line_follow_PID_I = -255;

    //DERIVATIVE
    gyro_PID_D = ((gyro_PID_error - gyro_PID_error_prev) / (float)(current_time - gyro_PID_loop_timer)) * gyro_PID_KD;
    gyro_PID_error_prev = gyro_PID_error;

    // SUMMATION
    gyro_PID_out = gyro_PID_P + gyro_PID_I + gyro_PID_D;
    //if(gyro_PID_out > twinky_max) gyro_PID_out = twinky_max;
    //if(gyro_PID_out < twinky_min) gyro_PID_out = twinky_min;




    if(gyro_foward_flag == true){
        if(gyro_PID_out >= 0){
            twinky_one_speed = twinky_max + gyro_PID_out/2;
            twinky_two_speed = twinky_max - gyro_PID_out/2;

        }else{
            twinky_one_speed = twinky_max - (-1.00 * gyro_PID_out)/2; 
            twinky_two_speed = twinky_max + (-1.00 * gyro_PID_out)/2; 
            
        }
    }else{
        if(gyro_PID_out >= 0){
            twinky_one_speed = -1.00*(twinky_max - gyro_PID_out/2); 
            twinky_two_speed = -1.00*(twinky_max + gyro_PID_out/2); 

        }else{
            twinky_one_speed = -1.00*(twinky_max + (-1.00*gyro_PID_out)/2);
            twinky_two_speed = -1.00*(twinky_max - (-1.00*gyro_PID_out)/2);

        }
    }

}









