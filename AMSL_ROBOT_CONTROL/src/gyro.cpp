#include "definitions.hpp"


void MPU_6050_update_Gyro_values(){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyro_degrees += ((float)current_time - (float)gyro_update_loop_timer) / 1000.00 * (g.gyro.z-0.012760) * 180.00/PI;
    gyro_update_loop_timer = current_time;
    
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
    //Serial.print("\tz-gyro:");
    //Serial.print(g.gyro.z,6);


    //////////////////////////////
    //This CODE IS TO ONLY FIND THE STANDING ERROR OF GYRO
    find_mean += g.gyro.z;
    find_mean_counter+= 1.00;
    float standing_gyro_mean = find_mean/find_mean_counter;// 0.005418 is a good number 0.013898
    Serial.print("  \tstanding_gyro_mean:");
    Serial.print(standing_gyro_mean, 6);
    /////////////////////////


    
    Serial.print("  \tgyro degrees:");
    Serial.println(gyro_degrees);
}



void BNO005_update_Gyro_values(){
    /*
    sensors_event_t event; 
    bno.getEvent(&event);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); // VECTOR_EULER   CIRCLES BACK TO 360
    
    //Display the floating point data 
    Serial.print("X: ");
    Serial.print(euler.x());
    Serial.print(" Y: ");
    Serial.print(euler.y());
    Serial.print(" Z: ");
    Serial.print(euler.z());
    Serial.print("");
    

    
    //////////////////////////////
    //This CODE IS TO ONLY FIND THE STANDING ERROR OF GYRO
    //float sample = euler.z();
    //find_mean += sample;
    //find_mean_counter+= 1.00;
    //loat standing_gyro_mean = find_mean/find_mean_counter;// 0.005418 is a good number 0.013898
    //Serial.print("  \tstanding_gyro_mean:");
    //Serial.print(standing_gyro_mean, 6);
    /////////////////////////

    
    gyro_degrees += ((float)(current_time - gyro_update_loop_timer)) / 1000.00 * ((float)(euler.z())) * 180.00/((float)PI);
    Serial.print("  \ttime diff:");
    Serial.print((float)(current_time - gyro_update_loop_timer));
    gyro_update_loop_timer = current_time;
    Serial.print("  \tgyro degrees:");
    Serial.println(gyro_degrees);
    */

    

















    /*
    // Display the floating point data  CIRCLES BACK TO 360
    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);
    Serial.println("");
    */
    

    imu::Quaternion quat = bno.getQuat();
    /*
    //Display the quat data
    Serial.print("qW: ");
    Serial.print(quat.w(), 4);
    Serial.print(" qX: ");
    Serial.print(quat.y(), 4);
    Serial.print(" qY: ");
    Serial.print(quat.x(), 4);
    Serial.print(" qZ: ");
    Serial.print(quat.z(), 4);
    Serial.println("");
    
    gyro_degrees = quat.z();
    */

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
            myservo1.writeMicroseconds(micros_p_in - gyro_PID_out);

            //twinky_one_speed = twinky_max + gyro_PID_out/2;
            //twinky_two_speed = twinky_max - gyro_PID_out/2;

        }else{
            //twinky_one_speed = twinky_max - (-1.00 * gyro_PID_out)/2; 
            //twinky_two_speed = twinky_max + (-1.00 * gyro_PID_out)/2; 
            
        }
    }else{
        if(gyro_PID_out >= 0){
            //twinky_one_speed = -1.00*(twinky_max - gyro_PID_out/2); 
            //twinky_two_speed = -1.00*(twinky_max + gyro_PID_out/2); 

        }else{
            //twinky_one_speed = -1.00*(twinky_max + (-1.00*gyro_PID_out)/2);
            //twinky_two_speed = -1.00*(twinky_max - (-1.00*gyro_PID_out)/2);

        }
    }

}









