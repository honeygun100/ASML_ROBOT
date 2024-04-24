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
    find_mean_counter += 1.00;
    float standing_gyro_mean = find_mean/find_mean_counter;// 0.005418 is a good number 0.013898
    Serial.print("  \tstanding_gyro_mean:");
    Serial.print(standing_gyro_mean, 6);
    /////////////////////////


    
    Serial.print("  \tgyro degrees:");
    Serial.println(gyro_degrees);
}


void displayCalStatus(){
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

void BNO005_get_standing_error(){
   
    int temp_millis = millis();
    float new_qW = 0.00;

    while(millis() - temp_millis < 500){
        imu::Quaternion quat = bno.getQuat();    
        new_qW = -1.00*(quat.w()-1);
        if(quat.z() >= 0){ // we are turning left
            new_qW = new_qW * 1.00;
        }else{ // we are turning right
            new_qW = new_qW * -1.00;
        }
    }

    gyro_read_offset = new_qW;
    if(print_gyro_values == 1){
        Serial.print("initial standing error gyro_read_offset is: ");
        Serial.print(gyro_read_offset,8);
    }
}

void BNO005_update_Gyro_values(){
    
    imu::Quaternion quat = bno.getQuat();
    //Display the quat data
    
    float new_qW = -1.00*(quat.w()-1);
    if(quat.z() >= 0){ // we are turning left
        new_qW = new_qW * 1.00;
    }else{ // we are turning right
        new_qW = new_qW * -1.00;
    }
    
    if(print_gyro_values == 1){
        Serial.print("qW: ");
        Serial.print(new_qW, 8);
        Serial.print(" qX: ");
        Serial.print(quat.y(), 8);
        Serial.print(" qY: ");
        Serial.print(quat.x(), 8);
        Serial.print(" qZ: ");
        Serial.print(quat.z(), 8);
    }


    gyro_degrees = new_qW - gyro_read_offset; // previously .00012207
    if(print_gyro_values == 1){
        Serial.print("   gyro_degrees: ");
        Serial.print(gyro_degrees, 8);
    }
    //gyro_degrees = quat.z();
}


void gyro_PID_loop(){
    
    //ERROR
    gyro_PID_error = gyro_degrees - gyro_desired; // If this is positive then it is leaning left and left wheel speed up

    //PROPORTIONAL
    gyro_PID_P = gyro_PID_error * gyro_PID_KP; // error is a decimal and KP is max vaue.

    //INTEGRAL
    gyro_PID_I += (gyro_PID_error) * (float)(current_time - gyro_PID_loop_timer) * gyro_PID_KI;
    ///if(gyro_PID_I > whlpair1_micro_p_in_max) gyro_PID_I = whlpair1_micro_p_in_max;
    //if(gyro_PID_I < whlpair1_micro_p_in_min) gyro_PID_I = whlpair1_micro_p_in_min;

    //DERIVATIVE
    gyro_PID_D = ((gyro_PID_error - gyro_PID_error_prev) / (float)(current_time - gyro_PID_loop_timer)) * gyro_PID_KD;
    gyro_PID_error_prev = gyro_PID_error;

    // SUMMATION
    gyro_PID_out = gyro_PID_P + gyro_PID_I + gyro_PID_D; 
    //if(gyro_PID_out > whlpair1_micro_p_in_max) gyro_PID_out = whlpair1_micro_p_in_max;
    //if(gyro_PID_out < whlpair1_micro_p_in_min) gyro_PID_out = whlpair1_micro_p_in_min;
    if(print_gyro_values == 1){
        Serial.print("  gyro_PID_out: ");
        Serial.print(gyro_PID_out);
        Serial.println();
    }


}









