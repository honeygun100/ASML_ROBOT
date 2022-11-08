#ifndef DEFINITIONS_HPP
#define DEFINITIONS_HPP

#include <Servo.h>
#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include "I2Cdev.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


//FUNCTION declarations
void gyro_PID_loop();
void update_Gyro_values();


//GYRO PID LOOP VARIABLES
extern Adafruit_MPU6050 mpu;
extern int gyro_update_loop_timer;
extern int gyro_PID_loop_timer;
extern float gyro_degrees;
extern float find_mean;
extern float find_mean_counter;

extern float gyro_degrees;                                  
extern float gyro_PID_error; 
extern float gyro_PID_error_prev; 
extern float gyro_PID_P; 
extern float gyro_PID_I;
extern float gyro_PID_D; 
extern float gyro_KP_divider; 
extern float gyro_PID_KP;
extern float gyro_PID_KI; 
extern float gyro_PID_KD; 
extern float gyro_PID_out; 
extern bool gyro_foward_flag; 


//MOTOR Variables
extern Servo myservo1; // Create Servo object to control the servo
extern Servo myservo2;
extern Servo myservo3;
extern Servo myservo4;
extern int current_time;
extern int p_in;
extern int micros_p_in;



//Logic variables
extern int end_flag; // low is to end on our side, high is to end on their side




#endif