#ifndef DEFINITIONS_HPP
#define DEFINITIONS_HPP

#include <Servo.h>
#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include "I2Cdev.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
//#include "Adafruit_TCS34725.h"

#define motor1_pin_servo_lib 3
#define motor2_pin_servo_lib 5
#define motor3_pin_servo_lib 6
#define motor4_pin_servo_lib 10
#define game_start_input_pin 12
#define game_start_output_pin 13
#define up_an_where_pin_input 2
#define up_and_where_pin_output 7

#define sensorFL 8
#define sensorRB 9

typedef struct States{
  int curr_low_bound = 0, curr_high_bound = 0, curr = 0, opp = 1, yellow = 0, blue = 0, black = 0, period = 0;
}States;



//Logic variables
extern int end_flag; // low is to end on our side, high is to end on their side
enum Direciton {forward, backward, left, right};
extern Direciton current_direction;

//FUNCTION declarations
void gyro_PID_loop();
void MPU_6050_update_Gyro_values();
void displayCalStatus();
void BNO005_update_Gyro_values();
void BNO005_get_standing_error();
void motor_move();
void control_move_time_delay(unsigned long x);
void choose_direction_and_move();
void control_move_forward(float leftwheel_p_in, float rightwheel_p_in);
void control_move_backward(float leftwheel_p_in, float rightwheel_p_in);
void control_move_left(float frontwheel_p_in, float backwheel_p_in);
void control_move_right(float frontwheel_p_in, float backwheel_p_in);

void move_forward(float leftwheel_p_in, float rightwheel_p_in);
void move_backward(float leftwheel_p_in, float rightwheel_p_in);
void move_left(float frontwheel_p_in, float backwheel_p_in);
void move_right(float frontwheel_p_in, float backwheel_p_in);
void stop_all_wheels();
float bound_check(float checker);

void getColor();
States checkColor(States state);

void release();
void clamp();

//GYRO PID LOOP VARIABLES
extern Adafruit_BNO055 bno;
extern Adafruit_MPU6050 mpu;
extern unsigned long gyro_update_loop_timer;
extern unsigned long gyro_PID_loop_timer;
extern float gyro_degrees;
extern float find_mean;
extern float find_mean_counter;
extern float gyro_read_offset;

extern float gyro_degrees;      
extern float gyro_degrees2;                         
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
extern float offset1;
extern float gyro_desired;
extern int print_gyro_values;


//MOTOR Variables
extern Servo myservo1; // Create Servo object to control the servo
extern Servo myservo2;
extern Servo myservo3;
extern Servo myservo4;
extern unsigned long current_time;
extern int p_in;
extern int micros_p_in;
extern int no_move_p_in; // no movement
extern float whlpair1_micro_p_in_max; // to move wheel forward
extern float whlpair2_micro_p_in_max; // to move wheel backward
extern float whlpair1_micro_p_in_min; // to move wheel forward
extern float whlpair2_micro_p_in_min; // to move wheel backward
extern float offset2;

//COLOR SENSOR Variables
extern int curr_low_bound;
extern int curr_high_bound;
extern int yellowBound[2];  // 9
extern int blueBound[2];   // 67
extern int blackBound[2]; // 121
extern States stateFL;
extern States stateRB;
enum home_options {blue, yellow};
extern home_options home;
//extern Adafruit_TCS34725 tcs;
//extern uint16_t r, g, b, c, colorTemp, lux;

//Lever variables
extern Servo lever_servo;
extern unsigned long release_timer;
extern unsigned long clamp_timer;
extern int lever_action_state;

#endif