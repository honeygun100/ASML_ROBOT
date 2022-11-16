#include "definitions.hpp"


#define motor1_pin_servo_lib 3
#define motor2_pin_servo_lib 5
#define motor3_pin_servo_lib 6
#define motor4_pin_servo_lib 9







//MOTOR Variables
Servo myservo1; // Create Servo object to control the servo
Servo myservo2;
Servo myservo3;
Servo myservo4;
unsigned long current_time = 0;
int p_in = 255/2;
int micros_p_in = 1.5 * 1000;
int no_move_p_in = 1.5 * 1000; // no movement
int whlpair1_micro_p_in_max = 1.628 * 1000; // to move wheel forward wheel 1 and 3
int whlpair2_micro_p_in_max = 1.628 * 1000; // to move wheel backward wheel 1 and 3
int whlpair1_micro_p_in_min = 1.373 * 1000; // to move wheel forward wheel 2 and 4
int whlpair2_micro_p_in_min = 1.373 * 1000; // to move wheel backward wheel 2 and 4

//GYRO PID LOOP VARIABLES
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_MPU6050 mpu;
unsigned long gyro_update_loop_timer = 0;
unsigned long gyro_PID_loop_timer = 0;
float find_mean = 0.00;
float find_mean_counter = 0.00;

float gyro_degrees = 0.00; // extern     
float gyro_degrees2 = 0.00; // extern
float gyro_PID_error = 0.00; // extern
float gyro_PID_error_prev = 0.00; // extern

float gyro_PID_P = 0.00; // extern
float gyro_PID_I = 0.00; // extern
float gyro_PID_D = 0.00; // extern

float gyro_KP_divider = 1.22; // extern 1.00 is ok, try 1.22 for now, larger gyro_KP_divider makes gyro_PID_KP smaller which makes gyro_PID_P smaller, so more angle to slow down
float gyro_PID_KP = whlpair1_micro_p_in_max/gyro_KP_divider; // extern
float gyro_PID_KI = 0.00; // extern
float gyro_PID_KD = 0.00; // extern
float gyro_PID_out = 0.00; // extern

bool gyro_foward_flag = true; // extern NOT USED

//Logic variables
int end_flag = 0; // low is to end on our side, high is to end on their side
Direciton current_direction;





void setup() {
  //pinMode(motor1_pin_servo_lib, OUTPUT);
  //analogWrite(motor1_pin_servo_lib, p_in);

  Serial.begin(9600);

  //Servo set up
  myservo1.attach(motor1_pin_servo_lib);
  myservo2.attach(motor2_pin_servo_lib);
  myservo3.attach(motor3_pin_servo_lib);
  myservo4.attach(motor4_pin_servo_lib);
  myservo1.writeMicroseconds(micros_p_in);
  myservo2.writeMicroseconds(micros_p_in);
  myservo3.writeMicroseconds(micros_p_in);
  myservo4.writeMicroseconds(micros_p_in);
  
  
  
  


  //GYRO setup
  // Try to initialize MPU_6050!
  /*
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
    delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ); //maybe 21 instead of 10
  //Serial.println("");
  delay(100);
  */


  
  // Try to initialize BNO005!
  if(!bno.begin()){
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  




  gyro_update_loop_timer = millis();
  gyro_PID_loop_timer = millis();



  current_direction = forward;
}











void loop() {
  // Use Serial to test inputs to motors and speeds/calibrate motors
  if(Serial.available()){
    char incomingCharacter = Serial.read();
    if(incomingCharacter == '1'){ // case one and two are for test code at bottom of void loop()
      p_in++;
      micros_p_in++;
      Serial.print(p_in);
      Serial.print("  ");
      Serial.println(micros_p_in);
    }else if(incomingCharacter == '2'){
      p_in--;
      micros_p_in--;
      Serial.print(p_in);
      Serial.print("  ");
      Serial.println(micros_p_in);
    }else if(incomingCharacter == '3'){
      gyro_KP_divider += .01;
      Serial.print("kp_divider is ");
      Serial.print(gyro_KP_divider);
      gyro_PID_KP = whlpair1_micro_p_in_max/gyro_KP_divider; // maybe replace micro_p_in_max with 1
      Serial.print("\tkp is ");
      Serial.println(gyro_PID_KP);
    }else if(incomingCharacter == '4'){
      gyro_KP_divider -= .01;
      Serial.print("kp_divider is ");
      Serial.print(gyro_KP_divider);
      gyro_PID_KP = whlpair1_micro_p_in_max/gyro_KP_divider; // maybe replace micro_p_in_max with 1
      Serial.print("\tkp is ");
      Serial.println(gyro_PID_KP);
    }else if(incomingCharacter == '5'){
      gyro_PID_KI += .5;
      Serial.print("gyro_PID_KI is ");
      Serial.println(gyro_PID_KI);
    }else if(incomingCharacter == '6'){
      gyro_PID_KI -= .5;
      Serial.print("gyro_PID_KI is ");
      Serial.println(gyro_PID_KI);
    }else if(incomingCharacter == '7'){
      gyro_PID_KD += .5;
      Serial.print("gyro_PID_KD is ");
      Serial.println(gyro_PID_KD);
    }else if(incomingCharacter == '8'){
      gyro_PID_KD -= .5;
      Serial.print("gyro_PID_KD is ");
      Serial.println(gyro_PID_KD);

    }




    
  }		

  

  current_direction = forward;
  choose_direction_and_move();
  





  //motor test code
  //analogWrite(motor1_pin_servo_lib, p_in);
  //myservo1.writeMicroseconds(micros_p_in); // when holding from correct position: left wheel -> 1.628 * 1000 for motor one to move forward coutner-clockwise
                                           //                                                   1.373 * 1000 for motor one to move backward clockwise
  //myservo2.writeMicroseconds(micros_p_in); // front wheel
  //myservo3.writeMicroseconds(micros_p_in); // right wheel
  //myservo4.writeMicroseconds(micros_p_in); // back wheel
  Serial.println("");

}






