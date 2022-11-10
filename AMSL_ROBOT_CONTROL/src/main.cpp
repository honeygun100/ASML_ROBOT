#include "definitions.hpp"


#define motor1_pin_servo_lib 3
#define motor2_pin_servo_lib 5
#define motor3_pin_servo_lib 6
#define motor4_pin_servo_lib 9




//GYRO PID LOOP VARIABLES
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_MPU6050 mpu;
unsigned long gyro_update_loop_timer = 0;
unsigned long gyro_PID_loop_timer = 0;
float find_mean = 0.00;
float find_mean_counter = 0.00;

float gyro_degrees = 0.00; // extern                                     
float gyro_PID_error = 0.00; // extern
float gyro_PID_error_prev = 0.00; // extern
float gyro_PID_P = 0.00; // extern
float gyro_PID_I = 0.00; // extern
float gyro_PID_D = 0.00; // extern
float gyro_KP_divider = 1.00; // extern
float gyro_PID_KP = 1.00/gyro_KP_divider; // extern
float gyro_PID_KI = 0.00; // extern
float gyro_PID_KD = 6.00; // extern
float gyro_PID_out = 0.00; // extern
bool gyro_foward_flag = true; // extern


//MOTOR Variables
Servo myservo1; // Create Servo object to control the servo
Servo myservo2;
Servo myservo3;
Servo myservo4;
unsigned long current_time = 0;
int p_in = 255/2;
int micros_p_in = 1.5 * 1000;
int micro_p_in_max = 1.5 * 1000; // to move wheel forward
int micro_p_in_min = 1.5 * 1000; // to move wheel backward


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
    if(incomingCharacter == '1'){
      p_in++;
      micros_p_in++;
    }else if(incomingCharacter == '2'){
      p_in--;
      micros_p_in--;
    }

    Serial.print(p_in);
    Serial.print("  ");
    Serial.println(micros_p_in);
  }		

  
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
  motor_move();








  //analogWrite(motor1_pin_servo_lib, p_in);
  myservo1.writeMicroseconds(micros_p_in);
  
  
  
  

}






