#include <Servo.h>
#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include "I2Cdev.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define motor1_pin 3
#define motor1_pin_servo_lib 3

Servo myservo; // Create Servo object to control the servo


//FUNCTION declarations
void gyro_PID_loop();
void update_Gyro_values();


//GYRO PID LOOP VARIABLES
Adafruit_MPU6050 mpu;
int gyro_update_loop_timer = 0;
int gyro_PID_loop_timer = 0;
float gyro_degrees = 0.00;

float find_mean = 0.00;
float find_mean_counter = 0.00;
/*
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data
char tmp_str[7]; // temporary variable used in convert function
unsigned long gyro_prev_time = 0;
unsigned long gyro_current_time = 0;
float gyro_degrees = 0.00;

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}
*/



//MOTOR Variables
int current_time = 0;
int p_in = 255/2;
int micro_p_in_test = 1.5 * 1000;




//Logic variables
int end_flag = 0; // low is to end on our side, high is to end on their side












void setup() {
  //pinMode(motor1_pin, OUTPUT);
  //analogWrite(motor1_pin, p_in);
  
  Serial.begin(9600);

  //Servo set up
  myservo.attach(motor1_pin_servo_lib); 
  myservo.writeMicroseconds(micro_p_in_test);



  //GYRO setup
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  //Serial.println("");
  delay(100);
  gyro_update_loop_timer = millis();
  gyro_PID_loop_timer = millis();

  /*
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  */
}











void loop() {
  // Use Serial to test inputs to motors and speeds/calibrate motors
  if(Serial.available()){
    if(Serial.read() == '1'){
      p_in++;
      micro_p_in_test++;
    }else{
      p_in--;
      micro_p_in_test--;
    }

    Serial.print(p_in);  
    Serial.print("  ");
    Serial.print(micro_p_in_test);
  }		

  //UPDATE_GYRO_VALUES
  current_time = millis();
  if(current_time - gyro_update_loop_timer > .000000000000001){ //100hz
      update_Gyro_values(); 
      gyro_update_loop_timer = current_time;
    }

  //GYRO_PID_LOOP
  current_time = millis();
  if(current_time - gyro_PID_loop_timer > 20){ //50hz
    gyro_PID_loop();
    gyro_PID_loop_timer = current_time;
  }

  /*
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  
  // print out data
  Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
  Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y));
  Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z));
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  Serial.print(" | tmp = "); Serial.print(temperature/340.00+36.53);
  Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x));
  Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y));
  Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z));
  Serial.println();
*/













  myservo.writeMicroseconds(micro_p_in_test);
  
  
  
  

}






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


}



