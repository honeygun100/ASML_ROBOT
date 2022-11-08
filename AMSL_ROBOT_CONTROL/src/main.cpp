#include "definitions.hpp"


#define motor1_pin_servo_lib 3
#define motor2_pin_servo_lib 5
#define motor3_pin_servo_lib 6
#define motor4_pin_servo_lib 9


//GYRO PID LOOP VARIABLES
Adafruit_MPU6050 mpu;
int gyro_update_loop_timer = 0;
int gyro_PID_loop_timer = 0;
float gyro_degrees = 0.00;
float find_mean = 0.00;
float find_mean_counter = 0.00;

float gyro_degrees = 0.00; // extern                                      //FOR GYRO PID control
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
int current_time = 0;
int p_in = 255/2;
int micros_p_in = 1.5 * 1000;
int micro_p_in_max = 1.5 * 1000; // to move wheel forward
int micro_p_in_min = 1.5 * 1000; // to move wheel backward


//Logic variables
int end_flag = 0; // low is to end on our side, high is to end on their side









void setup() {
  //pinMode(motor1_pin_servo_lib, OUTPUT);
  //analogWrite(motor1_pin_servo_lib, p_in);
  
  Serial.begin(9600);

  //Servo set up
  myservo1.attach(motor1_pin_servo_lib); 
  myservo1.writeMicroseconds(micros_p_in);



  //GYRO setup
  // Try to initialize!
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
  gyro_update_loop_timer = millis();
  gyro_PID_loop_timer = millis();
  */

}











void loop() {
  // Use Serial to test inputs to motors and speeds/calibrate motors
  if(Serial.available()){
    char incomingCharacter = Serial.read();
    if(incomingCharacter == '1'){
      p_in++;
      micros_p_in++;
    }else{
      p_in--;
      micros_p_in--;
    }

    Serial.print(p_in);  
    Serial.print("  ");
    Serial.println(micros_p_in);
  }		

  /*
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
  */

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








  //analogWrite(motor1_pin_servo_lib, p_in);
  myservo1.writeMicroseconds(micros_p_in);
  
  
  
  

}






