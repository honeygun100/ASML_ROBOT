#include <Servo.h>
#include <Arduino.h>

#define motor1_pin 3
#define motor1_pin_servo_lib 3

Servo myservo; // Create Servo object to control the servo


//FUNCTION declarations
void gyro_PID_loop();



//GYRO PID LOOP VARIABLES
int gyro_loop_timer = 0;


//MOTOR Variables
int current_time = 0;
int p_in = 255/2;
int micro_p_in_test = 1.5 * 1000;





int end_flag = 0; // low is to end on our side, high is to end on their side












void setup() {
  //pinMode(motor1_pin, OUTPUT);
  //analogWrite(motor1_pin, p_in);
  
  Serial.begin(9600);
  
  myservo.attach(motor1_pin_servo_lib); 
  myservo.writeMicroseconds(micro_p_in_test);
}











void loop() {
  // put your main code here, to run repeatedly:
  
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

  current_time = millis();
  if(current_time - gyro_loop_timer > 20){ //50Khz
    gyro_PID_loop(); 

  }


  myservo.writeMicroseconds(micro_p_in_test);
  
  /*
  for (int pos = 1000; pos <= 2000; pos += 10) { // goes from 0 degrees to 180 degrees
    myservo.writeMicroseconds(pos);
    delay(25);                               // waits 25ms for the servo to reach the position
  }
  delay(1000);
  for (int pos = 2000; pos >= 1000; pos -= 10) { // goes from 180 degrees to 0 degrees
    myservo.writeMicroseconds(pos);
    delay(25);                               // waits 25ms for the servo to reach the position
  }
  delay(1000);
  */
  
  

}








void gyro_PID_loop(){

}