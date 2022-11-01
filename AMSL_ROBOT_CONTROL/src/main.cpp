#include <Servo.h>
#include <Arduino.h>
#define motor1_pin 3
#define motor1_pin_servo_lib 9
Servo myservo; // Create Servo object to control the servo

int curr = 0;
int prev = 0;

void setup() {
  //pinMode(motor1_pin, OUTPUT);
  //analogWrite(motor1_pin, 255/2);
  Serial.begin(9600);
  myservo.attach(motor1_pin_servo_lib); 
}

void loop() {
  // put your main code here, to run repeatedly:
 
  myservo.write(0);			// Rotate to 0 degrees
  delay(1500);
  myservo.write(90);			// Rotate to 90 degrees
  delay(1500);
  myservo.write(0);			// Rotate to 180 degrees
  delay(1500);








}







/*
void setup() {
myServo.attach(3); // Servo is connected to digital pin 9
}

*/










