#include <Arduino.h>
#define motor1_pin 3



void setup() {
  pinMode(motor1_pin, OUTPUT);
  analogWrite(motor1_pin, 254);

}

void loop() {
  // put your main code here, to run repeatedly:
}