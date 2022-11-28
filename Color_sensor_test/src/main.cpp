#include <Arduino.h>

#define sensorOut 8

int frequency = 0;

void setup() {
  pinMode(sensorOut, INPUT);
  
  // Setting frequency-scaling to 20%
  
  
  Serial.begin(9600);
}

void loop() {
  // Setting red filtered photodiodes to be read
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  // Printing the value on the serial monitor
  Serial.print("R= ");//printing name
  Serial.print(frequency);//printing RED color frequency
  Serial.print("  ");

  // Setting Green filtered photodiodes to be read
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  // Printing the value on the serial monitor
  Serial.print("G= ");//printing name
  Serial.print(frequency);//printing RED color frequency
  Serial.print("  ");

  // Setting Blue filtered photodiodes to be read
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  // Printing the value on the serial monitor
  Serial.print("B= ");//printing name
  Serial.print(frequency);//printing RED color frequency
  Serial.println("  ");
}