#include "OpticalSensor.h"

#define SENSOR_PIN A9

OpticalSensor sensor(SENSOR_PIN);

void setup() 
{
  Serial.begin(9600);
  
  Serial.println("Setting up the optical sensor...");
  //pinMode(SENSOR_PIN, INPUT);
  sensor.Setup();
  sensor.Invert();
}
 
void loop() 
{
  Serial.print(analogRead(SENSOR_PIN));
  Serial.print(" ");
  Serial.print(sensor.ReadInput());
  Serial.println("");
  delay(100);
} 

