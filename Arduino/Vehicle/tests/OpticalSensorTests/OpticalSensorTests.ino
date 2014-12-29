#include "OpticalSensor.h"

OpticalSensor sensor(A9);

void setup() 
{
  Serial.begin(9600);
  
  Serial.println("Setting up the optical sensor...");
  sensor.Setup();
}
 
void loop() 
{
  Serial.println(sensor.ReadInput());
} 

