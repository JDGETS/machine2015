#include "UltrasonicSensor.h"

#define TRIGGER_PIN 52
#define OUTPUT_PIN 53
#define MAX_DISTANCE 15

UltrasonicSensor sensor(TRIGGER_PIN, OUTPUT_PIN, MAX_DISTANCE);

void setup() 
{
  Serial.begin(9600);
}
 
void loop() 
{
  Serial.print(sensor.ReadInput());
  Serial.println("");
  delay(100);
} 

