#include "UltrasonicSensor.h"

#define TRIGGER_PIN 2
#define OUTPUT_PIN 2
#define MAX_DISTANCE 100

UltrasonicSensor sensor(TRIGGER_PIN, OUTPUT_PIN, MAX_DISTANCE);

void setup() 
{
  Serial.begin(9600);
}
 
void loop() 
{
  Serial.print(sensor.ReadInput());
  Serial.println("");
} 

