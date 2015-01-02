#include "UltrasonicSensor.h"

#define TRIGGER_PIN 46
#define OUTPUT_PIN 44
#define MAX_DISTANCE 19

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

