#include <Servo.h>
#include "Hardware.h"
#include "Vehicle.h"
#include "AllStates.h"
  
void setup() 
{
  Serial.begin(9600);
  
  Vehicle::Setup();

  States::Charging charging;
  Serial.println("Starting Charging.");
  charging.Execute();
 
  States::NewShooting shooting;
  Serial.println("Starting Shooting.");
  shooting.Execute();
}
 
void loop() 
{
  
} 

