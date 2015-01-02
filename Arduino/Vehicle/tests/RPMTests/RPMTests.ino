#include <Servo.h>
#include "Hardware.h"
#include "Vehicle.h"
#include "AllStates.h"

States::Shooting state;
States::Hole first, second;
void setup() 
{
  Serial.begin(9600);
  
  Vehicle::Setup();

  Serial.println("Starting RPMTests.");
}
 
void loop() 
{
  Serial.println(state.ReadRPM( 4 ));
} 

