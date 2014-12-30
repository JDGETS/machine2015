#include <Servo.h>
#include <Hardware.h>
#include <Vehicle.h>
#include <AllStates.h>
using namespace Vehicle;

void setup() 
{
  Serial.begin(9600);
  
  Vehicle::Setup();
}
 
void loop() 
{
  Serial.print(States::AlignmentReedSwitchZoneLancement::TARGET, BIN);
  Serial.print(" ");
  Serial.println(reedswitches.GetValue(), BIN);
  delay(50);
} 

