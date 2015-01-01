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
  Serial.print(reedswitches.GetValue(), BIN);
  Serial.print(" (target=");
  Serial.print(States::AlignmentReedSwitchZoneLancement::TARGET, BIN);
  Serial.print(", ");
  for(int i = 3; i >= 0; i--){
    Serial.print(reedswitches.ARRAY[i]._TopReedSwitch.IsActive());
    Serial.print(reedswitches.ARRAY[i]._BottomReedSwitch.IsActive());
    Serial.print(", ");
  }
  Serial.println(")");
  delay(50);
} 

