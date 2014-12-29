#include <Servo.h>
#include "Hardware.h"
#include "Vehicle.h"
#include "AllStates.h"

void setup() 
{
  Serial.begin(9600);

  States::Racing state;
  Serial.println("Starting Racing.");
  state.Execute();
}
 
void loop() 
{
  
} 

