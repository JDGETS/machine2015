#include <Servo.h>
#include "Hardware.h"
#include "Vehicle.h"
#include "AllStates.h"
using namespace Vehicle;
using namespace Vehicle::States;

LinearStateMachine* machine = 0;

void LoadStateMachine(){
  machine = new LinearStateMachine();
  if(false) // If Shooter is detected
  {
    machine->Add(new Charging());
    machine->Add(new Racing()); // Change for full sequence with Shooter.
  }
  else
  {
    machine->Add(new Racing());
  }
}

void UnloadStateMachine(){
  delete machine;
  machine = 0;
}


void setup() 
{
  Serial.begin(9600);
  
  Vehicle::Setup();
  
  LoadStateMachine();
}
 
void loop() 
{
  if(machine == 0)
  {
    if(false)
    { //Monitor reset button (?)
      LoadStateMachine();
    }
  }
  else
  {
    machine->Execute();
    UnloadStateMachine();
  }
} 

