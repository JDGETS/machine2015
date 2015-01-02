#include <Servo.h>
#include "Hardware.h"
#include "Vehicle.h"
#include "AllStates.h"
using namespace Vehicle;
using namespace Vehicle::States;

LinearStateMachine* machine = 0;

void LoadStateMachine(){
  machine = new LinearStateMachine();
  if(true) // If Shooter is detected
  {
    machine->Add(new Charging());
    //To-do: add transition state that ignores the IR bottom sensor.
    machine->Add(new ParcoursAvecLanceur());
    machine->Add(new MonteeAvecLanceur());
    machine->Add(new DecenteAvecLanceur());  
    machine->Add(new VirageEntreeZoneLancement());
    machine->Add(new AlignmentReedSwitchZoneLancement());
    machine->Add(new NewShooting());
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

