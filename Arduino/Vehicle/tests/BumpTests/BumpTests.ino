#include <Servo.h>
#include "Hardware.h"
#include "Vehicle.h"
#include "AllStates.h"
using namespace Vehicle;
using namespace Vehicle::States;

void setup() 
{
  Serial.begin(9600);
  
  Vehicle::Setup();
  
  LinearStateMachine machine;
  machine.Add(new ParcoursAvecLanceur());
  machine.Add(new MonteeAvecLanceur());
  machine.Add(new DecenteAvecLanceur());
  machine.Add(new VirageEntreeZoneLancement());
  machine.Add(new AlignmentReedSwitchZoneLancement());
  
  Serial.println("Starting the BumpTests StateMachine.");
  
  machine.Execute();
}
 
void loop() 
{
  
} 

