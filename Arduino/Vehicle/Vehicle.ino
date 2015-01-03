#include <Servo.h>
#include "ForceStopVehicle.h"
#include "Hardware.h"
#include "Vehicle.h"
#include "AllStates.h"
using namespace Vehicle;
using namespace Vehicle::States;

LinearStateMachine* machine = 0;

void LoadStateMachine(){
  machine = new LinearStateMachine();
  if(shooterSwitch.ReadInput()) // If Shooter is detected
  {
    Serial.println("Loading shooting state machine.");
    machine->Add(new Charging());
    machine->Add(new ParcoursAvecLanceur());
    machine->Add(new MonteeAvecLanceur());
    machine->Add(new DecenteAvecLanceur());  
    machine->Add(new VirageEntreeZoneLancement());
    machine->Add(new AlignmentReedSwitchZoneLancement());
    machine->Add(new Shooting());
  }
  else
  {
    Serial.println("Loading racing state machine.");
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
  
  Serial.println("Loading state machine.");
  
  LoadStateMachine();
}'
 
void loop() 
{
  if(machine == 0 && !forceStopSwitch.ReadInput())
  {
      if(FORCE_STOP)
      {
        Serial.println("Waiting for signal to go back on");
        forceStopSwitch.WaitForPress();
        Vehicle::FORCE_STOP = false;
      }
      delay(4000);
      LoadStateMachine();
  }
  else
  {
    machine->Execute();
    UnloadStateMachine();
  }
} 

