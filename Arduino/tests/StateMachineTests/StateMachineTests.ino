#include <Servo.h>
#include <ForceStopVehicle.h>
#include <Hardware.h>
#include <Vehicle.h>
#include <State.h>
using namespace Vehicle::States;

class StateA : public State
{
public:
  StateA() : State() {};
  ~StateA() { Serial.println("Deleting StateA."); };
  void Execute() { Serial.println("Executing StateA."); End(); }
};

class StateB : public State
{
public:
  int timesLeft;
  StateB(int count) : State() { timesLeft = count;};
  ~StateB() { Serial.println("Deleting StateB."); };
  void Execute() { 
    Serial.println("Executing StateB."); 
    timesLeft--; 
    if(timesLeft <= 0)
      End(); 
  }
};

void setup() 
{
  Serial.begin(9600);
  
  LinearStateMachine machine;
  machine.Add(new StateA());
  machine.Add(new StateB(3));
  
  Serial.println("Starting the StateMachine.");
  
  machine.Execute();
  
  Serial.println("Done with the StateMachine.");
}
 
void loop() 
{
  
} 


