#include <Servo.h>
#include "Hardware.h"\
#include "Vehicle.h"
#include "AllStates.h"
using namespace Vehicle;


void setup() 
{
  
  Vehicle::Setup();
  

  motors.Speed(MOTOR_LEFT, 0.00);
  motors.Speed(MOTOR_RIGHT, 0.00) ;
  
  
  
}

void loop(){
  
}
