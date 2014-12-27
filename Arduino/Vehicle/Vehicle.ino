
#include <Servo.h>
#include "LimitSwitch.h"
#include "Store.h"

#define SHOOTER_SWITCH_PIN 38


namespace Vehicle{
  //Shooting stuff
  LimitSwitch shooterSwitch(SHOOTER_SWITCH_PIN);
  LimitSwitch bagHolderSwitch(BAG_HOLDER_SWITCH_PIN);

  unsigned long nextHoleTime = -1;
  
  Vehicle::Store store;
  
  namespace States{};
} using namespace Vehicle;

void setup() 
{
  Serial.begin(9600);
  
  //Init motor before servos (interference)
  pinMode(SHOOTER_MOTOR_MOSFET, OUTPUT);
  digitalWrite(SHOOTER_MOTOR_MOSFET, LOW);
  
  store.Setup();
  
  shooterSwitch.Setup();
  bagHolderSwitch.Setup();
  
  //pinMode(STORE_SWITCH_PIN, INPUT_PULLUP);  
  store.Initialize();

  store.bagsCount = 8; //The store is considered full here. //TO-DO: Remove when Charging state is done.
}
 
void loop() 
{
  
} 

