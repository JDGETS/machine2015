#include "Vehicle.h"

namespace Vehicle{
  Vehicle::Store store;
  LimitSwitch storeBagInSwitch(STORE_DETECT_BAG_SWITCH_PIN);
  LimitSwitch shooterSwitch(SHOOTER_SWITCH_PIN);

  void Setup(){
    //Init motor before servos (interference)
    pinMode(SHOOTER_MOTOR_MOSFET, OUTPUT);
    digitalWrite(SHOOTER_MOTOR_MOSFET, LOW);

    store.Setup();

    shooterSwitch.Setup();
    storeBagInSwitch.Setup();
  }
}