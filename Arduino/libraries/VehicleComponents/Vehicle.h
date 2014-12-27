#include <Servo.h>
#include <LimitSwitch.h>
#include <Store.h>
#include <State.h>

#define SHOOTER_SWITCH_PIN 38
#define STORE_DETECT_BAG_SWITCH_PIN 40
#define SHOOTER_MOTOR_MOSFET 7

// Charging
#define CHARGING_SWITCH_TO_STORE_DELAY 400 // Time it takes to charge and shoot the bag and time until it reaches the hole (millis)

// Shooter constants
#define LOAD_AND_SHOOT_DELAY 400 // Time it takes to charge and shoot the bag and time until it reaches the hole (millis)
#define BAG_FREE_FALL_DELAY 500 //Time for the bag to fall into place. Put some padding in there.

#ifndef VEHICLE_H
#define VEHICLE_H
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
  namespace States{};
}
#endif