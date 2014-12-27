#ifndef VEHICLE_H
#define VEHICLE_H

#include <Servo.h>
#include <LimitSwitch.h>
#include <Store.h>
#include <State.h>

// Pins
#define STORE_DETECT_BAG_SWITCH_PIN 40
#define STORE_SERVO_PIN             6
#define SHOOTER_SWITCH_PIN          38
#define SHOOTER_MOTOR_MOSFET        7

// Store
#define STORE_SERVO_MIN_MICROS      600
#define STORE_SERVO_MAX_MICROS      2400 
#define STORE_INITIALIZE_DELAY      3500  // Time to reinitialize the store to STORE_SERVO_INITIAL_MICROS
#define STORE_SERVO_INITIAL_MICROS  623   // Must be close enough to 600 (above)
#define STORE_SERVO_MICROS_STEP     27    // For 1/8 rotation
#define STORE_MAX_NUMBER_OF_BAGS    8
// STORE_OVERSHOOTING_*: 
//   Use `bagsPercent` if you want to write a formula based on number of bags in the Store. 
#define STORE_OVERSHOOTING_LOAD_BAG         bagsPercent
#define STORE_OVERSHOOTING_LOAD_BAG_DELAY   700 - 200 * bagsPercent 
#define STORE_OVERSHOOTING_UNLOAD_BAG       1 
#define STORE_OVERSHOOTING_UNLOAD_BAG_DELAY 500

// Charging
#define CHARGING_SWITCH_TO_STORE_DELAY  400   // Time it takes to charge and shoot the bag and time until it reaches the hole (millis)

// Shooter constants
#define SHOOTER_MOTOR_REQUIRED_TIME     700   // Time required to power the motor
#define SHOOTER_LOAD_AND_SHOOT_DELAY    400   // Time it takes to charge and shoot the bag and time until it reaches the hole (millis)
                                              // If it cuts too late, no big deal, the elastic will bring it back to initial position
#define SHOOTER_BAG_FREE_FALL_DELAY     500   // Time for the bag to fall into place. Put some padding in there.
#define SHOOTER_MOTOR_RDY_TO_RECV_DELAY 1000  // Delay to be sure the system is ready to receive the new bag (for the elastic to bring the launcher back to initial position). Is used after the motor has been shut down and unloading the new one.

namespace Vehicle{
  extern Vehicle::Store store;
  extern LimitSwitch storeBagInSwitch;
  extern LimitSwitch shooterSwitch;

  void Setup();

  namespace States{};
}

#endif