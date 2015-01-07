#ifndef VEHICLE_H
#define VEHICLE_H

#include <Arduino.h>
#include <Servo.h>
#include <Hardware.h>
#include <Store.h>
#include <State.h>

// Pins
#define VEHICLE_FORCESTOP_SWITCH_PIN  52
#define SHOOTER_SWITCH_PIN            48
#define STORE_DETECT_BAG_SWITCH_PIN   50
#define VEHICLE_OPTICAL_SENSOR_PIN    A9
#define VEHICLE_US_SENSOR_TRIGGER_PIN 42
#define VEHICLE_US_SENSOR_ECHO_PIN    40
#define STORE_SERVO_PIN               5
#define SHOOTER_OPTICAL_SENSOR_PIN    A8 
#define SHOOTER_US_SENSOR_TRIGGER_PIN 46
#define SHOOTER_US_SENSOR_ECHO_PIN    44
#define SHOOTER_MOTOR_MOSFET          7

// Motors configuration // Use MotorPinOut defined in Motors.cpp
#define LEFT_MOTOR_PIN_OUT MotorPinOut(A0, A1, 11, A2)
#define RIGHT_MOTOR_PIN_OUT MotorPinOut(A5, A4, 12, A3)

// Reedswitches configuration // Use MotorPinOut defined in Motors.cpp
#define REEDSWITCH_1 PairedReedSwitch(36, 38)
#define REEDSWITCH_2 PairedReedSwitch(32, 34)
#define REEDSWITCH_3 PairedReedSwitch(28, 30)
#define REEDSWITCH_4 PairedReedSwitch(26, 24)

// Servo configuration
#define STORE_SERVO_MIN_MICROS      600
#define STORE_SERVO_MAX_MICROS      2400 

// Vehicle
#define FORCESTOP_SWITCH_DEBOUNCE_TIME  1000
//#define VEHICLE_USE_ULTRASONIC // Comment this out to use the OpticalSensor
#ifdef VEHICLE_USE_ULTRASONIC // Ultrasonic
#define VEHICLE_US_SENSOR_MAX_DISTANCE        15
#define VEHICLE_US_SENSOR_MIN_TRIGGER_VALUE   8
#define VEHICLE_US_SENSOR_CONSECUTIVE_ZEROS   3
#else
#define VEHICLE_OPT_SENSOR_MIN_TRIGGER_VALUE  0.15
#endif

// Store
#define STORE_SERVO_INITIAL_MICROS  629   // Must be close enough to 600 (STORE_SERVO_MIN_MICROS) - Is sometimes 613 or 620
#define STORE_INITIALIZE_DELAY      4500  // Time to reinitialize the store to STORE_SERVO_INITIAL_MICROS
#define STORE_SERVO_MICROS_STEP     27    // For 1/8 rotation
#define STORE_MAX_NUMBER_OF_BAGS    8
// STORE_OVERSHOOTING_*: 
//   Use `bagsPercent` if you want to write a formula based on number of bags in the Store. 
#define STORE_OVERSHOOTING_LOAD_BAG         0.60*bagsPercent
#define STORE_OVERSHOOTING_LOAD_BAG_DELAY   550 + 450 * bagsPercent 
#define STORE_OVERSHOOTING_UNLOAD_BAG       0.60*bagsPercent + 0.3
#define STORE_OVERSHOOTING_UNLOAD_BAG_DELAY 650 + 450 * bagsPercent 

// Charging
#define CHARGING_SWITCH_TO_STORE_DELAY  400   // Time it takes to charge and shoot the bag and time until it reaches the hole (millis)
#define CHARGING_DONE_DELAY             1000  // Delay after the vehicle is done charging before transitioning into the next state.

// Shooter constants
#define SHOOTER_TARGET_MIN_RPM 15
#define SHOOTER_TARGET_MAX_RPM 20
#define SHOOTING_START_DELAY_STABILIZE  1000
#define NUMBER_OF_HOLES_TO_CALIBRATE    5
#define SHOOTER_USE_ULTRASONIC // Comment this out to use the OpticalSensor
#ifdef SHOOTER_USE_ULTRASONIC // Ultrasonic
#define SHOOTER_US_SENSOR_MAX_DISTANCE  20
#define SHOOTER_US_SENSOR_MIN_TRIGGER_VALUE 25
#define SHOOTER_SENSOR_DEBOUNCE_TIME    200
#define SHOOTER_MOTOR_REQUIRED_TIME     700   // Time required to power the motor
#define SHOOTER_LOAD_AND_SHOOT_DELAY    (385 + (rpm-12)*250/6)  // Time it takes to charge and shoot the bag and time until it reaches the hole (millis)
                                              // If it cuts too late, no big deal, the elastic will bring it back to initial position
                                              // Is dependant on sensor placement.
#else // Optical
#define SHOOTER_OPT_SENSOR_MIN_TRIGGER_VALUE 2.0
#define SHOOTER_SENSOR_DEBOUNCE_TIME    100
#define SHOOTER_MOTOR_REQUIRED_TIME     700   // Time required to power the motor
#define SHOOTER_LOAD_AND_SHOOT_DELAY    800   // Time it takes to charge and shoot the bag and time until it reaches the hole (millis)
                                              // If it cuts too late, no big deal, the elastic will bring it back to initial position
                                              // Is dependant on sensor placement.
#endif
#define SHOOTER_BAG_FREE_FALL_DELAY     500   // Time for the bag to fall into place. Put some padding in there.
#define SHOOTER_MOTOR_RDY_TO_RECV_DELAY 1000  // Delay to be sure the system is ready to receive the new bag (for the elastic to bring the launcher back to initial position). Is used after the motor has been shut down and unloading the new one.

// Drop Shooter
#define VEHICLE_SERVO_INITIAL_MICROS    623   // Must be close enough to (VEHICLE_SERVO_MIN_MICROS)
#define VEHICLE_SERVO_FINAL_MICROS      623   // Must be close enough to (VEHICLE_SERVO_MIN_MICROS)
#define VEHICLE_SERVO_DELAY             2000  // Delay required to drop the shooter on the ground

// Racing
#define RACING_LEFT_MOTOR_SPEED 0.65 // If there's no imperfection in the track.
#define RACING_RIGHT_MOTOR_SPEED 0.55

namespace Vehicle{
  extern bool COMPLETED_CHARGING;
  extern Motors motors;
  extern Vehicle::Store store;
  extern LimitSwitch forceStopSwitch;
  extern LimitSwitch shooterSwitch;
  extern LimitSwitch storeBagInSwitch;
  #ifdef SHOOTER_USE_ULTRASONIC
  extern UltrasonicSensor shooterSensor;
  #else
  extern OpticalSensor shooterSensor;
  #endif
  #ifdef VEHICLE_USE_ULTRASONIC
  extern UltrasonicSensor bottomSensor;
  #else
  extern OpticalSensor bottomSensor;
  #endif
  extern VehicleReedSwitches reedswitches;

  void Setup();
  void Stop();

  namespace States{};
}

#endif