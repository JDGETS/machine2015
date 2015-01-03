#include "Vehicle.h"

namespace Vehicle{
  bool FORCE_STOP = false;
  bool COMPLETED_CHARGING = false;
  Motors motors(RIGHT_MOTOR_PIN_OUT, LEFT_MOTOR_PIN_OUT);
  LimitSwitch shooterSwitch(SHOOTER_SWITCH_PIN);
  LimitSwitch forceStopSwitch(VEHICLE_FORCESTOP_SWITCH_PIN, FORCESTOP_SWITCH_DEBOUNCE_TIME);
  VehicleReedSwitches reedswitches(REEDSWITCH_4, REEDSWITCH_3, REEDSWITCH_2, REEDSWITCH_1); // Order of a binary number.
#ifdef VEHICLE_USE_ULTRASONIC
  UltrasonicSensor bottomSensor(VEHICLE_US_SENSOR_TRIGGER_PIN, VEHICLE_US_SENSOR_ECHO_PIN, VEHICLE_US_SENSOR_MAX_DISTANCE);
#else
  OpticalSensor bottomSensor(VEHICLE_OPTICAL_SENSOR_PIN);
#endif
  Vehicle::Store store;
  LimitSwitch storeBagInSwitch(STORE_DETECT_BAG_SWITCH_PIN);
#ifdef SHOOTER_USE_ULTRASONIC
  UltrasonicSensor shooterSensor(SHOOTER_US_SENSOR_TRIGGER_PIN, SHOOTER_US_SENSOR_ECHO_PIN, SHOOTER_US_SENSOR_MAX_DISTANCE);
#else
  OpticalSensor shooterSensor(SHOOTER_OPTICAL_SENSOR_PIN);
#endif

  void CheckForceStop(){
    if(!FORCE_STOP && !forceStopSwitch.ReadInput())
    {
      Serial.println("Vehicle force stopped!");
      FORCE_STOP = true;
      Stop();
      delay(2000);
    }
  }

  void Setup(){
    //Init motor before servos (interference)
    pinMode(SHOOTER_MOTOR_MOSFET, OUTPUT);
    digitalWrite(SHOOTER_MOTOR_MOSFET, LOW);

    motors.Setup();

    shooterSwitch.Setup();

    forceStopSwitch.Setup();

    bottomSensor.Setup();
#ifdef VEHICLE_USE_ULTRASONIC
    shooterSensor.SetMinTriggerValue(VEHICLE_US_SENSOR_MIN_TRIGGER_VALUE);
    shooterSensor.SetConsecutiveZerosN(VEHICLE_US_SENSOR_CONSECUTIVE_ZEROS);
#else
    bottomSensor.Invert();
    bottomSensor.SetMinTriggerValue(VEHICLE_OPT_SENSOR_MIN_TRIGGER_VALUE);
    bottomSensor.SetUseLongRead(true);
#endif

    store.Setup();

    storeBagInSwitch.Setup();
    
    shooterSensor.Setup();
    shooterSensor.SetDebounceTime(SHOOTER_SENSOR_DEBOUNCE_TIME);
  #ifdef SHOOTER_USE_ULTRASONIC
    shooterSensor.SetMinTriggerValue(SHOOTER_US_SENSOR_MIN_TRIGGER_VALUE);
  #else
    shooterSensor.SetMinTriggerValue(SHOOTER_OPT_SENSOR_MIN_TRIGGER_VALUE);
    shooterSensor.SetUseLongRead(false);
  #endif

    reedswitches.Setup();
  }


  void Stop()
  {
    motors.Speed(MOTOR_LEFT, 0);
    motors.Speed(MOTOR_RIGHT, 0);
    digitalWrite(SHOOTER_MOTOR_MOSFET, LOW);
  }
}