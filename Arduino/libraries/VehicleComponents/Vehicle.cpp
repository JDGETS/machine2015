#include "Vehicle.h"

namespace Vehicle{
  Motors motors(RIGHT_MOTOR_PIN_OUT, LEFT_MOTOR_PIN_OUT);
  VehicleReedSwitches reedswitches(REEDSWITCH_1, REEDSWITCH_2, REEDSWITCH_3, REEDSWITCH_4);
  OpticalSensor bottomOpticalSensor(VEHICLE_SENSOR_PIN);
  Vehicle::Store store;
  LimitSwitch storeBagInSwitch(STORE_DETECT_BAG_SWITCH_PIN);
  #ifdef SHOOTER_USE_ULTRASONIC
  UltrasonicSensor shooterSensor(SHOOTER_US_SENSOR_TRIGGER_PIN, SHOOTER_US_SENSOR_ECHO_PIN, SHOOTER_US_SENSOR_TRIGGER_PIN);
  #else
  OpticalSensor shooterSensor(SHOOTER_OPTICAL_SENSOR_PIN);
  #endif
  Servo vehicleServo; // Used to drop the shooter

  void Setup(){
    //Init motor before servos (interference)
    pinMode(SHOOTER_MOTOR_MOSFET, OUTPUT);
    digitalWrite(SHOOTER_MOTOR_MOSFET, LOW);

    motors.Setup();

    bottomOpticalSensor.Setup();
    bottomOpticalSensor.Invert();
    bottomOpticalSensor.SetMinTriggerValue(VEHICLE_SENSOR_MIN_VALUE);
    bottomOpticalSensor.SetUseLongRead(true);

    store.Setup();

    storeBagInSwitch.Setup();
    shooterSensor.SetDebounceTime(SHOOTER_SENSOR_DEBOUNCE_TIME);
    shooterSensor.Setup();
    #ifndef SHOOTER_OPTICAL_SENSOR_PIN
    shooterSensor.SetUseLongRead(true);
    #endif

    reedswitches.Setup();

    //Configure the right values first.
    //vehicleServo.writeMicroseconds(VEHICLE_SERVO_INITIAL_MICROS);
    //vehicleServo.attach(VEHICLE_SERVO_PIN, VEHICLE_SERVO_MIN_MICROS, VEHICLE_SERVO_MAX_MICROS);
  }
}