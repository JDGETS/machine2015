#include "Vehicle.h"

namespace Vehicle{
  Motors motors(RIGHT_MOTOR_PIN_OUT, LEFT_MOTOR_PIN_OUT);
  Vehicle::Store store;
  LimitSwitch storeBagInSwitch(STORE_DETECT_BAG_SWITCH_PIN);
  //LimitSwitch shooterSwitch(SHOOTER_SWITCH_PIN);
  OpticalSensor shooterSensor(SHOOTER_SENSOR_PIN);
  Servo vehicleServo; // Used to drop the shooter
  OpticalSensor bottomOpticalSensor(A9);

  void Setup(){
    //Init motor before servos (interference)
    pinMode(SHOOTER_MOTOR_MOSFET, OUTPUT);
    digitalWrite(SHOOTER_MOTOR_MOSFET, LOW);

    motors.Setup();

    bottomOpticalSensor.SetMinTriggerValue(VEHICLE_SENSOR_MIN_VALUE);
    bottomOpticalSensor.Invert();

    store.Setup();

    storeBagInSwitch.Setup();
    //shooterSwitch.Setup();
    shooterSensor.Setup();
    shooterSensor.SetDebounceTime(SHOOTER_SENSOR_DEBOUNCE_TIME);

    //Configure the right values first.
    //vehicleServo.writeMicroseconds(VEHICLE_SERVO_INITIAL_MICROS);
    //vehicleServo.attach(VEHICLE_SERVO_PIN, VEHICLE_SERVO_MIN_MICROS, VEHICLE_SERVO_MAX_MICROS);
  }
}