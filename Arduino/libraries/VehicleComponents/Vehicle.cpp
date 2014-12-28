#include "Vehicle.h"

namespace Vehicle{
  Vehicle::Store store;
  LimitSwitch storeBagInSwitch(STORE_DETECT_BAG_SWITCH_PIN);
  //LimitSwitch shooterSwitch(SHOOTER_SWITCH_PIN);
  OpticalSensor shooterSensor(SHOOTER_SENSOR_PIN);
  Servo vehicleServo; // Used to drop the shooter

  void Setup(){
    //Init motor before servos (interference)
    pinMode(SHOOTER_MOTOR_MOSFET, OUTPUT);
    digitalWrite(SHOOTER_MOTOR_MOSFET, LOW);

    store.Setup();

    storeBagInSwitch.Setup();
    //shooterSwitch.Setup();
    shooterSensor.Setup();
    shooterSensor.SetDebounceTime(SHOOTER_SENSOR_DEBOUNCE_TIME);
    shooterSensor.Invert();

    //Configure the right values first.
    //vehicleServo.writeMicroseconds(VEHICLE_SERVO_INITIAL_MICROS);
    //vehicleServo.attach(VEHICLE_SERVO_PIN, VEHICLE_SERVO_MIN_MICROS, VEHICLE_SERVO_MAX_MICROS);
  }
}