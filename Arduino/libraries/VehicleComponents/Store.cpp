/***
 * <Store>
 *   States::store.Setup() must be called to initialize the Store!
 */
#include <Servo.h>
#include "Vehicle.h"
using namespace Vehicle;

Store::Store(){
  servoPosition = STORE_SERVO_INITIAL_MICROS; //Set initial servo position behind the real desired position (get rid of deadzone)
  bagsCount = 0;
}

void Store::Setup(){
  servo.writeMicroseconds(servoPosition);
  servo.attach(STORE_SERVO_PIN, STORE_SERVO_MIN_MICROS, STORE_SERVO_MAX_MICROS);
  setupTime = millis(); 
};

void Store::Initialize(){
  unsigned long wait = STORE_INITIALIZE_DELAY - (millis() - setupTime);
  if(wait > 0)
    delay(wait); //Wait for store to align itself (setup)
  servoPosition += STORE_SERVO_MICROS_STEP*3; //Get rid of deadzone when changing directions
  servo.writeMicroseconds(servoPosition); //Set real initial servo
};

void Store::ServoLoadBag(){
  bagsCount += 1;
  double bagsPercent = ((double)bagsCount)/((double)STORE_MAX_NUMBER_OF_BAGS);
  servoPosition += STORE_SERVO_MICROS_STEP;
  servo.writeMicroseconds(servoPosition + STORE_SERVO_MICROS_STEP * STORE_OVERSHOOTING_LOAD_BAG);
  delay(STORE_OVERSHOOTING_LOAD_BAG_DELAY);
  servo.writeMicroseconds(servoPosition);
}

void Store::ServoUnloadBag(){
  bagsCount -= 1;
  double bagsPercent = ((double)bagsCount)/((double)STORE_MAX_NUMBER_OF_BAGS);
  servoPosition += STORE_SERVO_MICROS_STEP;
  servo.writeMicroseconds(servoPosition + STORE_SERVO_MICROS_STEP * STORE_OVERSHOOTING_UNLOAD_BAG);
  delay(STORE_OVERSHOOTING_UNLOAD_BAG_DELAY);
  servo.writeMicroseconds(servoPosition);
}

void Store::ServoMoveBag(){ // For testing purposes only.
  servoPosition += STORE_SERVO_MICROS_STEP;
  servo.writeMicroseconds(servoPosition);
}

bool Store::IsEmpty(){ 
  return bagsCount == 0;
}

bool Store::IsFull(){
  return bagsCount == STORE_MAX_NUMBER_OF_BAGS;
}
/***
 * </Store>
 */
