/***
 * <Store>
 *   States::store.Setup() must be called to initialize the Store!
 */
#include <Servo.h>
#include "Store.h"
using namespace Vehicle;

Store::Store(){
  servoPosition = STORE_SERVO_INITIAL_MICROS; //Set initial servo position behind the real desired position (get rid of deadzone)
  bagsCount = 1;
}

void Store::Setup(){
  servo.writeMicroseconds(servoPosition);
  servo.attach(STORE_SERVO_PIN, STORE_SERVO_MIN_MICROS, STORE_SERVO_MAX_MICROS);
  setupTime = millis(); 
};

void Store::Initialize(){
  unsigned long wait = INITIALIZE_DELAY - (millis() - setupTime);
  if(wait > 0)
    delay(wait); //Wait for store to align itself (setup)
  servoPosition += STORE_SERVO_MICROS_STEP*3; //Get rid of deadzone when changing directions
  servo.writeMicroseconds(servoPosition); //Set real initial servo
};

void Store::ServoLoadBag(){
  bagsCount += 1;
  servoPosition += STORE_SERVO_MICROS_STEP;
  double bagsPercent = ((double)bagsCount)/8.0;
  servo.writeMicroseconds(servoPosition + STORE_SERVO_MICROS_STEP * bagsPercent);
  delay(700 - 200 * bagsPercent);
  servo.writeMicroseconds(servoPosition);
}

void Store::ServoUnloadBag(){
  bagsCount -= 1;
  servoPosition += STORE_SERVO_MICROS_STEP;
  servo.writeMicroseconds(servoPosition + STORE_SERVO_MICROS_STEP);
  delay(500);
  servo.writeMicroseconds(servoPosition);
}

void Store::ServoMoveBag(){ // For testing purposes only.
  servoPosition += STORE_SERVO_MICROS_STEP;
  servo.writeMicroseconds(servoPosition);
}
/***
 * </Store>
 */
