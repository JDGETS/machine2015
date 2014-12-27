/***
 * <Store>
 *   States::store.Setup() must be called to initialize the Store!
 */
#ifndef STORE_H
#define STORE_H
#include <Servo.h>
#include <Arduino.h>
 
struct Store{
private:
    const static uint8_t STORE_SERVO_PIN = 6;
    const static int STORE_SERVO_MIN_MICROS = 600;
    const static int STORE_SERVO_MAX_MICROS = 2400; 
    unsigned long setupTime;
public:
    const static int INITIALIZE_DELAY = 2000;
    const static int STORE_SERVO_INITIAL_MICROS = 623; //Must be close enough to 600 (above)
    const static int STORE_SERVO_MICROS_STEP = 27; //For 1/8 rotation
    Servo servo; 
    int servoPosition;
    int bagsCount;
    
    Store();

    void Setup();
    void Initialize();
  
    void ServoLoadBag();
    void ServoUnloadBag();
    void ServoMoveBag(); // For testing purposes only.
};

namespace States{
  extern Store store;
};

#endif
