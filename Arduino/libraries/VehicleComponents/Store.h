/***
 * <Store>
 *   States::store.Setup() must be called to initialize the Store!
 */
#ifndef STORE_H
#define STORE_H
#include <Servo.h>
#include <Arduino.h>
 
namespace Vehicle{
    struct Store{
    private:
        unsigned long setupTime;
    public:
        Servo servo; 
        int servoPosition;
        int bagsCount;
        
        Store();

        void Setup();
        void Initialize();
      
        void ServoLoadBag();
        void ServoUnloadBag();
        void ServoMoveBag(); // For testing purposes only.

        bool IsEmpty();
        bool IsFull();
    };

    extern Store store;
}

#endif
