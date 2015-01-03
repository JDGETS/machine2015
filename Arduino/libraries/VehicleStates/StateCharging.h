#ifndef STATECHARGING_H
#define STATECHARGING_H
#include <Vehicle.h>

namespace Vehicle{
  namespace States {
    class Charging : public State
    {
    public:
      Charging() : State() {};

      void Execute() 
      {   
        Serial.println("Starting Charging State.");
        CHECK_FORCE_STOP_MACRO

        if(COMPLETED_CHARGING)
          return End();

        Serial.println("Initializing store.");
        store.Initialize();

        while(!Vehicle::store.IsFull()){
          storeBagInSwitch.WaitForPress();
          storeBagInSwitch.WaitForRelease();
          delay(CHARGING_SWITCH_TO_STORE_DELAY);
          Serial.println("Loaded a bag!");
          store.ServoLoadBag();
        }
        COMPLETED_CHARGING = true;
        delay(CHARGING_DONE_DELAY); 
        Serial.println("We are done with Charging.");
        End(); 
        return;
      }
    };
  }
}
#endif