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
        store.Initialize();

        while(Vehicle::store.bagsCount < 8){
          storeBagInSwitch.WaitForPress();
          storeBagInSwitch.WaitForRelease();
          delay(CHARGING_SWITCH_TO_STORE_DELAY);
          Serial.println("Loaded a bag!");
          store.ServoLoadBag();
        }
        Serial.println("We are done with Charging.");
        End(); 
        return;
      }
    };
  }
}
#endif