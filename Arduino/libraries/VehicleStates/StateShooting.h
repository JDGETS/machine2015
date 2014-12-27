#include "Vehicle.h"
using namespace Vehicle;

namespace Vehicle{
  namespace States {
    class Shooting : public State 
    {
    public:
      unsigned long nextHoleTime;
      unsigned long delta;

      Shooting() : State() {
        nextHoleTime = -1;
        delta = -1;
      };

      void Execute() 
      {
        unsigned long firstHole[2], secondHole[2];
        unsigned long firstHoleTime, secondHoleTime; 
        
        Serial.print("Start time: ");
        unsigned long time = millis();
        Serial.println(time);
        
        shooterSwitch.ResetDebouncing();
        shooterSwitch.WaitForPress(); //Switch is pressed, now we can start our sequence
        Serial.println("Waiting for first hole...");
        shooterSwitch.WaitForRelease();
        firstHole[0] = millis();
        shooterSwitch.WaitForPress();
        firstHole[1] = millis();
        Serial.println("Waiting for second hole...");
        shooterSwitch.WaitForRelease();
        secondHole[0] = millis();
        shooterSwitch.WaitForPress();
        secondHole[1] = millis();
        
        firstHoleTime = firstHole[1] - firstHole[0];
        secondHoleTime = secondHole[1] - secondHole[0];
        delta = secondHole[0] - firstHole[0];
        nextHoleTime = secondHole[1]; //It's aligned at the center
        Serial.print("Delay is: ");
        Serial.println(delta);
         
        Serial.print("Current is: ");
        Serial.println(millis());
        //First bag is already in the shooter
        unsigned long nextShootingTime = GetNextShootingTime();
        Serial.print("Next is: ");
        Serial.println(nextShootingTime);
        
        while(store.bagsCount > 0) {
          delay(nextShootingTime - millis() - LOAD_AND_SHOOT_DELAY); //TO-DO: Precise enough (?)
          Serial.println("Starting to crinque");
          digitalWrite(SHOOTER_MOTOR_MOSFET, HIGH);
          delay(700); //Approx
          //EndMoteur (if it cuts too late, 2 cases: 1) elastics strong enough to bring back the shooter at its place. no problem then. 2) not strong enough. then you have to adjust LOAD_AND_SHOOT_DELAY for bags #2-8).
          digitalWrite(SHOOTER_MOTOR_MOSFET, LOW); 
          Serial.print("BOOM! Only ");
          Serial.print(store.bagsCount);
          Serial.println(" bags left!");
          delay(2000); //Delay to be sure the system is ready to receive the new bag. TO-DO: Get rid of this ??? or constant this.
          if(store.bagsCount > 0){
            DropNextBag();
            nextShootingTime = GetNextShootingTime();
            Serial.print("Next: ");
            Serial.println(nextShootingTime);
          }
        }
         
        Serial.println("We are done with Shooting.");
        
        End();
        
        return;
      } 
      
      // Responsable for dropping the next bag. Include the delay in there. Only return when the bag is in the shooter ready to get shot.
      void DropNextBag()
      {
        Serial.println("Dropped bag.");
        
        store.ServoUnloadBag();
        
        delay(BAG_FREE_FALL_DELAY); //Wait for the bag to fall into place.
      }
      
      unsigned long GetNextShootingTime()
      { // Must take into account LOAD_AND_SHOOT_DELAY. This should be the next hole that is theoritically shootable in. Assume computing time is negligeable.
        unsigned long currentTime = millis();
        while(nextHoleTime < currentTime + LOAD_AND_SHOOT_DELAY)
        {
          nextHoleTime += delta;
        }
        return nextHoleTime;
      }
    };
  }
}
