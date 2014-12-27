#include "Vehicle.h"
using namespace Vehicle;

namespace Vehicle{
  namespace States {
    struct Hole{
      unsigned long beginning, end, time;
    };

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
        Hole first, second;
        
        Serial.print("Start time: ");
        unsigned long time = millis();
        Serial.println(time);
        
        shooterSwitch.ResetDebouncing();
        shooterSwitch.WaitForPress(); // Switch is pressed, now we can start our sequence
        Serial.println("Waiting for first hole...");
        ReadDataHole(&first);
        Serial.println("Waiting for second hole...");
        ReadDataHole(&second);
        
        delta = second.beginning - first.beginning;
        nextHoleTime = second.beginning; // It's aligned at the center
        Serial.print("Delay is: ");
        Serial.println(delta);
         
        Serial.print("Current is: ");
        Serial.println(millis());
        
        //First bag is already in the shooter
        unsigned long nextShootingTime = GetNextShootingTime();
        while(!store.IsEmpty()) {
          Serial.print("Next shooting time: ");
          Serial.println(nextShootingTime);
          delay(nextShootingTime - millis() - SHOOTER_LOAD_AND_SHOOT_DELAY);
          Serial.println("Starting to crinque");
          digitalWrite(SHOOTER_MOTOR_MOSFET, HIGH);
          delay(SHOOTER_MOTOR_REQUIRED_TIME); // If it cuts too late, no big deal, the elastic will bring it back to initial position
          digitalWrite(SHOOTER_MOTOR_MOSFET, LOW);
          Serial.print("BOOM! Only ");
          Serial.print(store.bagsCount);
          Serial.println(" bags left!");
          delay(SHOOTER_MOTOR_RDY_TO_RECV_DELAY); // Delay to be sure the system is ready to receive the new bag. TO-DO: Get rid of this ??? or constant this.
          DropNextBag();
          nextShootingTime = GetNextShootingTime();
        }
         
        Serial.println("We are done with Shooting.");
        
        End();
        
        return;
      } 

      void ReadDataHole(Hole *hole){
        shooterSwitch.WaitForRelease();
        hole->beginning = millis();
        shooterSwitch.WaitForPress();
        hole->end = millis();
        hole->time = hole->end - hole->beginning;
      }
      
      // Responsible for dropping the next bag. Include the delay in there. Only return when the bag is in the shooter ready to get shot.
      void DropNextBag()
      {
        store.ServoUnloadBag();
        delay(SHOOTER_BAG_FREE_FALL_DELAY); //Wait for the bag to fall into place.
        Serial.println("Dropped bag.");
      }
      
      // Must take into account SHOOTER_LOAD_AND_SHOOT_DELAY. This should be the next hole that is theoritically shootable in. Assume computing time is negligeable.
      unsigned long GetNextShootingTime()
      { 
        unsigned long currentTime = millis();
        while(nextHoleTime < currentTime + SHOOTER_LOAD_AND_SHOOT_DELAY)
        {
          nextHoleTime += delta;
        }
        return nextHoleTime;
      }
    };
  }
}
