#ifndef STATESHOOTING_H
#define STATESHOOTING_H
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
      float rpm = 0;

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

        delay(SHOOTING_START_DELAY_STABILIZE);

        ReadRPM(NUMBER_OF_HOLES_TO_CALIBRATE);
         
        Serial.print("Current is: ");
        Serial.println(millis());

        //First bag is already in the shooter
        unsigned long nextShootingTime = GetNextShootingTime();
        while(!store.IsEmpty()) {
          Serial.print("Next shooting time: ");
          Serial.println(nextShootingTime);
          delay(nextShootingTime - millis() - SHOOTER_LOAD_AND_SHOOT_DELAY);
          CHECK_FORCE_STOP_MACRO
          Serial.println("Starting to crinque");
          digitalWrite(SHOOTER_MOTOR_MOSFET, HIGH);
          delay(SHOOTER_MOTOR_REQUIRED_TIME); // If it cuts too late, no big deal, the elastic will bring it back to initial position
          digitalWrite(SHOOTER_MOTOR_MOSFET, LOW);
          Serial.print("BOOM! Only ");
          Serial.print(store.bagsCount);
          Serial.println(" bags left!");
          delay(SHOOTER_MOTOR_RDY_TO_RECV_DELAY); // Delay to be sure the system is ready to receive the new bag. EDIT: Not necessary since we're waiting at least a full hole. EDIT2: Gros cave, c'est pour pas que ça tombe entre le launcher et la craque.
          DropNextBag();
          shooterSensor.WaitForDetect();
          CHECK_FORCE_STOP_MACRO
          Serial.println("Waiting for calibration hole...");
          shooterSensor.WaitForUndetect();
          shooterSensor.WaitForDetect(); // Wait one full turn
          CHECK_FORCE_STOP_MACRO
          nextHoleTime = millis(); // Realign "nextHoleTime" at the end
          nextShootingTime = GetNextShootingTime();
        }
         
        Serial.println("We are done with Shooting.");
        
        End();
        
        return;
      } 

      float ReadRPM(int numberOfHoles)
      {
        float delta_total = 0;
        int delta_count = 0;
        bool firstRead = true;
        Hole *lastHole = new Hole();
        shooterSensor.WaitForDetect(); // Switch is pressed, now we can start our sequence

        for(int i=0; i < numberOfHoles; i++){
          Hole *hole = new Hole();
          Serial.print("Waiting for hole #");
          Serial.println(i+1);
          ReadDataHole(hole);
          if(!firstRead){
            float delta_unit = hole->beginning - lastHole->beginning;
            float rpm = GetRPM(delta_unit);
            Serial.print("RPM is ");
            Serial.println(rpm);
            if(rpm < SHOOTER_TARGET_MIN_RPM || rpm > SHOOTER_TARGET_MAX_RPM)
            {
              Serial.print("RPM not between target RPMs! RPM=");
              Serial.println(rpm);
              i--;
            }
            else
            {
              delta_total += delta_unit;
              delta_count++;
            }
          }
          firstRead = false;
          delete lastHole;
          lastHole = hole;
        }
        
        nextHoleTime = lastHole->end; // It's aligned at the center
        delta = delta_total / delta_count;

        rpm = GetRPM(delta);

        Serial.print("Done reading target data. RPM=");
        Serial.println(rpm);
        Serial.print("Delta is: ");
        Serial.println(delta);
        
        delete lastHole;
        
        return rpm;
      }

      float GetRPM(float delta)
      {
        return 60.0*1000.0/delta / 2.0; // 2 holes per rotation
      }

      void ReadDataHole(Hole *hole){
        Serial.println("StateShooting - Wait for undetect");
        shooterSensor.WaitForUndetect();
        hole->beginning = millis();
        Serial.println("StateShooting - Wait for detect");
        shooterSensor.WaitForDetect();
        CHECK_FORCE_STOP_MACRO
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
          nextHoleTime += delta; // Sensor offset
        }
        return nextHoleTime;
      }
    };
  }
}
#endif