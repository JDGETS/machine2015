// This state ends when the Vehicle is ready to go into his Racing state.
#ifndef STATEDROPSHOOTER_H
#define STATEDROPSHOOTER_H
#include <Vehicle.h>

namespace Vehicle{
  namespace States {
    class DropShooter : public State
    {
    public:
      DropShooter() : State() {};

      void Execute() 
      {
        vehiculeServo.writeMicroseconds(VEHICLE_SERVO_FINAL_MICROS);
        delay(VEHICLE_SERVO_DELAY);
        // TO-DO: Move forward a bit
        // TO-DO: Slight left turn in order to be ready to go into Race mode.
        Serial.println("We are done with DropShooter.");
        End(); 
        return;
      }
    };
  }
}
#endif