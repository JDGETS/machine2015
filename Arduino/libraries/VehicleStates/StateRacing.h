#ifndef STATERACING_H
#define STATERACING_H
#include <Vehicle.h>

namespace Vehicle{
  namespace States {
    class Racing : public State
    {
    public:

      Racing() : State() {};

      void Execute() 
      {
        MOTORS.Speed(MOTOR_LEFT, RACING_LEFT_MOTOR_SPEED);
        MOTORS.Speed(MOTOR_RIGHT, RACING_RIGHT_MOTOR_SPEED);
        CHECK_FORCE_STOP_MACRO
        // Never call End();
        return;
      }
    };
  }
}
#endif