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
        MOTORS.Speed(MOTOR_LEFT, RACING_MOTOR_SPEED);
        MOTORS.Speed(MOTOR_RIGHT, RACING_MOTOR_SPEED);
        // Never call End();
        return;
      }
    };
  }
}
#endif