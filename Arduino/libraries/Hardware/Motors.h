#ifndef MOTORS_H
#define MOTORS_H
#include <Arduino.h>

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- MOTORS ------------------------------------------------------------ ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

//Needed for interrupt()
extern long int LEFT_MOTOR_PWMVAL; 
extern long int RIGHT_MOTOR_PWMVAL;

enum MotorID { MOTOR_LEFT = 1, MOTOR_RIGHT = 2 };

struct MotorPinOut {
    const uint8_t DIR_A_PIN, DIR_B_PIN, PWN_PIN, ADC_PIN;
    MotorPinOut(uint8_t DIR_A_PIN, uint8_t DIR_B_PIN, uint8_t PWN_PIN, uint8_t ADC_PIN)
            : DIR_A_PIN(DIR_A_PIN), DIR_B_PIN(DIR_B_PIN), PWN_PIN(PWN_PIN), ADC_PIN(ADC_PIN) {};
};

struct Motors {
    const MotorPinOut RIGHT_PIN_OUT, LEFT_PIN_OUT;

    Motors(MotorPinOut RIGHT_PIN_OUT, MotorPinOut LEFT_PIN_OUT)
            : RIGHT_PIN_OUT(RIGHT_PIN_OUT), LEFT_PIN_OUT(LEFT_PIN_OUT), _LeftPWN(LEFT_MOTOR_PWMVAL),
              _RightPWM(RIGHT_MOTOR_PWMVAL) {};

    void Setup();

    const float Speed(const MotorID id) const;
    void Speed(const MotorID id, const float value);

    void BalancedMotorsSpeed(const MotorID referenceMotor, const float speed);

    void RunSpeedScript(const unsigned int numberOfSteps, const float * leftMotorSpeeds,
            const float * rightMotorSpeeds, const unsigned long * triggerTimes,
            const unsigned long firstPassTime);

private:
    const static float   RIGHT_MOTOR_RATIO = 0.65;
    long int &_RightPWM, &_LeftPWN;
    float _RightSpeed, _LeftSpeed;
};

//Needed for interrupt()
extern MotorPinOut LEFT_MOTOR_PIN_OUT;
extern MotorPinOut RIGHT_MOTOR_PIN_OUT;
extern Motors MOTORS;

#endif