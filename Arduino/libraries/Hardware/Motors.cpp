#include <Arduino.h>
#include "Motors.h"

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- MOTORS ------------------------------------------------------------ ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

//KEEP THESE VARIABLES GLOBAL (needed for interrupt())
long int LEFT_MOTOR_PWMVAL = 0; 
long int RIGHT_MOTOR_PWMVAL = 0;

MotorPinOut LEFT_MOTOR_PIN_OUT   = MotorPinOut(A0, A1, 11, A2);
MotorPinOut RIGHT_MOTOR_PIN_OUT  = MotorPinOut(A5, A4, 12, A3);
Motors MOTORS                    = Motors(RIGHT_MOTOR_PIN_OUT, LEFT_MOTOR_PIN_OUT);

void Motors::Setup()
{
    delayMicroseconds(100);

    pinMode(LEFT_PIN_OUT.PWN_PIN, OUTPUT);//OC1A
    pinMode(RIGHT_PIN_OUT.PWN_PIN, OUTPUT);//OC1B

    pinMode(LEFT_PIN_OUT.DIR_A_PIN, OUTPUT);//Dir1A
    pinMode(LEFT_PIN_OUT.DIR_B_PIN, OUTPUT);//Dir1B

    pinMode(RIGHT_PIN_OUT.DIR_A_PIN, OUTPUT);//Dir2A
    pinMode(RIGHT_PIN_OUT.DIR_B_PIN, OUTPUT);//Dir2B

    digitalWrite(LEFT_PIN_OUT.PWN_PIN, LOW);
    digitalWrite(LEFT_PIN_OUT.DIR_A_PIN, LOW);
    digitalWrite(LEFT_PIN_OUT.DIR_B_PIN, HIGH);
    _LeftPWN = 0;
    _LeftSpeed = 0.0;
    //_LeftDirectionRatio = 1.0;

    digitalWrite(RIGHT_PIN_OUT.PWN_PIN, LOW);
    digitalWrite(RIGHT_PIN_OUT.DIR_A_PIN, LOW);
    digitalWrite(RIGHT_PIN_OUT.DIR_B_PIN, HIGH);
    _RightPWM  = 0;
    _RightSpeed = 0.0;
    //_RightDirectionRatio = 1.0;

    //Setup PWM
    //SET : Fast PWM with Timer OverFlow Interrupt
    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(WGM13) | _BV(CS10);
    TIMSK1 = _BV(TOIE1);
    ICR1 = 1599; //Period value, TODO :  Add the correct formula from ATMEGA128 Datasheet
    interrupts();

    analogReference(INTERNAL1V1);
}

const float Motors::Speed(const MotorID id) const { return (id == MOTOR_RIGHT) ? _RightSpeed : _LeftSpeed; };
void Motors::Speed(const MotorID id, const float value)
{
    static const float min = -1.0, max = 1.0;

    if (id == MOTOR_RIGHT)
    {
        _RightSpeed = (value > max) ? max : (value < min) ? min : value;

        if (_RightSpeed > 0)
        {
            digitalWrite(RIGHT_PIN_OUT.DIR_A_PIN, LOW);
            digitalWrite(RIGHT_PIN_OUT.DIR_B_PIN, HIGH);
        }
        else
        {
            digitalWrite(RIGHT_PIN_OUT.DIR_A_PIN, HIGH);
            digitalWrite(RIGHT_PIN_OUT.DIR_B_PIN, LOW);
        }

        RIGHT_MOTOR_PWMVAL = map(
                (_RightSpeed > 0 ? _RightSpeed : -_RightSpeed) * 1599,
                0, 1599, 0, 1599);
    }
    else
    {
        _LeftSpeed = (value > max) ? max : (value < min) ? min : value;


        if (_LeftSpeed > 0)
        {
            digitalWrite(LEFT_PIN_OUT.DIR_A_PIN, LOW);
            digitalWrite(LEFT_PIN_OUT.DIR_B_PIN, HIGH);
        }
        else
        {
            digitalWrite(LEFT_PIN_OUT.DIR_A_PIN, HIGH);
            digitalWrite(LEFT_PIN_OUT.DIR_B_PIN, LOW);
        }

        LEFT_MOTOR_PWMVAL = map(
                ((_LeftSpeed > 0) ? _LeftSpeed : -_LeftSpeed) * 1599,
                0, 1599, 0, 1599);
    }
};

void Motors::BalancedMotorsSpeed(const MotorID referenceMotor, const float speed)
{
    MotorPinOut master = (referenceMotor == MOTOR_RIGHT) ? RIGHT_PIN_OUT : LEFT_PIN_OUT,
            slave = (referenceMotor == MOTOR_RIGHT) ? LEFT_PIN_OUT : RIGHT_PIN_OUT;

    int masterADC = analogRead(master.ADC_PIN),
            slaveADC = analogRead(slave.ADC_PIN);

    if (masterADC == 0)
    {
        masterADC = 1;
    }

    float masterSpeed = speed, slaveSpeed = speed + (slaveADC * masterSpeed / masterADC);

    Serial.print(masterADC); Serial.print(" ");
    Serial.print(slaveADC); Serial.print(" ");
    Serial.print(masterSpeed); Serial.print(" ");
    Serial.println(slaveSpeed);

    MOTORS.Speed((referenceMotor == MOTOR_RIGHT) ? MOTOR_RIGHT : MOTOR_LEFT, masterSpeed);
    MOTORS.Speed((referenceMotor == MOTOR_RIGHT) ? MOTOR_LEFT : MOTOR_RIGHT, slaveSpeed);

    delay(40);
}

void Motors::RunSpeedScript(const unsigned int numberOfSteps, const float * leftMotorSpeeds,
        const float * rightMotorSpeeds, const unsigned long * triggerTimes,
        const unsigned long firstPassTime)
{
    int i = 0;
    unsigned long currentTime = millis();
    while (i < numberOfSteps)
    {
        if (firstPassTime + triggerTimes[i] > currentTime)
        {
            MOTORS.Speed(MOTOR_LEFT, leftMotorSpeeds[i]);
            MOTORS.Speed(MOTOR_RIGHT, rightMotorSpeeds[i]);
            break;
        }

        i++;
    }
}