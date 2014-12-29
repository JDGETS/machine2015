/******************************************************
*
*   ROSE2, GRIS2, ..., GRIS1, ROSE1
*   BRUN = GND
*
*
*       SCRIPTING DE VITESSE
*       MINIMUM TIME BETWEEN STATE CHANGE
*
*
*******************************************************/
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Servo.h>

#include "Hardware.h"
#include "XBeeComm.h"
#include "Vehicle.h"
#include "AllStates.h"

// XBEECOM KEYBOARD BINDING
#define KEYBOARD_1 153
#define KEYBOARD_2 154
#define KEYBOARD_3 155
#define KEYBOARD_4 156
#define KEYBOARD_5 157
#define KEYBOARD_6 158
#define KEYBOARD_7 159
#define KEYBOARD_8 152

#define KEYBOARD_Q 185
#define KEYBOARD_W 191
#define KEYBOARD_A 177
#define KEYBOARD_S 187
#define KEYBOARD_D 180
#define KEYBOARD_E 181
#define KEYBOARD_Z 186
#define KEYBOARD_X 184
#define KEYBOARD_C 179

XBeeComm XBEECOMM               = XBeeComm(10, 9, 57600);

namespace INB4{
  void setup();
  void loop();
  void showXBeeCommRead(const int&);
}

void INB4::setup() {

    XBEECOMM.begin(57600);
    Serial.begin(9600);
    delayMicroseconds(100);

    Vehicle::Setup();
}

void INB4::loop() {

    static bool firstPass = true;

    static int xBeeRead;
    static float motorSpeed; static const float motorSpeedMin = 0.0, motorSpeedMax = 1.0;
    static float motorDirection; static const float motorDirectionMin = -1.0, motorDirectionMax = 1.0;

    if (firstPass)
    {

        firstPass = false;
    }

    xBeeRead = (XBEECOMM.available()) ? XBEECOMM.read() : 0;

    //showXBeeCommRead(xBeeRead);

    switch(xBeeRead) {

        case KEYBOARD_1:
        {
            motorSpeed = 0.50;
            MOTORS.Speed(MOTOR_LEFT, motorSpeed);
            MOTORS.Speed(MOTOR_RIGHT, motorSpeed);
            break;
        }

        case KEYBOARD_W:
        {
            motorSpeed += (motorSpeed < motorSpeedMax) ? 0.05 : 0.0;
            MOTORS.Speed(MOTOR_LEFT, motorSpeed);
            MOTORS.Speed(MOTOR_RIGHT, motorSpeed);
            break;
        };

        case KEYBOARD_S:
        {
            motorSpeed -= (motorSpeed > motorSpeedMin) ? 0.05  : 0.0;
            MOTORS.Speed(MOTOR_LEFT, motorSpeed);
            MOTORS.Speed(MOTOR_RIGHT, motorSpeed);
            break;
        };

        case KEYBOARD_A:
        {
           // motorDirection -= (motorDirection > motorDirectionMin) ? 0.05 : 0.0;

           // if (motorDirection < 0.0)
           // {
           //     MOTORS.Direction(TURN_LEFT, abs(motorDirection));
           // }
           // else
            //{
            //    MOTORS.Direction(TURN_RIGHT, motorDirection);
           // }

            break;
        };

        case KEYBOARD_D:
        {
           // motorDirection += (motorDirection < motorDirectionMax) ? 0.05  : 0.0;

           // if (motorDirection < 0.0)
           // {
            //    MOTORS.Direction(TURN_LEFT, abs(motorDirection));
           // }
            //else
           //{
                //MOTORS.Direction(TURN_RIGHT, motorDirection);
           //}

            break;
        };


        case KEYBOARD_Z:
        {
            motorSpeed = motorDirectionMax;
            MOTORS.Speed(MOTOR_LEFT, motorSpeed);
            MOTORS.Speed(MOTOR_RIGHT, motorSpeed);
            break;
        };

        case KEYBOARD_C:
        {
            motorSpeed = motorSpeedMin;
            MOTORS.Speed(MOTOR_LEFT, motorSpeed);
            MOTORS.Speed(MOTOR_RIGHT, motorSpeed);
            break;
        };

        default:
            break;
    }

    ///////////////////////////////////////////////////////////////////////////
    ///                                                                     ///
    ///                                                                     ///
    ///////////////////////////////////////////////////////////////////////////




    //CURRENT_STATES_ID = 0;

    //MOTORS.Speed(MOTOR_LEFT, -0.14);
    //MOTORS.Speed(MOTOR_RIGHT, -0.14);

    //const unsigned int nbStep               = 5;
    //const float leftSpeeds[nbStep]          = {0.30, 0.50, -0.50, -0.30, 0.00};
    //const float rightSpeeds[nbStep]         = {0.30, -0.50, 0.50, -0.30, 0.00};
    //const unsigned long timing[nbStep]      = {1000, 1500, 2000, 2500, 3000};
    //static const unsigned long startTime    = millis();

    //MOTORS.RunSpeedScript(nbStep, leftSpeeds, rightSpeeds, timing, startTime);




    /*
    MOTORS.Speed(MOTOR_LEFT, -0.290);
    MOTORS.Speed(MOTOR_RIGHT, -0.180);
    if (analogRead(MOTORS.RIGHT_PIN_OUT.ADC_PIN) > 2)
    {
        MOTORS.Speed(MOTOR_LEFT, 0.0);
    }
    */


    /*
    if ((millis() / 250) % 3 == 0)
    {
        MOTORS.Speed(MOTOR_LEFT, -0.45);
        MOTORS.Speed(MOTOR_RIGHT, -0.28);
        //Serial.println("||||||||||||||");
    }
    else
    {
        MOTORS.Speed(MOTOR_LEFT, 0.00);
        MOTORS.Speed(MOTOR_RIGHT, 0.00);
        //Serial.println(" ");
    }
    */



    //MOTORS.Speed(MOTOR_LEFT, -0.30);
    //MOTORS.Speed(MOTOR_RIGHT, -0.30);

    //MOTORS.BalancedMotorsSpeed(MOTOR_LEFT, 0.30);
    //MOTORS.Direction(TURN_LEFT);
    //MOTORS.Speed(MOTOR_RIGHT, -0.15);
    //MOTORS.Speed(MOTOR_LEFT, -0.15);


    //MOTORS.Speed(MOTOR_RIGHT, 0.30);
    //MOTORS.Speed(MOTOR_LEFT, 0.30);
    


    //Serial.println(analogRead(MOTORS.LEFT_PIN_OUT.ADC_PIN));


    //delay(1000);

    //MOTORS.Direction(FORWARD);
    //MOTORS.Speed(MOTOR_RIGHT, 0.00);
    //MOTORS.Speed(MOTOR_LEFT, 0.00);

    //delay(6000);

    //LEFT_MOTOR_PWMVAL = map(0.3 * 1599, 0, 1599, 0, 1599);
    //RIGHT_MOTOR_PWMVAL = map(0.3 * 1599, 0, 1599, 0, 1599);



    static bool lastOpticalSensorValue = false;

    /*CURRENT_STATES_ID = 0;


    lastOpticalSensorValue = OPTICAL_SENSOR.IsActiveWithoutBounce();

    //OPTICAL_SENSOR.ReadInput();
    if (OPTICAL_SENSOR.IsActiveWithoutBounce())
    {
        float value = OPTICAL_SENSOR.AnalogValue();

        if (value != 0.0)
        {
            Serial.println(value);
        }


    }
    else
    {
        float value = OPTICAL_SENSOR.AnalogValue();

        if (value != 0.0)
        {
           Serial.println(value);
        }
    }
    */



    //Serial.print(OPTICAL_SENSOR.ReadInput()); Serial.println("v");


    //MOTORS.Speed(MOTOR_RIGHT, 0.60);
    //MOTORS.Speed(MOTOR_LEFT, 0.60);



    //delay(50);


    ///////////////////////////////////////////////////////////////////////////
    ///                                                                     ///
    ///                                                                     ///
    ///////////////////////////////////////////////////////////////////////////

 
}

void INB4::showXBeeCommRead(const int &read) {

    switch(read) {
        case KEYBOARD_1: { XBEECOMM.println("KEYBOARD_1\n"); Serial.println("KEYBOARD_1\n"); break; };
        case KEYBOARD_2: { XBEECOMM.println("KEYBOARD_2\n"); Serial.println("KEYBOARD_2\n"); break; };
        case KEYBOARD_3: { XBEECOMM.println("KEYBOARD_3\n"); Serial.println("KEYBOARD_3\n"); break; };
        case KEYBOARD_4: { XBEECOMM.println("KEYBOARD_4\n"); Serial.println("KEYBOARD_4\n"); break; };
        case KEYBOARD_5: { XBEECOMM.println("KEYBOARD_5\n"); Serial.println("KEYBOARD_5\n"); break; };
        case KEYBOARD_6: { XBEECOMM.println("KEYBOARD_6\n"); Serial.println("KEYBOARD_6\n"); break; };
        case KEYBOARD_7: { XBEECOMM.println("KEYBOARD_7\n"); Serial.println("KEYBOARD_7\n"); break; };
        case KEYBOARD_8: { XBEECOMM.println("KEYBOARD_8\n"); Serial.println("KEYBOARD_8\n"); break; };

        case KEYBOARD_Q: { XBEECOMM.println("KEYBOARD_Q\n"); Serial.println("KEYBOARD_Q\n"); break; };
        case KEYBOARD_W: { XBEECOMM.println("KEYBOARD_W\n"); Serial.println("KEYBOARD_W\n"); break; };
        case KEYBOARD_A: { XBEECOMM.println("KEYBOARD_A\n"); Serial.println("KEYBOARD_A\n"); break; };
        case KEYBOARD_S: { XBEECOMM.println("KEYBOARD_S\n"); Serial.println("KEYBOARD_S\n"); break; };
        case KEYBOARD_D: { XBEECOMM.println("KEYBOARD_D\n"); Serial.println("KEYBOARD_D\n"); break; };
        case KEYBOARD_E: { XBEECOMM.println("KEYBOARD_E\n"); Serial.println("KEYBOARD_E\n"); break; };
        case KEYBOARD_Z: { XBEECOMM.println("KEYBOARD_Z\n"); Serial.println("KEYBOARD_Z\n"); break; };
        case KEYBOARD_X: { XBEECOMM.println("KEYBOARD_X\n"); Serial.println("KEYBOARD_X\n"); break; };
        case KEYBOARD_C: { XBEECOMM.println("KEYBOARD_C\n"); Serial.println("KEYBOARD_C\n"); break; };

        default:
        {
            if (read != 0)
            {
                Serial.println(read);
            }
            break;
        }

    }

    delay(20);
}

void setup(){ 
  INB4::setup(); 
}
void loop(){ 
  INB4::loop(); 
}
