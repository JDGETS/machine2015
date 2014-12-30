#ifndef STATEINB4_H
#define STATEINB4_H

#include "Hardware.h"
#include "Vehicle.h"
#include "AllStates.h"

typedef unsigned char uint8_t;

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- MACHINE STATES ---------------------------------------------------- ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////
namespace Vehicle{
    namespace States {

        class ParcoursAvecLanceur : public State {
        public:
            ParcoursAvecLanceur() : State() {};

            void Execute() {
                motors.Speed(MOTOR_LEFT, 0.40);
                motors.Speed(MOTOR_RIGHT, 0.40);
                Serial.println("ParcoursAvecLanceur - Waiting for detect...");
                bottomOpticalSensor.WaitForDetect();
                Serial.println(bottomOpticalSensor.AnalogValue());
                End();
            };
        };

        class MonteeAvecLanceur : public State {
        public:
            MonteeAvecLanceur() : State() {};

            void Execute() {
                motors.Speed(MOTOR_LEFT, 0.50);
                motors.Speed(MOTOR_RIGHT, 0.50);
                delay(200);
                Serial.println("MonteeAvecLanceur - Waiting for undetect...");
                bottomOpticalSensor.WaitForUndetect();
                Serial.println(bottomOpticalSensor.AnalogValue());
                delay(200);
                Serial.println("MonteeAvecLanceur - Waiting for detect...");
                bottomOpticalSensor.WaitForDetect();
                Serial.println(bottomOpticalSensor.AnalogValue());
                End();
            };
        };

        class DecenteAvecLanceur : public State {
        public:
            DecenteAvecLanceur() : State() {};

            void Execute()
            {
                motors.Speed(MOTOR_LEFT, -0.20);
                motors.Speed(MOTOR_RIGHT, -0.20);
                delay(200);
                Serial.print("DecenteAvecLanceur - Waiting for undetect... ");
                bottomOpticalSensor.WaitForUndetect();
                Serial.println(bottomOpticalSensor.AnalogValue());
                motors.Speed(MOTOR_LEFT, 0);
                motors.Speed(MOTOR_RIGHT, 0);
                delay(1000);
                motors.Speed(MOTOR_LEFT, -0.33);
                motors.Speed(MOTOR_RIGHT, -0.33);
                delay(500);
                Serial.print("DecenteAvecLanceur - Waiting for detect... ");
                bottomOpticalSensor.WaitForDetect();
                Serial.println(bottomOpticalSensor.AnalogValue());
                motors.Speed(MOTOR_LEFT, 0);
                motors.Speed(MOTOR_RIGHT, 0);

                End();
            };
        };
    
        class VirageEntreeZoneLancement : public State {
        public:
            VirageEntreeZoneLancement() : State() {};

            void Execute()
            {
                static const unsigned long startTime        = millis();
                static const unsigned int nbStep            = 3;

                static const float leftSpeeds[nbStep]       = { -0.25, 0.25, 0.30 };

                static const float rightSpeeds[nbStep]      = { 0.45, 0.45, 0.30 };

                static const unsigned long timing[nbStep]   = { 1900, 3100, 3102 };

                motors.RunSpeedScript(nbStep, leftSpeeds, rightSpeeds, timing, startTime);

                if (reedswitches.GetValue() != 0) 
                {
                    Serial.print("VirageEntreeZoneLancement - Detected Reedswitch. Ending state.");
                    End();
                }

                return;
            };

        };

        class AlignmentReedSwitchZoneLancement : public State{
        public:
            const static unsigned long TARGET = 0b0100;

            AlignmentReedSwitchZoneLancement() : State(){};

            void Execute() {
                unsigned long value = reedswitches.GetValue();
                float lastSpeed = 0;
                float speed = 0;
                float MIN_SPEED = 0.30;
                while(value != TARGET){
                    //This loop will position the vehicle, step by step (w/ impulsions).
                    motors.Speed(MOTOR_LEFT, 0);
                    motors.Speed(MOTOR_RIGHT, 0);
                    if(value == 0){
                        //We lost the signal, let's continue what we were doing.
                        speed = lastSpeed;
                    }
                    else if(value > TARGET){
                        speed = - MIN_SPEED;
                    }
                    else if(value < TARGET){
                        speed = + MIN_SPEED;
                    }
                    else{
                        speed = 0;
                    }

                    motors.Speed(MOTOR_LEFT, speed);
                    motors.Speed(MOTOR_RIGHT, speed);
                    delay(100);
                    lastSpeed = speed;
                }

                End();
            };

        };
    }
}
#endif