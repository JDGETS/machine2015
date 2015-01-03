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
                delay(3000); // Ignore IR for a while (station de recharge)
                bottomSensor.WaitForDetect();
                CHECK_FORCE_STOP_MACRO
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
                bottomSensor.WaitForUndetect();
                CHECK_FORCE_STOP_MACRO
                delay(200);
                Serial.println("MonteeAvecLanceur - Waiting for detect...");
                bottomSensor.WaitForDetect();
                CHECK_FORCE_STOP_MACRO
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
                Serial.println("DecenteAvecLanceur - Waiting for undetect... ");
                bottomSensor.WaitForUndetect();
                CHECK_FORCE_STOP_MACRO
                motors.Speed(MOTOR_LEFT, 0);
                motors.Speed(MOTOR_RIGHT, 0);
                delay(1000);
                motors.Speed(MOTOR_LEFT, -0.40);
                motors.Speed(MOTOR_RIGHT, -0.40);
                delay(500);
                Serial.println("DecenteAvecLanceur - Waiting for detect... ");
                bottomSensor.WaitForDetect();
                CHECK_FORCE_STOP_MACRO
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
                static const unsigned int nbStep            = 4;

                static const float leftSpeeds[nbStep]       = { 0.30, -0.25, -0.05, 0.30 };

                static const float rightSpeeds[nbStep]      = { 0.30, 0.55, 0.45, 0.30 };

                static const unsigned long timing[nbStep]   = { 100, 1900, 3100, 3102 };

                motors.RunSpeedScript(nbStep, leftSpeeds, rightSpeeds, timing, startTime);

                CHECK_FORCE_STOP_MACRO

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
            const static unsigned long TARGET_MASK = (TARGET << 1) - 1; // If you want to make it 0b0100, you Have to use target mask 0b0111.

            AlignmentReedSwitchZoneLancement() : State(){};

            void Execute() {
                unsigned long value = reedswitches.GetValue();
                unsigned long impulsion_time = 100;
                unsigned long lastTargetCheck = millis() - impulsion_time - 1;
                float MIN_SPEED = 0.25;
                float speed = MIN_SPEED;
                float lastSpeed = speed;
                Serial.println("starting");
                while(value != TARGET){
                    if(impulsion_time < millis()-lastTargetCheck){
                        //This loop will position the vehicle, step by step (w/ impulsions).
                        motors.Speed(MOTOR_LEFT, 0);
                        motors.Speed(MOTOR_RIGHT, 0);
                            Serial.println(value,BIN);
                            Serial.println(TARGET,BIN);
                        if(value == 0){
                            //We lost the signal, let's continue what we were doing.
                            speed = lastSpeed;
                        }
                        else if(value > TARGET_MASK){
                            speed = - MIN_SPEED;
                        }
                        else if(value < TARGET_MASK){
                            speed = + MIN_SPEED;
                        }
                        else{
                            speed = 0;
                        }

                        delay(100);
                        motors.Speed(MOTOR_LEFT, speed);
                        motors.Speed(MOTOR_RIGHT, speed);
                        lastTargetCheck = millis();
                        lastSpeed = speed;
                        CHECK_FORCE_STOP_MACRO
                    }
                    value = reedswitches.GetValue();
                }
                motors.Speed(MOTOR_LEFT, 0);
                motors.Speed(MOTOR_RIGHT, 0);

                End();
            };

        };
    }
}
#endif