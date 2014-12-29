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
            ParcoursAvecLanceur() : State(), firstpass(true) {};

            void Update() {

                if (firstpass)
                {
                    time = millis();
                    firstpass = false;
                }

                if (bottomOpticalSensor.LongReadInput(false))
                {
                    End();
                    delay(20);
                }
            };

            void Execute() {
                motors.Speed(MOTOR_LEFT, 0.35);
                motors.Speed(MOTOR_RIGHT, 0.35);
            };

        private:
            bool firstpass;
            unsigned long time;
        };

        class MonteeAvecLanceur : public State {
        public:
            MonteeAvecLanceur() : State(), sommetDosDane(false), _FirstExecuteTime(0) {};

            void Update() {

                /*
                        27 DEC 00:35 -> le char rentre dans la pente et change d'etat (le vehicule stop) presque immidiatement apr;es
                 */


                if (!bottomOpticalSensor.LongReadInput(true))
                {
                    sommetDosDane = true;
                }
                else if (bottomOpticalSensor.LongReadInput(false) && sommetDosDane)
                {
                    End();
                    delay(20);
                }
            };

            void Execute() {
                motors.Speed(MOTOR_LEFT, 0.35);
                motors.Speed(MOTOR_RIGHT, 0.35);
            };

        private:
            unsigned long _FirstExecuteTime;
            bool active;
            bool sommetDosDane;
        };

        class DecenteAvecLanceur : public State {
        public:
            DecenteAvecLanceur() : State(), _FirstExecuteTime(0) {};

            void Update()
            {
                if (!bottomOpticalSensor.LongReadInput(true))
                {
                    End();
                    delay(20);
                }
            };

            void Execute()
            {
                motors.Speed(MOTOR_LEFT, -0.34);
                motors.Speed(MOTOR_RIGHT, -0.34);
            };

        private:
            unsigned long _FirstExecuteTime;
        };
    }


    namespace States {

        class VirageEntreeZoneLancement : public State {
        public:
            VirageEntreeZoneLancement() : State(), firstPass(true),
                switchNextState(false) {};

            void Update()
            {
                if (switchNextState) // TO-DO: ReedSwitch detected
                {
                    End();
                }
            };

            void Execute()
            {
                static bool firstExecute = true;
                static bool turnTriggered = false;
                static unsigned long turnTime = 0;

                /*
                        26 DEC - 23:57 -> la roue de droite a pogner le rebors alors certain event non pas peut faire se qu'il devait faire a cause de ca ex: reculer avec la roue de droit
                        27 DEC - 00:04 -> la dernier step de RunSpeedScript n'a pas etait pris en compte et l'avant dernier step est rester indefiniment et les dernier step a etait ignorer
                        27 DEC - 00:09 -> la roue de gauche a pogner le rebors alors certain event non pas pu faire se qu'il devait faire
                 */


                static const unsigned long startTime        = millis();
                static const unsigned int nbStep            = 7;
                static const float leftSpeeds[nbStep]       = {
                        0.0, 0.0,
                        -0.13, 0.0,
                        0.0, 0.0,
                        0.0
                };

                static const float rightSpeeds[nbStep]      = {
                        0.0, 0.0,
                        -0.13, 0.0,
                        0.35, 0.0,
                        0.0
                };

                static const unsigned long timing[nbStep]   = {
                        500, 1500,
                        2500, 3500,
                        5500, 6500,
                        6501
                };
            /*
                static const unsigned long startTime        = millis();
                static const unsigned int nbStep            = 19;
                static const float leftSpeeds[nbStep]       = {
                        0.0,    0.0,
                        -0.14,  0.0,
                        0.22,   0.0,
                        -0.45,  0.0,
                        -0.80,  0.0,
                        0.0,    0.0,
                        -0.60,  0.0,
                        0.0,    0.0,
                        0.35,   0.0,
                        0.0
                 };                  };
             
                static const float rightSpeeds[nbStep]      = {
                        0.0,    0.0,
                        -0.14,  0.0,
                        0.22,   0.0,
                        0.55,   0.0,
                        0.20,   0.0,
                        0.70,   0.0,
                        0.0,    0.0,
                        0.50,   0.0,
                        0.35,   0.0,
                        0.0
                };

                static const unsigned long timing[nbStep]   = {
                        500,    1500,
                        2500,   3500,
                        3650,   4650,
                        4900,   5900,
                        6000,   7000,
                        7100,   8100,
                        8300,   9300,
                        9450,   10450,
                        10950,  11950,
                        11952
                };
            */

                motors.RunSpeedScript(nbStep, leftSpeeds, rightSpeeds, timing, startTime);

            };

        private:
            bool switchNextState;
            bool firstPass;
            unsigned long time;

        };

        class AlignmentReedSwitchZoneLancement : public State{
        public:
            AlignmentReedSwitchZoneLancement() : State(){};

            void Update() {

            };

            void Execute() {
                motors.Speed(MOTOR_LEFT, 0.00);
                motors.Speed(MOTOR_RIGHT, 0.00);
            };

        };
    }
}
#endif