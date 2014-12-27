
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

#ifndef INB4_H
#define INB4_H

#ifndef SKETCH
#include "/usr/share/arduino/hardware/arduino/cores/arduino/Arduino.h"

#define USBCON
#include "/usr/share/arduino/hardware/arduino/cores/arduino/USBAPI.h"
#include "/usr/share/arduino/libraries/SoftwareSerial/SoftwareSerial.h"
#include "/usr/share/arduino/hardware/arduino/variants/mega/pins_arduino.h"

#include "/usr/share/arduino/hardware/tools/avr/lib/avr/include/avr/iomxx0_1.h"
#include "/usr/share/arduino/hardware/tools/avr/lib/avr/include/math.h"

//#include "/usr/share/arduino/hardware/tools/avr/lib/avr/include/avr/interrupt.h"
//#include "/usr/share/arduino/hardware/tools/avr/lib/avr/include/avr/io.h"
#endif

typedef unsigned char uint8_t;

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- INB4 CLASS -------------------------------------------------------- ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

typedef struct LimitSwitch LimitSwitch;
typedef struct ReedSwitch ReedSwtich;
typedef struct XBeeComm XBeeComm;
typedef struct OpticalSensor OpticalSensor;

typedef struct MotorPinOut MotorPinOut;
typedef struct Motors Motors;

namespace States {

    typedef struct ChargementBatterie ChargementBatterie;
    typedef struct NouvellePocheDansMagasin NouvellePocheDansMagasin;

    typedef struct ParcoursAvecLanceur ParcoursAvecLanceur;
    typedef struct MonteeAvecLanceur MonteeAvecLanceur;
    typedef struct DecenteAvecLanceur DecenteAvecLanceur;

    typedef struct VirageEntreeZoneLancement VirageEntreeZoneLancement;
    typedef struct AlignmentReedSwitchZoneLancement AlignmentReedSwitchZoneLancement;
    typedef struct DeploiementCanon DeploiementCanon;

    typedef struct DeploiementLimitSwitchs DeploiementLimitSwitchs;
    typedef struct VerificationDeclenchementLimitSwitchs VerificationDeclenchementLimitSwitchs;

    typedef struct SequenceCalculRotationCible SequenceCalculRotationCible;
    typedef struct SequenceLancementPoches SequenceLancementPoches;

    typedef struct SeparationLanceur SeparationLanceur;
    typedef struct VirageSortieZoneLancement VirageSortieZoneLancement;

    typedef struct ParcoursSansLanceur ParcoursSansLanceur;
    typedef struct MonteeSansLanceur MonteeSansLanceur;
    typedef struct DecenteSansLanceur DecenteSansLanceur;

    const unsigned short CHARGEMENT_BATTERIE_STATE_ID = 1;
    const unsigned short NOUVELLE_POCHE_DANS_MAGASIN_STATE_ID = 2;

    const unsigned short PARCOURS_AVEC_LANCEUR_STATE_ID = 3;
    const unsigned short MONTEE_AVEC_LANCEUR_STATE_ID = 4;
    const unsigned short DECENTE_AVEC_LANCEUR_STATE_ID = 5;

    const unsigned short VIRAGE_ENTREE_ZONE_LANCEMENT_STATE_ID = 6;
    const unsigned short ALIGNEMENT_REEDSWITCH_ZONE_LANCEMENT_STATE_ID = 7;
    const unsigned short DEPLOIEMENT_CANON_STATE_ID = 8;

    const unsigned short DEPLOIEMENT_LIMITSWITCHS_STATE_ID = 9;
    const unsigned short VERIFICATION_DECLENCHEMENT_LIMITSWITCHS_STATE_ID = 10;

    const unsigned short SEQUENCE_CALCUL_ROTATION_CIBLE_STATE_ID = 11;
    const unsigned short SEQUENCE_LANCEMENT_POCHES_STATE_ID = 12;

    const unsigned short SEPARATION_LANCEUR_STATE_ID = 13;
    const unsigned short VIRAGE_SORTIE_ZONE_LANCEMENT_STATE_ID = 14;

    const unsigned short PARCOURS_SANS_LANCEUR_STATE_ID = 15;
    const unsigned short MONTEE_SANS_LANCEUR_STATE_ID = 16;
    const unsigned short DECENTE_SANS_LANCEUR_STATE_ID = 17;
}

enum MotorID { MOTOR_LEFT = 1, MOTOR_RIGHT = 2 };
enum DirectionID { TURN_LEFT = 0, TURN_RIGHT = 1, FORWARD = 2, BACKWARD = 3 };

class INB4 {
public:
    static void setup();
    static void loop();
    static void showXBeeCommRead(const int &read);
};

extern long int LEFT_MOTOR_PWMVAL;
extern long int RIGHT_MOTOR_PWMVAL;

extern XBeeComm XBEECOMM;
extern LimitSwitch LIMIT_SWITCH;

extern float OPTICAL_SENSOR_MIN_ACTIVE_VALUE;
extern OpticalSensor OPTICAL_SENSOR;

extern float RIGHT_MOTOR_RATIO;
extern MotorPinOut LEFT_MOTOR_PIN_OUT;
extern MotorPinOut RIGHT_MOTOR_PIN_OUT;
extern Motors MOTORS;

extern unsigned short CURRENT_STATES_ID;

namespace States {

    extern ChargementBatterie CHARGEMENT_BATTERIE;
    extern NouvellePocheDansMagasin NOUVELLE_POCHE_DANS_MAGASIN;

    extern ParcoursAvecLanceur PARCOURS_AVEC_LANCEUR;
    extern MonteeAvecLanceur MONTEE_AVEC_LANCEUR;
    extern DecenteAvecLanceur DECENTE_AVEC_LANCEUR;

    extern VirageEntreeZoneLancement VIRAGE_ENTREE_ZONE_LANCEMENT;
    extern AlignmentReedSwitchZoneLancement ALIGNEMENT_REEDSWITCH_ZONE_LANCEMENT;
    extern DeploiementCanon DEPLOIEMENT_CANON;

    extern DeploiementLimitSwitchs DEPLOIEMENT_LIMITSWITCHS;
    extern VerificationDeclenchementLimitSwitchs VERIFICATION_DECLENCHEMENT_LIMITSWITCHS;

    extern SequenceCalculRotationCible SEQUENCE_CALCUL_ROTATION_CIBLE;
    extern SequenceLancementPoches SEQUENCE_LANCEMENT_POCHES;

    extern SeparationLanceur SEPARATION_LANCEUR;
    extern VirageSortieZoneLancement VIRAGE_SORTIE_ZONE_LANCEMENT;

    extern ParcoursSansLanceur PARCOURS_SANS_LANCEUR;
    extern MonteeSansLanceur MONTEE_SANS_LANCEUR;
    extern DecenteSansLanceur DECENTE_SANS_LANCEUR;
}

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- OPTICAL SENSOR ---------------------------------------------------- ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

struct OpticalSensor {
    const uint8_t ANALOG_VO_PIN;
    OpticalSensor(uint8_t ANALOG_VO_PIN)
            : ANALOG_VO_PIN(ANALOG_VO_PIN), _Active(false), _AnalogValue(0.0) {};

    void Setup() { pinMode(ANALOG_VO_PIN, INPUT); };

    const float & ReadInputWithoutBounce()
    {
        _lastRead = millis();
        _AnalogValue = analogRead(ANALOG_VO_PIN) * (5.0 / 1023.0);
        _Active = (_AnalogValue > OPTICAL_SENSOR_MIN_ACTIVE_VALUE);

        return _AnalogValue;
    }

    const float & ReadInputWithBounce()
    {
        if (_lastRead + BOUNCING_VALUE < millis())
        {
            _lastRead = millis();
            _AnalogValue = analogRead(ANALOG_VO_PIN) * (5.0 / 1023.0);
            _Active = (_AnalogValue > OPTICAL_SENSOR_MIN_ACTIVE_VALUE);
        }

        return _AnalogValue;
    };

    const float & AnalogValue() { return _AnalogValue; };

    const bool IsActiveWithoutBounce()
    {
        ReadInputWithoutBounce();
        return _Active;
    }

    const bool IsActiveWithBounce()
    {
        ReadInputWithBounce();
        return _Active;
    };

private:
    static const unsigned long BOUNCING_VALUE = 10;
    float _AnalogValue;
    bool _Active;
    unsigned long _lastRead;
};



///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- LIMIT SWITCH ------------------------------------------------------ ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

struct LimitSwitch {
    const uint8_t PIN;
    LimitSwitch(uint8_t PIN) : PIN(PIN), active(false) {};
    void Setup() { pinMode(PIN, INPUT_PULLUP); };
    const bool IsActive() const { return active; };
    const bool ReadInput()
    {
        active = (digitalRead(PIN) == 0);
        return IsActive();
    };

private:
    bool active;
};

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- REED SWITCH ------------------------------------------------------- ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

struct ReedSwitch {
    const uint8_t PIN;
    ReedSwitch(uint8_t PIN) : PIN(PIN), active(false) {};
    void Setup() { pinMode(PIN, INPUT_PULLUP); };
    const bool IsActive() const { return active; };
    const bool ReadInput()
    {
        active = (digitalRead(PIN) == 0);
        return IsActive();
    };

private:
    bool active;
};

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- XBEE COMMUNICATION ------------------------------------------------ ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

struct XBeeComm : SoftwareSerial {
    const uint8_t RX_PIN, TX_PIN;
    const long COMM_BAUDRATE;
    XBeeComm(uint8_t RX_PIN, uint8_t TX_PIN, uint16_t COMM_BAUDRATE)
            : SoftwareSerial(RX_PIN, TX_PIN), RX_PIN(RX_PIN), TX_PIN(TX_PIN),
              COMM_BAUDRATE(COMM_BAUDRATE) {};
    void Setup() { begin(COMM_BAUDRATE); delayMicroseconds(100); };
    const char ReadInput() { return ( available() ) ? read() : 0; };
};

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- MOTORS ------------------------------------------------------------ ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

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
    void Setup()
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

    const float Speed(const MotorID id) const { return (id == MOTOR_RIGHT) ? _RightSpeed : _LeftSpeed; };
    void Speed(const MotorID id, const float value)
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

    void BalancedMotorsSpeed(const MotorID referenceMotor, const float speed)
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

    void RunSpeedScript(const unsigned int numberOfSteps, const float * leftMotorSpeeds,
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


private:
    long int &_RightPWM, &_LeftPWN;
    float _RightSpeed, _LeftSpeed;
};

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- MACHINE STATES ---------------------------------------------------- ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

namespace States {

    struct ChargementBatterie {

        ChargementBatterie() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };

    struct NouvellePocheDansMagasin {

        NouvellePocheDansMagasin() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };
}

namespace States {

    struct ParcoursAvecLanceur {

        ParcoursAvecLanceur() : active(false), firstpass(true) {};

        void Update() {

            if (firstpass)
            {
                time = millis();
                firstpass = false;
            }

            if (OPTICAL_SENSOR.IsActiveWithBounce())
            {
                CURRENT_STATES_ID = MONTEE_AVEC_LANCEUR_STATE_ID;
                active = false;
            }
            else
            {
                active = true;
            }
        };

        void Execute() {

            MOTORS.Speed(MOTOR_LEFT, 0.35);
            MOTORS.Speed(MOTOR_RIGHT, 0.35);
        };

        const bool & IsActive() { return active; };

    private:
        bool active;
        bool firstpass;
        unsigned long time;
    };

    struct MonteeAvecLanceur {

        MonteeAvecLanceur() : active(false), sommetDosDane(false) {};

        void Update() {

            /*
                    27 DEC 00:35 -> le char rentre dans la pente et change d'etat (le vehicule stop) presque immidiatement apr;es



             */


            if (!OPTICAL_SENSOR.IsActiveWithBounce()
                    && _FirstExecuteTime + 2000 > millis())
            {
                sommetDosDane = true;
            }
            else if (OPTICAL_SENSOR.IsActiveWithoutBounce() && sommetDosDane)
            {
                CURRENT_STATES_ID = DECENTE_AVEC_LANCEUR_STATE_ID;
                active = false;
            }
            else
            {
                active = true;
            }
        };

        void Execute() {

            if (_FirstExecuteTime == 0)
            {
                _FirstExecuteTime = millis();
            }

            MOTORS.Speed(MOTOR_LEFT, 0.35);
            MOTORS.Speed(MOTOR_RIGHT, 0.35);
        };

        const bool & IsActive() { return active; };

    private:
        unsigned long _FirstExecuteTime = 0;
        bool active;
        bool sommetDosDane;
    };

    struct DecenteAvecLanceur {

        DecenteAvecLanceur() : active(false) {};

        void Update()
        {
            if (!OPTICAL_SENSOR.IsActiveWithBounce()
                    || _FirstExecuteTime + 1500 > millis())
            {
                CURRENT_STATES_ID = VIRAGE_ENTREE_ZONE_LANCEMENT_STATE_ID;
                active = false;
            }
            else
            {
                active = true;
            }
        };

        void Execute()
        {
            if (_FirstExecuteTime == 0)
            {
                _FirstExecuteTime = millis();
            }

            MOTORS.Speed(MOTOR_LEFT, -0.34);
            MOTORS.Speed(MOTOR_RIGHT, -0.34);
        };

        const bool & IsActive() { return active; };

    private:
        unsigned long _FirstExecuteTime = 0;
        bool active;
    };
}

namespace States {

    struct VirageEntreeZoneLancement {

        VirageEntreeZoneLancement() : active(false), firstPass(true),
            switchNextState(false) {};

        void Update()
        {
            if (switchNextState)
            {
                CURRENT_STATES_ID = ALIGNEMENT_REEDSWITCH_ZONE_LANCEMENT_STATE_ID;
                active = false;
            }
            else
            {
                active = true;
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
            static const unsigned int nbStep            = 10;
            static const float leftSpeeds[nbStep]       = {
                    0.0, -0.14, 0.22, -0.45, -0.80, 0.0,
                    -0.60, 0.0, 0.35,

                    0.0
            };

            static const float rightSpeeds[nbStep]      = {
                    0.0, -0.14, 0.22, 0.55, 0.20, 0.70,
                    0.0, 0.50, 0.35,

                    0.0
            };

            static const unsigned long timing[nbStep]   = {
                    500, 1500, 1650, 1900, 2000, 2100,
                    2300, 2450, 2950,

                    2952
            };

            MOTORS.RunSpeedScript(nbStep, leftSpeeds, rightSpeeds, timing, startTime);

        };

        const bool & IsActive() { return active; };

    private:
        bool switchNextState;
        bool active;
        bool firstPass;
        unsigned long time;

    };

    struct AlignmentReedSwitchZoneLancement {

        AlignmentReedSwitchZoneLancement() {};

        void Update() {

            active = true;
        };

        void Execute() {

            MOTORS.Speed(MOTOR_LEFT, 0.00);
            MOTORS.Speed(MOTOR_RIGHT, 0.00);
        };

        const bool & IsActive() { return active; };

    private:
        bool active;

    };

    struct DeploiementCanon {

        DeploiementCanon() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };
}

namespace States {

    struct DeploiementLimitSwitchs {

        DeploiementLimitSwitchs() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };

    struct VerificationDeclenchementLimitSwitchs {

        VerificationDeclenchementLimitSwitchs() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };
}

namespace States {

    struct SequenceCalculRotationCible {

        SequenceCalculRotationCible() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };

    struct SequenceLancementPoches {

        SequenceLancementPoches() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };
}

namespace States {

    struct SeparationLanceur {

        SeparationLanceur() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };

    struct VirageSortieZoneLancement {

        VirageSortieZoneLancement() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };
}

namespace States {

    struct ParcoursSansLanceur {

        ParcoursSansLanceur() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };

    struct MonteeSansLanceur {

        MonteeSansLanceur() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };

    struct DecenteSansLanceur {

        DecenteSansLanceur() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };
}


#endif INB4_H

class MotorRebalancing
{
public:

private:

};



long int LEFT_MOTOR_PWMVAL = 0;
long int RIGHT_MOTOR_PWMVAL = 0;

XBeeComm XBEECOMM               = XBeeComm(10, 9, 57600);
LimitSwitch LIMIT_SWITCH        = LimitSwitch(46);

float OPTICAL_SENSOR_MIN_ACTIVE_VALUE   = 4.85;
OpticalSensor OPTICAL_SENSOR            = OpticalSensor(A12);

float RIGHT_MOTOR_RATIO = 0.65;
float MOTOR_MIN_SPEED = 0.25, MOTOR_MAX_SPEED = 1.0;

MotorPinOut LEFT_MOTOR_PIN_OUT   = MotorPinOut(A0, A1, 11, A2);
MotorPinOut RIGHT_MOTOR_PIN_OUT  = MotorPinOut(A5, A4, 12, A3);
Motors MOTORS                    = Motors(RIGHT_MOTOR_PIN_OUT, LEFT_MOTOR_PIN_OUT);


unsigned short CURRENT_STATES_ID = States::PARCOURS_AVEC_LANCEUR_STATE_ID;

namespace States {

    ChargementBatterie CHARGEMENT_BATTERIE = ChargementBatterie();
    NouvellePocheDansMagasin NOUVELLE_POCHE_DANS_MAGASIN = NouvellePocheDansMagasin();

    ParcoursAvecLanceur PARCOURS_AVEC_LANCEUR = ParcoursAvecLanceur();
    MonteeAvecLanceur MONTEE_AVEC_LANCER = MonteeAvecLanceur();
    DecenteAvecLanceur DECENTE_AVEC_LANCER = DecenteAvecLanceur();

    VirageEntreeZoneLancement VIRAGE_ENTREE_ZONE_LANCEMENT = VirageEntreeZoneLancement();
    AlignmentReedSwitchZoneLancement ALIGNEMENT_REEDSWITCH_ZONE_LANCEMENT = AlignmentReedSwitchZoneLancement();
    DeploiementCanon DEPLOIEMENT_CANON = DeploiementCanon();

    DeploiementLimitSwitchs DEPLOIEMENT_LIMITSWITCHS = DeploiementLimitSwitchs();
    VerificationDeclenchementLimitSwitchs VERIFICATION_DECLENCHEMENT_LIMITSWITCHS = VerificationDeclenchementLimitSwitchs();

    SequenceCalculRotationCible SEQUENCE_CALCUL_ROTATION_CIBLE = SequenceCalculRotationCible();
    SequenceLancementPoches SEQUENCE_LANCEMENT_POCHES = SequenceLancementPoches();

    SeparationLanceur SEPARATION_LANCEUR = SeparationLanceur();
    VirageSortieZoneLancement VIRAGE_SORTIE_ZONE_LANCEMENT = VirageSortieZoneLancement();

    ParcoursSansLanceur PARCOURS_SANS_LANCEUR = ParcoursSansLanceur();
    MonteeSansLanceur MONTEE_SANS_LANCEUR = MonteeSansLanceur();
    DecenteSansLanceur DECENTE_SANS_LANCEUR = DecenteSansLanceur();
};

void INB4::setup() {

    XBEECOMM.begin(57600);
    Serial.begin(9600);
    delayMicroseconds(100);

    MOTORS.Setup();
    OPTICAL_SENSOR.Setup();
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

    CURRENT_STATES_ID = 0;


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



    //Serial.print(OPTICAL_SENSOR.ReadInput()); Serial.println("v");


    //MOTORS.Speed(MOTOR_RIGHT, 0.60);
    //MOTORS.Speed(MOTOR_LEFT, 0.60);



    //delay(50);


    ///////////////////////////////////////////////////////////////////////////
    ///                                                                     ///
    ///                                                                     ///
    ///////////////////////////////////////////////////////////////////////////

    switch (CURRENT_STATES_ID){

        case States::CHARGEMENT_BATTERIE_STATE_ID :
        {
            States::CHARGEMENT_BATTERIE.Update();
            if (States::CHARGEMENT_BATTERIE.IsActive())
            {
                States::CHARGEMENT_BATTERIE.Execute();
            }

            break;
        }

        case States::NOUVELLE_POCHE_DANS_MAGASIN_STATE_ID :
        {
            States::NOUVELLE_POCHE_DANS_MAGASIN.Update();
            if (States::NOUVELLE_POCHE_DANS_MAGASIN.IsActive())
            {
                States::NOUVELLE_POCHE_DANS_MAGASIN.Execute();
            }

            break;
        }


        case States::PARCOURS_AVEC_LANCEUR_STATE_ID:
        {
            States::PARCOURS_AVEC_LANCEUR.Update();
            if (States::PARCOURS_AVEC_LANCEUR.IsActive())
            {
                States::PARCOURS_AVEC_LANCEUR.Execute();
            }

            break;
        };

        case States::MONTEE_AVEC_LANCEUR_STATE_ID :
        {
            States::MONTEE_AVEC_LANCER.Update();
            if (States::MONTEE_AVEC_LANCER.IsActive())
            {
                States::MONTEE_AVEC_LANCER.Execute();
            }

            break;
        };

        case States::DECENTE_AVEC_LANCEUR_STATE_ID :
        {
            States::DECENTE_AVEC_LANCER.Update();
            if (States::DECENTE_AVEC_LANCER.IsActive())
            {
                States::DECENTE_AVEC_LANCER.Execute();
            }

            break;
        };


        case States::VIRAGE_ENTREE_ZONE_LANCEMENT_STATE_ID :
        {
            States::VIRAGE_ENTREE_ZONE_LANCEMENT.Update();
            if (States::VIRAGE_ENTREE_ZONE_LANCEMENT.IsActive())
            {
                States::VIRAGE_ENTREE_ZONE_LANCEMENT.Execute();
            }

            break;
        };

        case States::ALIGNEMENT_REEDSWITCH_ZONE_LANCEMENT_STATE_ID :
        {
            States::ALIGNEMENT_REEDSWITCH_ZONE_LANCEMENT.Update();
            if (States::ALIGNEMENT_REEDSWITCH_ZONE_LANCEMENT.IsActive())
            {
                States::ALIGNEMENT_REEDSWITCH_ZONE_LANCEMENT.Execute();
            }

            break;
        };

        case States::DEPLOIEMENT_CANON_STATE_ID :
        {
            States::DEPLOIEMENT_CANON.Update();
            if (States::DEPLOIEMENT_CANON.IsActive())
            {
                States::DEPLOIEMENT_CANON.Execute();
            }

            break;
        };

        case States::DEPLOIEMENT_LIMITSWITCHS_STATE_ID :
        {
            States::DEPLOIEMENT_LIMITSWITCHS.Update();
            if (States::DEPLOIEMENT_LIMITSWITCHS.IsActive())
            {
                States::DEPLOIEMENT_LIMITSWITCHS.Execute();
            }

            break;
        };

        case States::VERIFICATION_DECLENCHEMENT_LIMITSWITCHS_STATE_ID :
        {
            States::VERIFICATION_DECLENCHEMENT_LIMITSWITCHS.Update();
            if (States::VERIFICATION_DECLENCHEMENT_LIMITSWITCHS.IsActive())
            {
                States::VERIFICATION_DECLENCHEMENT_LIMITSWITCHS.Execute();
            }

            break;
        };

        case States::SEQUENCE_CALCUL_ROTATION_CIBLE_STATE_ID :
        {
            States::SEQUENCE_CALCUL_ROTATION_CIBLE.Update();
            if (States::SEQUENCE_CALCUL_ROTATION_CIBLE.IsActive())
            {
                States::SEQUENCE_CALCUL_ROTATION_CIBLE.Execute();
            }

            break;
        };

        case States::SEQUENCE_LANCEMENT_POCHES_STATE_ID:
        {
            States::SEQUENCE_LANCEMENT_POCHES.Update();
            if (States::SEQUENCE_LANCEMENT_POCHES.IsActive())
            {
                States::SEQUENCE_LANCEMENT_POCHES.Execute();
            }

            break;
        };


        case States::SEPARATION_LANCEUR_STATE_ID :
        {
            States::SEPARATION_LANCEUR.Update();
            if (States::SEPARATION_LANCEUR.IsActive())
            {
                States::SEPARATION_LANCEUR.Execute();
            }

            break;
        };

        case States::VIRAGE_SORTIE_ZONE_LANCEMENT_STATE_ID :
        {
            States::VIRAGE_SORTIE_ZONE_LANCEMENT.Update();
            if (States::VIRAGE_SORTIE_ZONE_LANCEMENT.IsActive())
            {
                States::VIRAGE_SORTIE_ZONE_LANCEMENT.Execute();
            }

            break;
        };


        case States::PARCOURS_SANS_LANCEUR_STATE_ID :
        {
            States::PARCOURS_SANS_LANCEUR.Update();
            if (States::PARCOURS_SANS_LANCEUR.IsActive())
            {
                States::PARCOURS_SANS_LANCEUR.Execute();
            }

            break;
        };

        case States::MONTEE_SANS_LANCEUR_STATE_ID :
        {
            States::MONTEE_SANS_LANCEUR.Update();
            if (States::MONTEE_SANS_LANCEUR.IsActive())
            {
                States::MONTEE_SANS_LANCEUR.Execute();
            }

            break;
        };

        case States::DECENTE_SANS_LANCEUR_STATE_ID :
        {
            States::DECENTE_SANS_LANCEUR.Update();
            if (States::DECENTE_SANS_LANCEUR.IsActive())
            {
                States::DECENTE_SANS_LANCEUR.Execute();
            }

            break;
        };

        default:
            break;
    };




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



