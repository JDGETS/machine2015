
/******************************************************
*
*   BLANC = V_IN
*   BRUN = GND
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
}

enum MotorID { MOTOR_LEFT = 0, MOTOR_RIGHT = 1 };
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
extern OpticalSensor OPTICAL_SENSOR;

extern MotorPinOut LEFT_MOTOR_PIN_OUT;
extern MotorPinOut RIGHT_MOTOR_PIN_OUT;
extern Motors MOTORS;

extern unsigned short CURRENT_STATES_ID;

namespace States {

    extern ChargementBatterie CHARGEMENT_BATTERIE;
    extern NouvellePocheDansMagasin NOUVELLE_POCHE_DANS_MAGASIN;

    extern ParcoursAvecLanceur PARCOURS_AVEC_LANCEUR;
    extern MonteeAvecLanceur MONTEE_AVEC_LANCER;
    extern DecenteAvecLanceur DECENTE_AVEC_LANCER;

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
    extern MonteeSansLanceur MONTEE_SANS_LANCER;
    extern DecenteSansLanceur DECENTE_SANS_LANCER;
}

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- OPTICAL SENSOR ---------------------------------------------------- ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

struct OpticalSensor {
    const uint8_t VCC_PIN, ANALOG_VO_PIN;
    OpticalSensor(uint8_t VCC_PIN, uint8_t ANALOG_VO_PIN) :
        VCC_PIN(VCC_PIN), ANALOG_VO_PIN(ANALOG_VO_PIN), _Active(false), _AnalogValue(0.0) {};

    void Setup()
    {
        pinMode(VCC_PIN, OUTPUT);
        pinMode(ANALOG_VO_PIN, INPUT);

        digitalWrite(VCC_PIN, HIGH);
    };

    const float & ReadInput()
    {
        _AnalogValue = analogRead(ANALOG_VO_PIN) * (5.0 / 1023.0);
        _Active = (_AnalogValue > 3.00);
        return _AnalogValue;
    };

    const float & AnalogValue() { return _AnalogValue; };
    const bool IsActive() const { return _Active; };

private:
    float _AnalogValue;
    bool _Active;
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
        digitalWrite(LEFT_PIN_OUT.DIR_B_PIN, LOW);
        _LeftPWN = 0;
        _LeftSpeed = 0.0;
        _LeftDirectionRatio = 1.0;

        digitalWrite(RIGHT_PIN_OUT.PWN_PIN, LOW);
        digitalWrite(RIGHT_PIN_OUT.DIR_B_PIN, LOW);
        _RightPWM  = 0;
        _RightSpeed = 0.0;
        _RightDirectionRatio = 1.0;

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
        static const float min = 0.0, max = 1.0;

        if (id == MOTOR_RIGHT)
        {
            _RightSpeed = (value > max) ? max : (value < min) ? min : value;
        }
        else
        {
            _LeftSpeed = (value > max) ? max : (value < min) ? min : value;
        }

        UpdateMotors(); // write motors output
    };

    const float DirectionAngle() { return _DirectionAngle; };
    void Direction(const DirectionID id, const float value)
    {
        static const float min = -1.0, max = 1.0;
        float ratio = (value > max) ? max : (value < min) ? min : value;

        switch (id)
        {
            case BACKWARD :
            {
                _DirectionAngle = 270.0;
                _RightDirectionRatio = 1.0;
                _LeftDirectionRatio = 1.0;
                break;
            };

            case FORWARD :
            {
                _DirectionAngle = 90.0;
                _RightDirectionRatio = -1.0;
                _LeftDirectionRatio = -1.0;
                break;
            };

            case TURN_LEFT :
            {
                if (ratio == 0.0)
                {
                    // tour a gauche avec faible rayon de bracage
                }
                else if (ratio > 0.0)
                {
                    // 90 - 180 | 1....0
                    // sin = left
                    // -cos = right

                    _DirectionAngle = 90.0 + (90.0 * ratio);
                    _LeftDirectionRatio = sin(_DirectionAngle);
                    _RightDirectionRatio = -cos(_DirectionAngle);
                }
                else if (ratio < 0.0)
                {
                    // 270 - 180 | -1....0
                    // sin = left
                    // cos = right

                    _DirectionAngle = 270.0 - (90.0 * abs(ratio));
                    _LeftDirectionRatio = sin(_DirectionAngle);
                    _RightDirectionRatio = cos(_DirectionAngle);
                }

                break;
            };

            case TURN_RIGHT :
            {
                if (ratio == 0.0)
                {
                    // droit a gauche avec faible rayon de bracage
                }
                else if (ratio > 0.0)
                {
                    // 90 - 0 | 1....0
                    // cos = left
                    // sin = right

                    _DirectionAngle = 90.0 * ratio;
                    _LeftDirectionRatio = cos(_DirectionAngle);
                    _RightDirectionRatio = sin(_DirectionAngle);
                }
                else if (ratio < 0.0)
                {
                    // 270 - 360 | -1....0
                    // -cos = left
                    // sin = right

                    _DirectionAngle = 360.0 - (90.0 * abs(ratio));
                    _LeftDirectionRatio = -cos(_DirectionAngle);
                    _RightDirectionRatio = sin(_DirectionAngle);
                }

                break;
            };

            default:
                break;
        }

        UpdateMotors(); // write motors output
    }

protected:

    void UpdateMotors()
    {

        if (_LeftDirectionRatio < 0)
        {
            digitalWrite(LEFT_PIN_OUT.DIR_A_PIN, LOW);
            digitalWrite(LEFT_PIN_OUT.DIR_B_PIN, HIGH);
        }
        else
        {
            digitalWrite(LEFT_PIN_OUT.DIR_A_PIN, HIGH);
            digitalWrite(LEFT_PIN_OUT.DIR_B_PIN, LOW);
        }

        if (_RightDirectionRatio < 0)
        {
            digitalWrite(RIGHT_PIN_OUT.DIR_A_PIN, LOW);
            digitalWrite(RIGHT_PIN_OUT.DIR_B_PIN, HIGH);
        }
        else
        {
            digitalWrite(RIGHT_PIN_OUT.DIR_A_PIN, HIGH);
            digitalWrite(RIGHT_PIN_OUT.DIR_B_PIN, LOW);
        }

        _LeftPWN = map(_LeftSpeed * 1599 * _RightDirectionRatio, 0, 1599, 0, 1599);
        _RightPWM = map(_RightSpeed  * 1599 * _RightDirectionRatio, 0, 1599, 0, 1599);
    };

private:
    long int &_RightPWM, &_LeftPWN;
    float _RightSpeed, _LeftSpeed;

    float _RightDirectionRatio, _LeftDirectionRatio, _DirectionAngle;
};

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- MACHINE STATES ---------------------------------------------------- ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

namespace States {

    struct DecenteSansLanceur {
        static const unsigned short STATE_ID = 17;

        DecenteSansLanceur() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };

    struct MonteeSansLanceur {
        static const unsigned short STATE_ID = 16;

        MonteeSansLanceur() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };

    struct ParcoursSansLanceur {
        static const unsigned short STATE_ID = 15;

        ParcoursSansLanceur() {};

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

    struct VirageSortieZoneLancement {
        static const unsigned short STATE_ID = 14;

        VirageSortieZoneLancement() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };

    struct SeparationLanceur {
        static const unsigned short STATE_ID = 13;

        SeparationLanceur() {};

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

    struct SequenceLancementPoches {
        static const unsigned short STATE_ID = 12;

        SequenceLancementPoches() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };

    struct SequenceCalculRotationCible {
        static const unsigned short STATE_ID = 11;

        SequenceCalculRotationCible() {};

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

    struct VerificationDeclenchementLimitSwitchs {
        static const unsigned short STATE_ID = 10;

        VerificationDeclenchementLimitSwitchs() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };

    struct DeploiementLimitSwitchs {
        static const unsigned short STATE_ID = 9;

        DeploiementLimitSwitchs() {};

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

    struct DeploiementCanon {
        static const unsigned short STATE_ID = 8;

        DeploiementCanon() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };

    struct AlignmentReedSwitchZoneLancement {
        static const unsigned short STATE_ID = 7;

        AlignmentReedSwitchZoneLancement() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };

    struct VirageEntreeZoneLancement {
        static const unsigned short STATE_ID = 6;

        VirageEntreeZoneLancement() : active(false) {};

        void Update()
        {
            active = true;
        };

        void Execute()
        {
            MOTORS.Speed(MOTOR_LEFT, 0.00);
            MOTORS.Speed(MOTOR_RIGHT, 0.00);
        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };
}

namespace States {

    struct DecenteAvecLanceur {
        static const unsigned short STATE_ID = 5;

        DecenteAvecLanceur() : active(false) {};

        void Update()
        {
            OPTICAL_SENSOR.ReadInput();
            if (!OPTICAL_SENSOR.IsActive())
            {
                CURRENT_STATES_ID = VirageEntreeZoneLancement::STATE_ID;
                active = false;
            }
            else
            {
                active = true;
            }
        };

        void Execute()
        {
            MOTORS.Speed(MOTOR_LEFT, 0.50);
            MOTORS.Speed(MOTOR_RIGHT, 0.50);
        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };

    struct MonteeAvecLanceur {
        static const unsigned short STATE_ID = 4;

        MonteeAvecLanceur() : active(false), sommetDosDane(false) {};

        void Update() {

            OPTICAL_SENSOR.ReadInput();
            if (!OPTICAL_SENSOR.IsActive())
            {
                sommetDosDane = true;
            }
            else if (OPTICAL_SENSOR.IsActive() && sommetDosDane)
            {
                CURRENT_STATES_ID = DecenteAvecLanceur::STATE_ID;
                active = false;
            }
            else
            {
                active = true;
            }
        };

        void Execute() {

            MOTORS.Speed(MOTOR_LEFT, 0.50);
            MOTORS.Speed(MOTOR_RIGHT, 0.50);
        };

        const bool & IsActive() { return active; };

    private:
        bool active;
        bool sommetDosDane;
    };

    struct ParcoursAvecLanceur {
        static const unsigned short STATE_ID = 3;

        ParcoursAvecLanceur() : active(false), firstpass(true) {};

        void Update() {

            if (firstpass)
            {
                time = millis();
                firstpass = false;
            }

            OPTICAL_SENSOR.ReadInput();
            if (OPTICAL_SENSOR.IsActive() && time + 3000 < millis())
            {
                CURRENT_STATES_ID = MonteeAvecLanceur::STATE_ID;

                active = false;
            }
            else
            {
                active = true;
            }
        };

        void Execute() {

            MOTORS.Speed(MOTOR_LEFT, 0.50);
            MOTORS.Speed(MOTOR_RIGHT, 0.50);
        };

        const bool & IsActive() { return active; };

    private:
        bool active;
        bool firstpass;
        unsigned long time;
    };




}

namespace States {

    struct NouvellePocheDansMagasin {
        static const unsigned short STATE_ID = 2;

        NouvellePocheDansMagasin() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };

    struct ChargementBatterie {
        static const unsigned short STATE_ID = 1;

        ChargementBatterie() {};

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

long int LEFT_MOTOR_PWMVAL = 0;
long int RIGHT_MOTOR_PWMVAL = 0;

XBeeComm XBEECOMM               = XBeeComm(10, 9, 57600);
LimitSwitch LIMIT_SWITCH        = LimitSwitch(46);
OpticalSensor OPTICAL_SENSOR    = OpticalSensor(48, A12);


MotorPinOut LEFT_MOTOR_PIN_OUT   = MotorPinOut(5, 4, 12, A1);
MotorPinOut RIGHT_MOTOR_PIN_OUT  = MotorPinOut(7, 6, 11, A0);
Motors MOTORS                    = Motors(RIGHT_MOTOR_PIN_OUT, LEFT_MOTOR_PIN_OUT);

unsigned short CURRENT_STATES_ID = States::PARCOURS_AVEC_LANCEUR.STATE_ID;

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
    MonteeSansLanceur MONTEE_SANS_LANCER = MonteeSansLanceur();
    DecenteSansLanceur DECENTE_SANS_LANCER = DecenteSansLanceur();

};

void INB4::setup() {

    XBEECOMM.begin(57600);
    Serial.begin(9600);
    delayMicroseconds(100);

    MOTORS.Setup();
    OPTICAL_SENSOR.Setup();

    //LIMIT_SWITCH.Setup();
}

void INB4::loop() {

    static int xBeeRead;
    static float motorSpeed; static const float motorSpeedMin = 0.0, motorSpeedMax = 1.0;
    static float motorDirection; static const float motorDirectionMin = -1.0, motorDirectionMax = 1.0;

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
            motorDirection -= (motorDirection > motorDirectionMin) ? 0.05 : 0.0;

            if (motorDirection < 0.0)
            {
                MOTORS.Direction(TURN_LEFT, abs(motorDirection));
            }
            else
            {
                MOTORS.Direction(TURN_RIGHT, motorDirection);
            }

            break;
        };

        case KEYBOARD_D:
        {
            motorDirection += (motorDirection < motorDirectionMax) ? 0.05  : 0.0;

            if (motorDirection < 0.0)
            {
                MOTORS.Direction(TURN_LEFT, abs(motorDirection));
            }
            else
            {
                MOTORS.Direction(TURN_RIGHT, motorDirection);
            }

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


    OPTICAL_SENSOR.ReadInput();
    if (OPTICAL_SENSOR.IsActive())
    {
        Serial.print("TRUE "); Serial.print(OPTICAL_SENSOR.AnalogValue()); Serial.println("v");
    }
    else
    {
        Serial.print("FALSE "); Serial.print(OPTICAL_SENSOR.AnalogValue()); Serial.println("v");
    }


    //Serial.print(OPTICAL_SENSOR.ReadInput()); Serial.println("v");


    delay(30);


    ///////////////////////////////////////////////////////////////////////////
    ///                                                                     ///
    ///                                                                     ///
    ///////////////////////////////////////////////////////////////////////////

    switch (CURRENT_STATES_ID){

        case States::ChargementBatterie::STATE_ID :
        {
            States::CHARGEMENT_BATTERIE.Update();
            if (States::CHARGEMENT_BATTERIE.IsActive())
            {
                States::CHARGEMENT_BATTERIE.Execute();
            }

            break;
        }

        case States::NouvellePocheDansMagasin::STATE_ID :
        {
            States::NOUVELLE_POCHE_DANS_MAGASIN.Update();
            if (States::NOUVELLE_POCHE_DANS_MAGASIN.IsActive())
            {
                States::NOUVELLE_POCHE_DANS_MAGASIN.Execute();
            }

            break;
        }


        case States::ParcoursAvecLanceur::STATE_ID :
        {
            States::PARCOURS_AVEC_LANCEUR.Update();
            if (States::PARCOURS_AVEC_LANCEUR.IsActive())
            {
                States::PARCOURS_AVEC_LANCEUR.Execute();
            }

            break;
        };

        case States::MonteeAvecLanceur::STATE_ID :
        {
            States::MONTEE_AVEC_LANCER.Update();
            if (States::MONTEE_AVEC_LANCER.IsActive())
            {
                States::MONTEE_AVEC_LANCER.Execute();
            }

            break;
        };

        case States::DecenteAvecLanceur::STATE_ID :
        {
            States::DECENTE_AVEC_LANCER.Update();
            if (States::DECENTE_AVEC_LANCER.IsActive())
            {
                States::DECENTE_AVEC_LANCER.Execute();
            }

            break;
        };


        case States::VirageEntreeZoneLancement::STATE_ID :
        {
            States::VIRAGE_ENTREE_ZONE_LANCEMENT.Update();
            if (States::VIRAGE_ENTREE_ZONE_LANCEMENT.IsActive())
            {
                States::VIRAGE_ENTREE_ZONE_LANCEMENT.Execute();
            }

            break;
        };

        case States::AlignmentReedSwitchZoneLancement::STATE_ID :
        {
            States::ALIGNEMENT_REEDSWITCH_ZONE_LANCEMENT.Update();
            if (States::ALIGNEMENT_REEDSWITCH_ZONE_LANCEMENT.IsActive())
            {
                States::ALIGNEMENT_REEDSWITCH_ZONE_LANCEMENT.Execute();
            }

            break;
        };

        case States::DeploiementCanon::STATE_ID :
        {
            States::DEPLOIEMENT_CANON.Update();
            if (States::DEPLOIEMENT_CANON.IsActive())
            {
                States::DEPLOIEMENT_CANON.Execute();
            }

            break;
        };

        case States::DeploiementLimitSwitchs::STATE_ID :
        {
            States::DEPLOIEMENT_LIMITSWITCHS.Update();
            if (States::DEPLOIEMENT_LIMITSWITCHS.IsActive())
            {
                States::DEPLOIEMENT_LIMITSWITCHS.Execute();
            }

            break;
        };

        case States::VerificationDeclenchementLimitSwitchs::STATE_ID :
        {
            States::VERIFICATION_DECLENCHEMENT_LIMITSWITCHS.Update();
            if (States::VERIFICATION_DECLENCHEMENT_LIMITSWITCHS.IsActive())
            {
                States::VERIFICATION_DECLENCHEMENT_LIMITSWITCHS.Execute();
            }

            break;
        };

        case States::SequenceCalculRotationCible::STATE_ID :
        {
            States::SEQUENCE_CALCUL_ROTATION_CIBLE.Update();
            if (States::SEQUENCE_CALCUL_ROTATION_CIBLE.IsActive())
            {
                States::SEQUENCE_CALCUL_ROTATION_CIBLE.Execute();
            }

            break;
        };

        case States::SequenceLancementPoches::STATE_ID :
        {
            States::SEQUENCE_LANCEMENT_POCHES.Update();
            if (States::SEQUENCE_LANCEMENT_POCHES.IsActive())
            {
                States::SEQUENCE_LANCEMENT_POCHES.Execute();
            }

            break;
        };


        case States::SeparationLanceur::STATE_ID :
        {
            States::SEPARATION_LANCEUR.Update();
            if (States::SEPARATION_LANCEUR.IsActive())
            {
                States::SEPARATION_LANCEUR.Execute();
            }

            break;
        };

        case States::VirageSortieZoneLancement::STATE_ID :
        {
            States::VIRAGE_SORTIE_ZONE_LANCEMENT.Update();
            if (States::VIRAGE_SORTIE_ZONE_LANCEMENT.IsActive())
            {
                States::VIRAGE_SORTIE_ZONE_LANCEMENT.Execute();
            }

            break;
        };


        case States::ParcoursSansLanceur::STATE_ID :
        {
            States::PARCOURS_SANS_LANCEUR.Update();
            if (States::PARCOURS_SANS_LANCEUR.IsActive())
            {
                States::PARCOURS_SANS_LANCEUR.Execute();
            }

            break;
        };

        case States::MonteeSansLanceur::STATE_ID :
        {
            States::MONTEE_SANS_LANCER.Update();
            if (States::MONTEE_SANS_LANCER.IsActive())
            {
                States::MONTEE_SANS_LANCER.Execute();
            }

            break;
        };

        case States::DecenteSansLanceur::STATE_ID :
        {
            States::DECENTE_SANS_LANCER.Update();
            if (States::DECENTE_SANS_LANCER.IsActive())
            {
                States::DECENTE_SANS_LANCER.Execute();
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



