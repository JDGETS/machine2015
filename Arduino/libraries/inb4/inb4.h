

#ifndef INB4_H
#define INB4_H

#ifndef SKETCH
#include "/usr/share/arduino/hardware/arduino/cores/arduino/Arduino.h"

#define USBCON
#include "/usr/share/arduino/hardware/arduino/cores/arduino/USBAPI.h"
#include "/usr/share/arduino/libraries/SoftwareSerial/SoftwareSerial.h"
#include "/usr/share/arduino//hardware/arduino/variants/mega/pins_arduino.h"

#include "/usr/share/arduino/hardware/tools/avr/lib/avr/include/avr/iomxx0_1.h"
//#include "/usr/share/arduino/hardware/tools/avr/lib/avr/include/avr/interrupt.h"
//#include "/usr/share/arduino/hardware/tools/avr/lib/avr/include/avr/io.h"
#endif


typedef unsigned char uint8_t;
//typedef unsigned int uint16_t;

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- REED SWITCH ------------------------------------------------------- ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

typedef struct ReedSwitch {
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
} ReedSwitch;

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- RGB LED ----------------------------------------------------------- ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

typedef struct RgbLed {
    const uint8_t PIN_RED, PIN_GREEN, PIN_BLUE;
    RgbLed(uint8_t PIN_RED, uint8_t PIN_GREEN, uint8_t PIN_BLUE)
            : PIN_RED(PIN_RED), PIN_GREEN(PIN_GREEN),  PIN_BLUE(PIN_BLUE),
              red_active(false), green_active(false), blue_active(false) {};
    void Setup()
    {
        pinMode(PIN_RED, OUTPUT);
        pinMode(PIN_GREEN, OUTPUT);
        pinMode(PIN_BLUE, OUTPUT);
    };
    const uint8_t Red() const { return static_cast<uint8_t>(red_active ? HIGH : LOW); };
    const uint8_t Green() const { return static_cast<uint8_t>(green_active ? HIGH : LOW); };
    const uint8_t Blue() const { return static_cast<uint8_t>(blue_active ? HIGH : LOW); };
    const int Red(bool value) { red_active = value; return Red(); };
    const int Green(bool value) { green_active = value; return Green(); };
    const int Blue(bool value) { blue_active = value; return Blue(); }
    void WriteOutput()
    {
        digitalWrite(PIN_RED, Red());
        digitalWrite(PIN_GREEN, Green());
        digitalWrite(PIN_BLUE, Blue());
    };
private:
    bool red_active, green_active, blue_active;
} RgbLed;

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- XBEE COMMUNICATION ------------------------------------------------ ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

typedef struct XBeeComm  : SoftwareSerial {
    const uint8_t RX_PIN, TX_PIN;
    const long COMM_BAUDRATE;
    XBeeComm(uint8_t RX_PIN, uint8_t TX_PIN, uint16_t COMM_BAUDRATE)
            : SoftwareSerial(RX_PIN, TX_PIN), RX_PIN(RX_PIN), TX_PIN(TX_PIN),
              COMM_BAUDRATE(COMM_BAUDRATE) {};
    void Setup() { begin(COMM_BAUDRATE); };
    const char ReadInput() { return available() ? read() : 0; };
} XBeeComm;

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- MOTORS ------------------------------------------------------------ ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

typedef struct MotorPinOut {
    const uint8_t DIR_A_PIN, DIR_B_PIN, PWN_PIN, ADC_PIN;
    MotorPinOut(uint8_t DIR_A_PIN, uint8_t DIR_B_PIN, uint8_t PWN_PIN, uint8_t ADC_PIN)
            : DIR_A_PIN(DIR_A_PIN), DIR_B_PIN(DIR_B_PIN), PWN_PIN(PWN_PIN), ADC_PIN(ADC_PIN) {};
} MotorPinOut;

typedef struct Motors {
    static const enum MotorID { RIGHT = 0, LEFT = 1};
    const MotorPinOut RIGHT_PIN_OUT, LEFT_PIN_OUT;
    Motors(MotorPinOut RIGHT_PIN_OUT, MotorPinOut LEFT_PIN_OUT)
            : RIGHT_PIN_OUT(RIGHT_PIN_OUT), LEFT_PIN_OUT(LEFT_PIN_OUT) {};


    const int PwmVal (const MotorID id) const { return (id == RIGHT) ? _rightPwmVal : _leftPwmVal; };
    void PwmVal(const MotorID id, const int value) { (id == RIGHT) ? _rightPwmVal = value : _leftPwmVal = value; };


private:
    int _rightPwmVal, _leftPwmVal;
} Motors;

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- MAIN CLASS -------------------------------------------------------- ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

class inb4 {

public:

    static ReedSwitch PULL_UP_H_REEDSWITCH = ReedSwitch(50);
    static ReedSwitch PULL_UP_V_REEDSWITCH = ReedSwitch(52);
    static RgbLed RGB_LED  = RgbLed(49, 51, 53);

    static XBeeComm XBEECOMM = XBeeComm(10, 9, 57600);

    static MotorPinOut LEFT_MOTOR_PIN_OUT = MotorPinOut(5, 4, 12, A1);
    static MotorPinOut RIGHT_MOTOR_PIN_OUT = MotorPinOut(7, 6, 11, A0);
    static Motors MOTORS = Motors(RIGHT_MOTOR_PIN_OUT, LEFT_MOTOR_PIN_OUT);

    static void setup();
    static void loop();

};

#endif // INB4_H

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- CODE / IMPLEMENTATION --------------------------------------------- ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////


#ifdef SKETCH
ISR(TIMER1_OVF_vect)
{
    OCR1A = inb4::MOTORS.PwmVal(Motors::LEFT);
    OCR1B = inb4::MOTORS.PwmVal(Motors::RIGHT);
}
#endif

void inb4::setup()
{
    PULL_UP_V_REEDSWITCH.Setup();
    PULL_UP_H_REEDSWITCH.Setup();
    RGB_LED.Setup();
    XBEECOMM.Setup();
};

void inb4::loop()
{
    ///////////////////////////////////////////////////////////////////////////
    ///                                                                     ///
    ///                                                                     ///
    ///////////////////////////////////////////////////////////////////////////

    PULL_UP_H_REEDSWITCH.ReadInput();
    PULL_UP_V_REEDSWITCH.ReadInput();

    if (PULL_UP_V_REEDSWITCH.IsActive() && PULL_UP_H_REEDSWITCH.IsActive())
    {
        RGB_LED.Red(true); RGB_LED.Green(true); RGB_LED.Blue(true);
    }
    else if (PULL_UP_H_REEDSWITCH.IsActive())
    {
        RGB_LED.Red(false); RGB_LED.Green(true); RGB_LED.Blue(false);
    }
    else if (PULL_UP_V_REEDSWITCH.IsActive())
    {
        RGB_LED.Red(false); RGB_LED.Green(false); RGB_LED.Blue(true);
    }
    else
    {
        RGB_LED.Red(true); RGB_LED.Green(false); RGB_LED.Blue(false);
    }

    RGB_LED.WriteOutput();

    ///////////////////////////////////////////////////////////////////////////
    ///                                                                     ///
    ///                                                                     ///
    ///////////////////////////////////////////////////////////////////////////

    switch (XBEECOMM.ReadInput())
    {
        case 'w':
        { // Forward

            break;
        }

        case 's':
        { // Backward

            break;
        }

        case 'a':
        { // Left

            break;
        }

        case 'd':
        { // Right


            break;
        }

        case 'q':
        { // Stop


            break;
        }

        case 'f':
        { // Half Speed

            break;
        }

        case 'g':
        { // Full Speed



            break;
        }

        default:
            break;
    };






};