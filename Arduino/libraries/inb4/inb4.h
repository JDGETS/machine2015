





#ifndef INB4_H
#define INB4_H



#ifndef SKETCH
#include "/usr/share/arduino/hardware/arduino/cores/arduino/Arduino.h"

#define USBCON
#include "/usr/share/arduino/hardware/arduino/cores/arduino/USBAPI.h"

#endif



typedef unsigned char uint8_t;

extern void Level1();


class inb4 {

public:

    static const uint8_t PIN_OUT_STATES [];
    static const uint8_t PIN_OUT_LAST_INDEX;
    static const uint8_t PULL_UP_PIN;

    static void setup();
    static void loop();
    static void coady();

    //static void Level1();

};

class XbeeCommunication {

};

class Motors {

    static const unsigned short LEFT_MOTOR_ID = -1, RIGHT_MOTOR_ID = 1, BOTH_MOTORS_ID = 0;

    static void Initialize() {};

    static void Forwards(unsigned short motorID) {};
    static void Backwards(unsigned short motorID) {};
    static void Stop(unsigned short motorID) {};
};



#endif // INB4_H

const uint8_t inb4::PIN_OUT_STATES[] = {53, 51, 49, 47, 45, 43, 35, 33, 31, 29, 27, 25};
const uint8_t inb4::PIN_OUT_LAST_INDEX = 12;
const uint8_t inb4::PULL_UP_PIN = 38;

void inb4::setup()
{
    Serial.begin(9600);
    pinMode(PULL_UP_PIN, INPUT_PULLUP);

    pinMode(PIN_OUT_STATES[0], OUTPUT);
    pinMode(PIN_OUT_STATES[1], OUTPUT);
    pinMode(PIN_OUT_STATES[2], OUTPUT);
    pinMode(PIN_OUT_STATES[3], OUTPUT);
    pinMode(PIN_OUT_STATES[4], OUTPUT);
    pinMode(PIN_OUT_STATES[5], OUTPUT);
    pinMode(PIN_OUT_STATES[6], OUTPUT);
    pinMode(PIN_OUT_STATES[7], OUTPUT);
    pinMode(PIN_OUT_STATES[8], OUTPUT);
    pinMode(PIN_OUT_STATES[9], OUTPUT);
    pinMode(PIN_OUT_STATES[10], OUTPUT);
    pinMode(PIN_OUT_STATES[11], OUTPUT);
    pinMode(PIN_OUT_STATES[12], OUTPUT);
};

void inb4::loop()
{
    int sensorVal = digitalRead(PULL_UP_PIN);
    Serial.println(sensorVal);

    digitalWrite(PIN_OUT_STATES[0], sensorVal);

    if (false) { Level1(); }
};

void inb4::coady() {};

void Level1()
{
    const unsigned long timeout = 20;

    digitalWrite(inb4::PIN_OUT_STATES[0], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[0], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[1], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[1], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[2], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[2], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[3], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[3], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[4], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[4], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[5], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[5], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[6], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[6], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[7], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[7], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[8], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[8], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[9], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[9], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[10], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[10], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[11], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[11], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[12], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[12], LOW);
};