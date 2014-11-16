


#include "/usr/share/arduino/hardware/arduino/cores/arduino/Arduino.h"
#include "inb4.h"


void inb4::setup()
{
    pinMode(pinLed, OUTPUT);
};

void inb4::loop()
{
    digitalWrite(pinLed, HIGH);
    delay(1000);

    digitalWrite(pinLed, LOW);
    delay(1000);
};

