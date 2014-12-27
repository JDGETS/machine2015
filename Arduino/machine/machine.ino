

#include <Arduino.h>
#include <SoftwareSerial.h>

#include "LimitSwitch.h"
#include "OpticalSensor.h"
#include "ReedSwitch.h"
#include "XBeeComm.h"
#include "StateINB4.h"
#include "Motors.h"

#define SKETCH



/// grep -r "string to search" .
#include <INB4.h>
//#include <inb4.h>

ISR(TIMER1_OVF_vect)
{
    OCR1A =  LEFT_MOTOR_PWMVAL ;
    OCR1B =  RIGHT_MOTOR_PWMVAL ;
}

void setup(){ 
  INB4::setup(); 
}
void loop(){ 
  INB4::loop(); 
}

