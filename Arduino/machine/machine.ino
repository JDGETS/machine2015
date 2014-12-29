

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

void setup(){ 
  INB4::setup(); 
}
void loop(){ 
  INB4::loop(); 
}

