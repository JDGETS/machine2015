//Must include "Shooter/Store.ino"
//Used to calibrate the values for STORE_SERVO_ROTATION_STEP and the different formulas for the bags' store.
#include <Servo.h>
#include "Store.h"

// Calibration tools
String readString;
void setup() {
  Serial.begin(9600);
  Serial.println("Start magasin.");
  States::store.Setup();
  States::store.Initialize();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();  //gets one byte from serial buffer
    readString += c; //makes the string readString
    delay(2);  //slow looping to allow buffer to fill with next character
  }

  if (readString.length() > 0) {
    if(readString == "1"){ // Move one empty (calibration)
      States::store.servoPosition += Store::STORE_SERVO_MICROS_STEP;
      States::store.servo.writeMicroseconds(States::store.servoPosition);
      Serial.print("Microseconds is: ");
      Serial.println(States::store.servoPosition);
    }
    if(readString == "q"){ // Load bag
      Serial.print("Loading one bag. total=");
      Serial.println(States::store.bagsCount);
      States::store.ServoLoadBag();
    }
    else if(readString == "p"){ // Unload bag
      Serial.print("Unloading one bag. total=");
      Serial.println(States::store.bagsCount);
      States::store.ServoUnloadBag();
    }
    
    readString=""; // Empty for next input
  }
}

