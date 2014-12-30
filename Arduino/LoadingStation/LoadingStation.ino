#include <Servo.h>
#include "LimitSwitch.h"

//////// Pins //////// 
#define ELEVATOR_SWITCH_PIN     46
#define DOOR_SWITCH_PIN         50
#define ELEVATOR_PIN            4
#define DOOR_PIN                5
#define CONVEYOR1_PIN           6
#define CONVEYOR2_PIN           7

//////// PWM //////// 
#define CONVEYOR_RUN   1400
#define CONVEYOR_STOP  1500

//////// Variables //////// 
LimitSwitch elevatorSwitch(ELEVATOR_SWITCH_PIN);
LimitSwitch doorSwitch(DOOR_SWITCH_PIN);
Servo conveyor1;
Servo conveyor2;

void setup() {    
  Serial.begin(9600);
  
  elevatorSwitch.Setup();
  doorSwitch.Setup();
  
  pinMode(ELEVATOR_PIN, OUTPUT);
  pinMode(DOOR_PIN, OUTPUT);
  digitalWrite(ELEVATOR_PIN, LOW);
  digitalWrite(DOOR_PIN, LOW);
  
  conveyor1.writeMicroseconds(CONVEYOR_STOP); // Set initial position
  conveyor1.attach(CONVEYOR1_PIN);
  
  conveyor2.writeMicroseconds(CONVEYOR_STOP); // Set initial position
  conveyor2.attach(CONVEYOR2_PIN);
}

void loop(){  
  return
  // La porte est ouverte
  if ( ! doorSwitch.ReadInput() )
  {
    Serial.print("Elevator door opened. Closing...");
    // On la ferme et on attend
    digitalWrite(DOOR_PIN, HIGH);
    doorSwitch.WaitForPress();
    digitalWrite(DOOR_PIN, LOW);
    Serial.print("Elevator door closed.");
  }
  else{
    // L'ascenseur n'est pas en haut
    if ( ! elevatorSwitch.ReadInput() )
    {
      // Arreter le tapis sur l'ascenseur (au cas ou il roulerait...)
      conveyor2.writeMicroseconds(CONVEYOR_STOP);
      // On monte l'ascenseur et on attend
      Serial.print("Starting elevator.");
      digitalWrite(ELEVATOR_PIN, HIGH);
      elevatorSwitch.WaitForPress();
      // Arreter l'ascenseur
      digitalWrite(ELEVATOR_PIN, LOW); 
      Serial.print("Stopping elevator.");
    }
    else{
      // Si les deux switches sont appuyées, on roule les deux tapis.
      conveyor1.writeMicroseconds(CONVEYOR_RUN);
      conveyor2.writeMicroseconds(CONVEYOR_RUN);
    }
  }
}
