#include <Servo.h>
#include "LimitSwitch.h"

//////// Pins //////// 
#define ELEVATOR_SWITCH_PIN     38
#define DOOR_SWITCH_PIN         36
#define BAG_SWITCH_PIN          32
#define ELEVATOR_PIN            9
#define DOOR_PIN                10
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

unsigned long conveyor1_running_until = 0;
unsigned long conveyor1_stopped_until = 0;
unsigned long CONVEYOR1_RUNTIME = 1000;
unsigned long CONVEYOR1_STOPTIME = 3000;


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
  // La porte est ouverte
  if ( ! doorSwitch.ReadInput() )
  {
    Serial.println("Elevator door opened. Closing...");
    // On la ferme et on attend
    conveyor1.writeMicroseconds(CONVEYOR_STOP);
    digitalWrite(DOOR_PIN, HIGH);
    doorSwitch.WaitForPress();
    digitalWrite(DOOR_PIN, LOW);
    Serial.println("Elevator door closed.");
  }
  else{
    // L'ascenseur n'est pas en haut
    if ( ! elevatorSwitch.ReadInput() )
    {
      // Arreter le tapis sur l'ascenseur (au cas ou il roulerait...)
      conveyor2.writeMicroseconds(CONVEYOR_STOP);
      // On monte l'ascenseur et on attend
      Serial.println("Starting elevator.");
      conveyor1.writeMicroseconds(CONVEYOR_STOP);
      digitalWrite(ELEVATOR_PIN, HIGH);
      elevatorSwitch.WaitForPress();
      // Arreter l'ascenseur
      digitalWrite(ELEVATOR_PIN, LOW); 
      Serial.println("Stopping elevator.");
      delay(1000);
      conveyor1_running_until = millis();
    }
    else{
      // Si les deux switches sont appuyées, on roule les deux tapis avec un petit pattern sur le premier.
      if(conveyor1_running_until != 0 && conveyor1_running_until < millis())
      {
        Serial.println("Stop conveyor1");
        conveyor1_stopped_until = millis() + CONVEYOR1_STOPTIME;
        conveyor1_running_until = 0;
        conveyor1.writeMicroseconds(CONVEYOR_STOP);
      }
      else if(conveyor1_stopped_until != 0 && conveyor1_stopped_until < millis())
      {
        Serial.println("Start conveyor1");
        conveyor1_running_until = millis() + CONVEYOR1_RUNTIME;
        conveyor1_stopped_until = 0;
        conveyor1.writeMicroseconds(CONVEYOR_RUN);
      }
      conveyor2.writeMicroseconds(CONVEYOR_RUN);
    }
  }
}
