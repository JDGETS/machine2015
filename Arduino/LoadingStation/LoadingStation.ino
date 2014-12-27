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
#define ELEVATOR_STOP  1500
#define ELEVATOR_RUN   1400
#define DOOR_STOP      1500
#define DOOR_RUN       1400
#define CONVEYOR_RUN   1400
#define CONVEYOR_STOP  1500

//////// Variables //////// 
LimitSwitch elevatorSwitch(ELEVATOR_SWITCH_PIN);
LimitSwitch doorSwitch(DOOR_SWITCH_PIN);
Servo elevator;
Servo door;
Servo conveyor1;
Servo conveyor2;


void setup() {    
  Serial.begin(9600);
  
  elevatorSwitch.Setup();
  doorSwitch.Setup();
  
  elevator.attach(ELEVATOR_PIN); 
  elevator.writeMicroseconds(ELEVATOR_STOP);
  
  door.attach(DOOR_PIN);
  door.writeMicroseconds(DOOR_STOP);
  
  conveyor1.attach(CONVEYOR1_PIN);
  Serial.print("Starting fixed conveyor belt.");
  conveyor1.writeMicroseconds(CONVEYOR_RUN);
  
  conveyor2.attach(CONVEYOR2_PIN);
  conveyor2.writeMicroseconds(CONVEYOR_STOP);
}

void loop(){  
  // La porte est ouverte
  if ( ! doorSwitch.ReadInput() )
  {
    Serial.print("Elevator door opened. Closing...");
    // On la ferme et on attend
    door.writeMicroseconds(DOOR_RUN);
    doorSwitch.WaitForPress();
    door.writeMicroseconds(DOOR_STOP);
    Serial.print("Elevator door closed.");
  }
  
  // L'ascenseur n'est pas en haut
  if ( ! elevatorSwitch.ReadInput() )
  {
    // Arreter le tapis sur l'ascenseur (au cas ou il roulerait...)
    conveyor2.writeMicroseconds(CONVEYOR_STOP);
    // On monte l'ascenseur et on attend
    Serial.print("Starting elevator.");
    elevator.writeMicroseconds(ELEVATOR_RUN);
    elevatorSwitch.WaitForPress();
    // Arreter l'ascenseur
    elevator.writeMicroseconds(ELEVATOR_STOP); 
    Serial.print("Stopping elevator.");
  }
          
  // Demarrer le tapis de l'ascenseur
  Serial.print("Starting elevator's conveyor belt.");
  conveyor2.writeMicroseconds(CONVEYOR_RUN);
 
}
