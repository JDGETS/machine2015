#include <Servo.h>
#include "ForceStopVehicle.h"
#include "LimitSwitch.h"
#include "ForceStopVehicle.h"
#include "Hardware.h"
#include "Vehicle.h"
#include "AllStates.h"

//////// Pins //////// 
#define ELEVATOR_SWITCH_PIN     34
#define BAG_SWITCH_PIN          32
#define DRILL_PIN               11
#define ELEVATOR_PIN            10
#define CONVEYOR1_PIN           6
#define CONVEYOR2_PIN           4
#define CONVEYOR2_MOTOR_PIN     9

#define TIME_LIMIT_SWITCH_TO_FREE_FALL 1000
#define TIME_FREE_FALL_AND_MAG_MOVE    1000

//////// PWM //////// 
#define CONVEYOR_RUN   00
#define CONVEYOR_RUN_2   89
#define CONVEYOR_STOP  92

//////// Variables //////// 
LimitSwitch elevatorSwitch(ELEVATOR_SWITCH_PIN);
LimitSwitch bagSwitch(BAG_SWITCH_PIN);
Servo conveyor1;
Servo conveyor2;

unsigned long conveyor1_running_until = 0;
unsigned long conveyor1_stopped_until = 0;
unsigned long CONVEYOR1_RUNTIME = 2000;
unsigned long CONVEYOR1_STOPTIME = 5000;

void setup() {    
  Serial.begin(9600);
  
  elevatorSwitch.Setup();
  bagSwitch.Setup();
  
  pinMode(ELEVATOR_PIN, OUTPUT);
  digitalWrite(ELEVATOR_PIN, LOW);
  pinMode(DRILL_PIN, OUTPUT);
  digitalWrite(DRILL_PIN, LOW);
  pinMode(CONVEYOR2_MOTOR_PIN, OUTPUT);
  digitalWrite(CONVEYOR2_MOTOR_PIN, LOW);
  
  conveyor1.write(CONVEYOR_STOP); // Set initial position
  conveyor1.attach(CONVEYOR1_PIN);
  conveyor1.write(CONVEYOR_STOP); // Set initial position
  
  conveyor2.write(CONVEYOR_RUN_2); // Set initial position
  conveyor2.attach(CONVEYOR2_PIN);
  conveyor2.write(CONVEYOR_RUN_2); // Set initial position
  
  pinMode(CONVEYOR2_MOTOR_PIN, OUTPUT);
  digitalWrite(CONVEYOR2_MOTOR_PIN, HIGH);
  
  delay(1000);
  bagSwitch.WaitForPress();
  delay(3000);
}

boolean bagSwitchStatus = false;
float bagCount = 0;
unsigned long tempTime = 0;
boolean waitingForPocheToTombe = 0;

void CheckPoches(){
  if(bagSwitch.ReadInput() && !bagSwitchStatus){
    bagSwitchStatus = true;
    conveyor1.write(CONVEYOR_STOP);
    conveyor2.write(CONVEYOR_STOP);
    delay(1000);
    conveyor2.write(CONVEYOR_RUN_2);
  }
  else if(!bagSwitch.ReadInput() && bagSwitchStatus){
    bagSwitchStatus = false;
    
  }
}

void SetConveyors(){
  if(conveyor1_running_until != 0 && conveyor1_running_until < millis())
  {
    Serial.println("Stop conveyor1");
    conveyor1_stopped_until = millis() + CONVEYOR1_STOPTIME;
    conveyor1_running_until = 0;
    conveyor1.write(CONVEYOR_STOP);
  }
  else if(conveyor1_stopped_until != 0 && conveyor1_stopped_until < millis())
  {
    Serial.println("Start conveyor1");
    conveyor1_running_until = millis() + CONVEYOR1_RUNTIME;
    conveyor1_stopped_until = 0;
    conveyor1.write(CONVEYOR_RUN);
  }
  conveyor2.write(CONVEYOR_RUN_2);
}

void loop(){
    digitalWrite(CONVEYOR2_MOTOR_PIN, HIGH);
    // L'ascenseur n'est pas en haut
    if ( ! elevatorSwitch.ReadInput() )
    {
      Serial.println("Elevator not at the top... Going up.");
      // Arreter le tapis sur l'ascenseur (au cas ou il roulerait...)
      conveyor1.write(CONVEYOR_STOP);
      // On monte l'ascenseur et on attend
      Serial.println("Starting elevator.");
      digitalWrite(ELEVATOR_PIN, HIGH);
      elevatorSwitch.WaitForPress();
      Serial.println("Elevator at the top... Shutting down motors.");
      // Arreter l'ascenseur
      digitalWrite(ELEVATOR_PIN, LOW); 
      Serial.println("Stopping elevator.");
      delay(1000);
      conveyor1_running_until = millis();
    }
    else{
      Serial.println("Starting conveyors.");
      // Si les deux switches sont appuyées, on roule les deux tapis avec un petit pattern sur le premier.
      SetConveyors();
      CheckPoches();
    }
}

