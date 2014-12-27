#include <Servo.h>
#include "LimitSwitch.h"
#include "Store.h"

#define SHOOTER_SWITCH_PIN 38
#define BAG_HOLDER_SWITCH_PIN 40
#define STORE_DETECT_BAG_SWITCH_PIN 42
#define STORE_SWITCH_PIN 44
#define SHOOTER_MOTOR_MOSFET 7

namespace States {
    struct Charging {
        Charging() : active(false) {};

        void Update() {
           //Called before execute.
        };

        void Execute() {
          //Do shit
        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };
}

unsigned long nextHoleTime = -1;
unsigned long delta = -1;
unsigned long LOAD_AND_SHOOT_DELAY = 400; // Time it takes to charge and shoot the bag and time until it reaches the hole (millis)
unsigned long BAG_FREE_FALL_DELAY = 500; //Time for the bag to fall into place. Put some padding in there.
unsigned long STORE_ANGLE = 50;
unsigned long bagsLeft = 8;

LimitSwitch shooterSwitch(SHOOTER_SWITCH_PIN);
LimitSwitch bagHolderSwitch(BAG_HOLDER_SWITCH_PIN);
void setup() 
{
  Serial.begin(9600);
  
  //Init motor before servos (interference)
  pinMode(SHOOTER_MOTOR_MOSFET, OUTPUT);
  digitalWrite(SHOOTER_MOTOR_MOSFET, LOW);
  
  States::store.Setup();
  
  shooterSwitch.Setup();
  bagHolderSwitch.Setup();
  
  //pinMode(STORE_SWITCH_PIN, INPUT_PULLUP);  
  States::store.Initialize();

  States::store.bagsCount = 8; //The store is considered full here. //TO-DO: Remove when Charging state is done.
}
 
void loop() 
{
  unsigned long firstHole[2], secondHole[2];
  unsigned long firstHoleTime, secondHoleTime; 
  
  Serial.print("Start time: ");
  unsigned long time = millis();
  Serial.println(time);

  shooterSwitch.ResetDebouncing();
  shooterSwitch.WaitForPress(); //Switch is pressed, now we can start our sequence
  Serial.println("Waiting for first hole...");
  shooterSwitch.WaitForRelease();
  firstHole[0] = millis();
  shooterSwitch.WaitForPress();
  firstHole[1] = millis();
  Serial.println("Waiting for second hole...");
  shooterSwitch.WaitForRelease();
  secondHole[0] = millis();
  shooterSwitch.WaitForPress();
  secondHole[1] = millis();
  
  firstHoleTime = firstHole[1] - firstHole[0];
  secondHoleTime = secondHole[1] - secondHole[0];
  delta = secondHole[0] - firstHole[0];
  nextHoleTime = secondHole[1]; //It's aligned at the center
  Serial.print("Delay is: ");
  Serial.println(delta);
 
  Serial.print("Current is: ");
  Serial.println(millis());
  //First bag is already in the shooter
  unsigned long nextShootingTime = GetNextShootingTime();
  Serial.print("Next is: ");
  Serial.println(nextShootingTime);
    
  while(bagsLeft > 0) {
    delay(nextShootingTime - millis() - LOAD_AND_SHOOT_DELAY); //TO-DO: Precise enough (?)
    Serial.println("Starting to crinque");
    digitalWrite(SHOOTER_MOTOR_MOSFET, HIGH);
    //bagHolderSwitch.WaitForRelease();
    delay(700);
    //bagHolderSwitch.WaitForPress();
    //EndMoteur (if it cuts too late, 2 cases: 1) elastics strong enough to bring back the shooter at its place. no problem then. 2) not strong enough. then you have to adjust LOAD_AND_SHOOT_DELAY for bags #2-8).
    digitalWrite(SHOOTER_MOTOR_MOSFET, LOW); 
    bagsLeft--;
    Serial.print("BOOM! Only ");
    Serial.print(bagsLeft);
    Serial.println(" bags left!");
    delay(2000); //Delay to be sure the system is ready to receive the new bag. TO-DO: Get rid of this ??? or constant this.
    if(bagsLeft > 0){
      DropNextBag();
      nextShootingTime = GetNextShootingTime();
      Serial.print("Next: ");
      Serial.println(nextShootingTime);
    }
  }
 
  Serial.println("We are done.");
  
  //Change state.
  
  return;
} 

//Responsable for dropping the next bag. Include the delay in there. Only return when the bag is in the shooter ready to get shot.
void DropNextBag(){
  Serial.println("Dropped bag.");

  States::store.ServoUnloadBag();

  delay(BAG_FREE_FALL_DELAY); //Wait for the bag to fall into place.
}

unsigned long GetNextShootingTime(){ //Must take into account LOAD_AND_SHOOT_DELAY. This should be the next hole that is theoritically shootable in. Assume computing time is negligeable.
  unsigned long currentTime = millis();
  while(nextHoleTime < currentTime + LOAD_AND_SHOOT_DELAY){
    nextHoleTime += delta;
  }
  return nextHoleTime;
}
