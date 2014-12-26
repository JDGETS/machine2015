#define USE_STORE_SERVO
#include <Servo.h>
#include "Store.h"

#define SHOOTER_SWITCH_PIN 38
#define BAG_HOLDER_SWITCH_PIN 40
#define STORE_SWITCH_PIN 12
#define SHOOTER_MOTOR_MOSFET 7
//#define USE_STORE_SERVO

struct LimitSwitch {
    const uint8_t PIN;
    LimitSwitch(uint8_t PIN) : PIN(PIN), active(false), lastStatusChange(0) {};
    void Setup() { pinMode(PIN, INPUT_PULLUP); };
    const bool IsActive() const { return active; };
    const bool ReadInput()
    {
        active = (digitalRead(PIN) == 0);
        return IsActive();
    };
    
    unsigned long lastStatusChange;
    const static int DEBOUNCE_TIME = 100;
    
    void WaitForPress(){
      while(millis() < lastStatusChange + DEBOUNCE_TIME); //Debounce
      while(digitalRead(PIN) == 0);
      lastStatusChange = millis();
    }
    
    void WaitForRelease(){
      while(millis() < lastStatusChange + DEBOUNCE_TIME); //Debounce
      while(digitalRead(PIN) != 0);
      lastStatusChange = millis();
    }

private:
    bool active;
};

unsigned long nextHoleTime = -1;
unsigned long delta = -1;
unsigned long LOAD_AND_SHOOT_DELAY = 500; // Time it takes to charge and shoot the bag and time until it reaches the hole (millis)
unsigned long BAG_FREE_FALL_DELAY = 500; //Time for the bag to fall into place. Put some padding in there.
unsigned long STORE_ANGLE = 50;
unsigned long bagsLeft = 8;

LimitSwitch shooterSwitch(SHOOTER_SWITCH_PIN);
LimitSwitch bagHolderSwitch(BAG_HOLDER_SWITCH_PIN);
void setup() 
{
  Serial.begin(9600);
  States::store.Setup();
  
  pinMode(SHOOTER_SWITCH_PIN, INPUT_PULLUP);
  pinMode(BAG_HOLDER_SWITCH_PIN, INPUT_PULLUP);
  //pinMode(STORE_SWITCH_PIN, INPUT_PULLUP);  
  pinMode(SHOOTER_MOTOR_MOSFET, OUTPUT);
  digitalWrite(SHOOTER_MOTOR_MOSFET, LOW);  
  
  States::store.Initialize();

  States::store.bagsCount = 8; //The store is considered full here. //TO-DO: Remove when Charging state is done.

  digitalWrite(SHOOTER_MOTOR_MOSFET, HIGH);
  delay(1000);
  digitalWrite(SHOOTER_MOTOR_MOSFET, LOW);  
}
 
void loop() 
{
  unsigned long firstHole[2], secondHole[2];
  unsigned long firstHoleTime, secondHoleTime; 
  
  Serial.print("Start time: ");
  unsigned long time = millis();
  Serial.println(time);

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
  nextHoleTime = secondHole[1];
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
    //StartMoteur
    bagHolderSwitch.WaitForRelease();
    bagHolderSwitch.WaitForPress();
    //EndMoteur (if it cuts too late, 2 cases: 1) elastics strong enough to bring back the shooter at its place. no problem then. 2) not strong enough. then you have to adjust LOAD_AND_SHOOT_DELAY for bags #2-8).
    bagsLeft--;
    Serial.print("BOOM! Only ");
    Serial.print(bagsLeft);
    Serial.println(" bags left!");
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
  States::store.ServoMoveBag();
  return;
  
#ifdef USE_STORE_SERVO
    States::store.ServoUnloadBag();
#else //With Moteur
  unsigned long MOTOR_STEP_TIME = 20;
  while(digitalRead(STORE_SWITCH_PIN) != 0){
    //moteurOn
    delay(MOTOR_STEP_TIME);
    //moteurOff
  }
  while(digitalRead(STORE_SWITCH_PIN) != 0){
    //moteurOn
    delay(MOTOR_STEP_TIME);
    //moteurOff
  }
#endif
  delay(BAG_FREE_FALL_DELAY); //Wait for the bag to fall into place.
}

unsigned long GetNextShootingTime(){ //Must take into account LOAD_AND_SHOOT_DELAY. This should be the next hole that is theoritically shootable in. Assume computing time is negligeable.
  unsigned long currentTime = millis();
  while(nextHoleTime < currentTime + LOAD_AND_SHOOT_DELAY){
    nextHoleTime += delta;
  }
  return nextHoleTime;
}
