#include "LimitSwitch.h"

void LimitSwitch::Setup() { 
  pinMode(PIN, INPUT_PULLUP); 
  lastStatusChange = millis();
};

const bool LimitSwitch::IsActive() const 
{
  return active; 
};

const bool LimitSwitch::ReadInput()
{
    active = (digitalRead(PIN) == 0);
    return IsActive();
};

void LimitSwitch::ResetDebouncing()
{
  lastStatusChange = 0;
}

void LimitSwitch::WaitForPress()
{
  while(millis() < lastStatusChange + DEBOUNCE_TIME); //Debounce
  while(digitalRead(PIN) != 0);
  lastStatusChange = millis();
}

void LimitSwitch::WaitForRelease()
{
  while(millis() < lastStatusChange + DEBOUNCE_TIME); //Debounce
  while(digitalRead(PIN) == 0);
  lastStatusChange = millis();
}