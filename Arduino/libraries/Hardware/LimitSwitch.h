#ifndef LIMITSWITCH_H
#define LIMITSWITCH_H
#include <Arduino.h>

struct LimitSwitch {

  LimitSwitch(const uint8_t PIN) 
    : PIN(PIN), DEBOUNCE_TIME(100), active(false), lastStatusChange(0){ };

  LimitSwitch(const uint8_t PIN, const unsigned long DEBOUNCE_TIME) 
    : PIN(PIN), DEBOUNCE_TIME(DEBOUNCE_TIME), active(false), lastStatusChange(0){ };

  void Setup();

  const bool IsActive() const;
  const bool ReadInput();

  void ResetDebouncing();
  void WaitForPress();
  void WaitForRelease();

private:
  const uint8_t PIN;
  const unsigned long DEBOUNCE_TIME;
  bool active;
  unsigned long lastStatusChange;
};

#endif