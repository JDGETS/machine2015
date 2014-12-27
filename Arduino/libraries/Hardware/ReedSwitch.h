#include <Arduino.h>

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- REED SWITCH ------------------------------------------------------- ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

struct ReedSwitch {
    const uint8_t PIN;
    ReedSwitch(uint8_t PIN) : PIN(PIN), active(false) {};
    void Setup() { pinMode(PIN, INPUT_PULLUP); };
    const bool IsActive() const { return active; };
    const bool ReadInput()
    {
        active = (digitalRead(PIN) == 0);
        return IsActive();
    };

private:
    bool active;
};
