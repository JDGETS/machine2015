#ifndef REED_SWITCH_H
#define REED_SWITCH_H

#include <Arduino.h>

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- REED SWITCH ------------------------------------------------------- ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

class ReedSwitch {

public:
    const uint8_t PIN;
    ReedSwitch(uint8_t PIN);
    void Setup();
    const bool IsActive();

private:
    bool _Active;
};

class PairedReedSwitch {

public:
    const uint8_t TOP_PIN, BOTTOM_PIN;

    PairedReedSwitch(uint8_t TOP_PIN, uint8_t BOTTOM_PIN);
    void Setup();
    const bool IsBottomReedSwitchActive();
    const bool IsTopReedSwitchActive();
    const bool IsInRange();

private:
    ReedSwitch _TopReedSwitch, _BottomReedSwitch;
};

class VehicleReedSwitches {

public:
    PairedReedSwitch ARRAY[4];

    VehicleReedSwitches(PairedReedSwitch indexZero, PairedReedSwitch indexOne,
            PairedReedSwitch indexTwo, PairedReedSwitch indexThree);

    void Setup();
    unsigned int BinValue();
};

#endif // REED_SWITCH_H