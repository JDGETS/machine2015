#include "ReedSwitch.h"


///////////////////////////////////////////////////////////////////////////////
/// ----------------------------------------------------------------------- ///
///////////////////////////////////////////////////////////////////////////////

ReedSwitch::ReedSwitch(uint8_t PIN)
    : PIN(PIN), _Active(false)
{};

void ReedSwitch::Setup()
{
    pinMode(PIN, INPUT_PULLUP);
};

const bool ReedSwitch::IsActive()
{
    _Active = (digitalRead(PIN) == 0);
    return _Active;
};

///////////////////////////////////////////////////////////////////////////////
/// ----------------------------------------------------------------------- ///
///////////////////////////////////////////////////////////////////////////////


PairedReedSwitch::PairedReedSwitch(uint8_t TOP_PIN, uint8_t BOTTOM_PIN)
    : TOP_PIN(TOP_PIN), BOTTOM_PIN(BOTTOM_PIN),
      _TopReedSwitch(TOP_PIN), _BottomReedSwitch(BOTTOM_PIN)
{};

void PairedReedSwitch::Setup()
{
    _TopReedSwitch.Setup();
    _BottomReedSwitch.Setup();
};

const bool PairedReedSwitch::IsBottomReedSwitchActive()
{
    return _BottomReedSwitch.IsActive();
};

const bool PairedReedSwitch::IsTopReedSwitchActive()
{
    return _TopReedSwitch.IsActive();
};

const bool PairedReedSwitch::IsInRange()
{
    return (_BottomReedSwitch.IsActive() && _TopReedSwitch.IsActive());
};

///////////////////////////////////////////////////////////////////////////////
/// ----------------------------------------------------------------------- ///
///////////////////////////////////////////////////////////////////////////////

VehicleReedSwitches::VehicleReedSwitches(PairedReedSwitch indexZero, PairedReedSwitch indexOne,
        PairedReedSwitch indexTwo, PairedReedSwitch indexThree)
        : ARRAY({indexOne, indexOne, indexOne, indexOne})
{};

void VehicleReedSwitches::Setup()
{
    ARRAY[0].Setup();
    ARRAY[1].Setup();
    ARRAY[2].Setup();
    ARRAY[3].Setup();
};

unsigned int VehicleReedSwitches::BinValue()
{
    int value = 0;
    for(int i = 0; i < 4; i++)
    {
        value += ARRAY[i].IsInRange() << i;
    }
    return value;
};


