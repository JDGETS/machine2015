#include "OpticalSensor.h"

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- OPTICAL SENSOR ---------------------------------------------------- ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

void OpticalSensor::Setup() 
{
  pinMode(ANALOG_VO_PIN, INPUT);
};

const float & OpticalSensor::ReadInputWithoutBounce()
{
    _lastRead = millis();
    _AnalogValue = analogRead(ANALOG_VO_PIN) * (5.0 / 1023.0);
    _Active = (_AnalogValue > OPTICAL_SENSOR_MIN_ACTIVE_VALUE);

    return _AnalogValue;
}

const float & OpticalSensor::ReadInputWithBounce()
{
    if (_lastRead + BOUNCING_VALUE < millis())
    {
        _lastRead = millis();
        _AnalogValue = analogRead(ANALOG_VO_PIN) * (5.0 / 1023.0);
        _Active = (_AnalogValue > OPTICAL_SENSOR_MIN_ACTIVE_VALUE);
    }

    return _AnalogValue;
};

const float & OpticalSensor::AnalogValue() 
{
  return _AnalogValue;
};

const bool OpticalSensor::IsActive() const 
{
  return _Active;
};

const bool OpticalSensor::IsActiveWithoutBounce()
{
    ReadInputWithoutBounce();
    return _Active;
}

const bool OpticalSensor::IsActiveWithBounce()
{
    ReadInputWithBounce();
    return _Active;
};
