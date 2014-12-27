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

const float & OpticalSensor::ReadInput()
{
  _AnalogValue = analogRead(ANALOG_VO_PIN) * (5.0 / 1023.0);
  _Active = (_AnalogValue > 4.70);
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