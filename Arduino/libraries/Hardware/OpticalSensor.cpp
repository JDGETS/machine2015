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
    _Active = (_AnalogValue > MIN_ACTIVE_VALUE );

    return _AnalogValue;
}

const float & OpticalSensor::ReadInputWithBounce()
{
    if (_lastRead + BOUNCING_VALUE < millis())
    {
        _lastRead = millis();
        _AnalogValue = analogRead(ANALOG_VO_PIN) * (5.0 / 1023.0);
        _Active = (_AnalogValue > MIN_ACTIVE_VALUE);
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

// timing selon le state
// prendre le controle
// comparer les x nombres de derniere valeur
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

const bool OpticalSensor::LongReadInput(bool ifUnsure)
{
    int delay = 250;
    unsigned long start = micros();
    unsigned long count = 0;
    unsigned long total = 0;
    while(true)
    {
        count++;
        total += (int)(analogRead(ANALOG_VO_PIN) * (5.0 / 1023.0) > MIN_ACTIVE_VALUE);
        delayMicroseconds(20);

        if(micros() - start < delay)
        {
            if(total == 0)
            {
                return false;
            }
            else if(total == count)
            {
                return true;
            }
            else
            {
                return ifUnsure;
            }
        }
    }
};