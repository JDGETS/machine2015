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
  if(inverted)
    _AnalogValue = analogRead(ANALOG_VO_PIN) * (5.0 / 1023.0);
  else
    _AnalogValue = (1023 - analogRead(ANALOG_VO_PIN)) * (5.0 / 1023.0);
  _Active = (_AnalogValue > MIN_ACTIVE_VALUE );

  return _AnalogValue;
}

const float & OpticalSensor::AnalogValue() 
{
  return _AnalogValue;
};

const bool OpticalSensor::IsActive() const 
{
  return _Active;
};

const bool OpticalSensor::LongReadInput(bool ifUnsure)
{
    unsigned long delay = LONG_READ_TIME;
    unsigned long start = micros();
    unsigned long count = 0;
    unsigned long total = 0;
    while(true)
    {
        count++;
        ReadInput();
        total += (int)(IsActive());
        delayMicroseconds(20);

        if(micros() - start > delay)
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

void OpticalSensor::Invert() 
{
  inverted = !inverted;
};

void OpticalSensor::ResetDebouncing()
{
  lastStatusChange = 0;
}

void OpticalSensor::WaitForActive()
{
  while(millis() < lastStatusChange + DEBOUNCE_TIME); //Debounce
  while(!IsActive()) ReadInput();
  lastStatusChange = millis();
}

void OpticalSensor::WaitForInactive()
{
  while(millis() < lastStatusChange + DEBOUNCE_TIME); //Debounce
  while(IsActive()) ReadInput();
  lastStatusChange = millis();
}