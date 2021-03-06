#include "ForceStopVehicle.h"
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

float & OpticalSensor::ReadInput()
{
  _AnalogValue = analogRead(ANALOG_VO_PIN) * (5.0 / 1023.0);

  if(_Inverted)
    _AnalogValue = 5 - _AnalogValue;

  _Detected = (_AnalogValue < MIN_TRIGGER_VALUE );

  return _AnalogValue;
}

const float & OpticalSensor::AnalogValue() 
{
  return _AnalogValue;
};

const bool OpticalSensor::IsDetected() const 
{
  return _Detected;
};

bool OpticalSensor::LongReadInput(bool ifUnsure)
{
    unsigned long delay = LONG_READ_TIME;
    unsigned long start = micros();
    unsigned long count = 0;
    unsigned long total = 0;
    while(true)
    {
        count++;
        ReadInput();
        total += (int)(IsDetected());
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

void OpticalSensor::Invert(){
  _Inverted = !_Inverted;
}

void OpticalSensor::ResetDebouncing()
{
  lastStatusChange = 0;
}

void OpticalSensor::WaitForDetect()
{
  while(millis() < lastStatusChange + DEBOUNCE_TIME); //Debounce
  if(useLongRead)
    while(!LongReadInput(false)) {
      CHECK_FORCE_STOP_MACRO
    }
  else
    while(!IsDetected()){ 
      ReadInput(); 
      CHECK_FORCE_STOP_MACRO
    }
  lastStatusChange = millis();
}

void OpticalSensor::WaitForUndetect()
{
  while(millis() < lastStatusChange + DEBOUNCE_TIME); //Debounce
  if(useLongRead){
    while(LongReadInput(true)){
      CHECK_FORCE_STOP_MACRO;
    };
  }
  else
    while(IsDetected()){ 
      ReadInput(); 
      CHECK_FORCE_STOP_MACRO; 
    }
  lastStatusChange = millis();
}