#include "ForceStopVehicle.h"
#include <UltrasonicSensor.h>
#include <NewPing.h>

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- ULTRASONIC SENSOR ------------------------------------------------- ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

void UltrasonicSensor::Setup() 
{
  // Nothing to do here. Setup is in the NewPing constructor.
};

float & UltrasonicSensor::ReadInput()
{
  _DistanceValue = sonar.ping() / US_ROUNDTRIP_CM;

  bool detected = (_DistanceValue < MIN_TRIGGER_VALUE && _DistanceValue != 0 );
//Serial.print(_DistanceValue);Serial.print(",");
  if(_DistanceValue == 0){
    consecutiveZeros++;
    if(consecutiveZeros < CONSECUTIVE_ZEROS_N){
      return _DistanceValue; //Don't set _Detected
    }
  }
  else
    consecutiveZeros = 0;

  _Detected = detected;

  return _DistanceValue;
}

const float & UltrasonicSensor::DistanceValue() 
{
  return _DistanceValue;
};

const bool UltrasonicSensor::IsDetected() const 
{
  return _Detected;
};

void UltrasonicSensor::ResetDebouncing()
{
  lastStatusChange = 0;
}

void UltrasonicSensor::WaitForDetect()
{
  while(millis() < lastStatusChange + DEBOUNCE_TIME); //Debounce
  ReadInput();
  while(!IsDetected()){ delay(WAIT_READ_DELAY); ReadInput(); CHECK_FORCE_STOP_MACRO; }
  lastStatusChange = millis();
}

void UltrasonicSensor::WaitForUndetect()
{
  while(millis() < lastStatusChange + DEBOUNCE_TIME); //Debounce
  ReadInput();
  while(IsDetected()){ delay(WAIT_READ_DELAY); ReadInput(); CHECK_FORCE_STOP_MACRO; }
  lastStatusChange = millis();
}