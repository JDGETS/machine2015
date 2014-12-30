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

  _Detected = (_DistanceValue < MIN_TRIGGER_VALUE && _DistanceValue != 0 );

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
  while(!IsDetected()) ReadInput();
  lastStatusChange = millis();
}

void UltrasonicSensor::WaitForUndetect()
{
  while(millis() < lastStatusChange + DEBOUNCE_TIME); //Debounce
  while(IsDetected()) ReadInput();
  lastStatusChange = millis();
}