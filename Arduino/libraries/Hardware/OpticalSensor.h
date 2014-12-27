#ifndef OPTICAL_SENSOR_H
#define OPTICAL_SENSOR_H
#include <Arduino.h>

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- OPTICAL SENSOR ---------------------------------------------------- ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

struct OpticalSensor {
    const uint8_t ANALOG_VO_PIN;
    OpticalSensor(uint8_t ANALOG_VO_PIN)
            : ANALOG_VO_PIN(ANALOG_VO_PIN), _Active(false), _AnalogValue(0.0) {};

    void Setup();;

    const float & ReadInput();

    const float & AnalogValue();
    const bool IsActive() const;

private:
    float _AnalogValue;
    bool _Active;
};

#endif