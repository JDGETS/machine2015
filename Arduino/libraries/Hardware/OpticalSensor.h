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

    void Setup();

    const float & ReadInputWithoutBounce();
    const float & ReadInputWithBounce();
    const float & AnalogValue();
    const bool IsActive() const;
    const bool IsActiveWithoutBounce();
    const bool IsActiveWithBounce();
    const bool LongReadInput(bool ifUnsure);

private:
	static const float MIN_ACTIVE_VALUE = 4.80;
    static const unsigned long BOUNCING_VALUE = 30;
    float _AnalogValue;
    bool _Active;
    unsigned long _lastRead;
};

#endif