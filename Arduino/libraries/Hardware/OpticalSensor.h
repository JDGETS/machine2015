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
            : ANALOG_VO_PIN(ANALOG_VO_PIN), 
            _AnalogValue(0.0), 
            _Active(false), 
            inverted(false),
            LONG_READ_TIME(250), 
            DEBOUNCE_TIME(100), 
            MIN_ACTIVE_VALUE(4.80) {};

    void Setup();

    const float & ReadInput();
    const float & AnalogValue();
    const bool IsActive() const;
    const bool LongReadInput(bool ifUnsure);

    void ResetDebouncing();
    void WaitForActive();
    void WaitForInactive();

    void Invert(); // By default, 1023 = something is in front, 0 = infinite distance, if you want to invert this, call Invert()

    void SetMinActiveValue(float min){ MIN_ACTIVE_VALUE = min; };
    void SetDebounceTime(unsigned long millis){ DEBOUNCE_TIME = millis; };
    void SetLongReadTime(unsigned long micros){ LONG_READ_TIME = micros; };
private:
    float MIN_ACTIVE_VALUE;
    unsigned long LONG_READ_TIME;
    unsigned long DEBOUNCE_TIME;
    float _AnalogValue;
    bool _Active;
    bool inverted;
    unsigned long lastStatusChange;
};

#endif