#ifndef OPTICAL_SENSOR_H
#define OPTICAL_SENSOR_H
#include <Arduino.h>

// 2 rose 2 gris .... 1 gris 1 rose

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
            _Detected(false), 
            _Inverted(false), 
            LONG_READ_TIME(250), 
            DEBOUNCE_TIME(100), 
            MIN_TRIGGER_VALUE(4.80) {};

    void Setup();

    const float & ReadInput();
    const float & AnalogValue();
    const bool IsDetected() const;
    const bool LongReadInput(bool ifUnsure);

    void ResetDebouncing();
    void WaitForDetect(bool useLongRead = true);
    void WaitForUndetect(bool useLongRead = true);

    void Invert(); // By default, 0 is infinite distance

    void SetMinTriggerValue(float min){ MIN_TRIGGER_VALUE = min; };
    void SetDebounceTime(unsigned long millis){ DEBOUNCE_TIME = millis; };
    void SetLongReadTime(unsigned long micros){ LONG_READ_TIME = micros; };
private:
    float MIN_TRIGGER_VALUE;
    unsigned long LONG_READ_TIME;
    unsigned long DEBOUNCE_TIME;
    float _AnalogValue;
    bool _Detected;
    bool _Inverted;
    unsigned long lastStatusChange;
};

#endif