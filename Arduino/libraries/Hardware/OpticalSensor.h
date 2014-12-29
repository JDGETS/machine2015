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
            useLongRead(false), 
            LONG_READ_TIME(2500), 
            DEBOUNCE_TIME(100), 
            MIN_TRIGGER_VALUE(4.80),
            IGNORE_ZEROS(false) {};

    void Setup();

    float & ReadInput();
    const float & AnalogValue();
    const bool IsDetected() const;
    bool LongReadInput(bool ifUnsure);

    void ResetDebouncing();
    void WaitForDetect();
    void WaitForUndetect();

    void Invert(); // By default, 0 is infinite distance

    void SetMinTriggerValue(float min){ MIN_TRIGGER_VALUE = min; };
    void SetDebounceTime(unsigned long millis){ DEBOUNCE_TIME = millis; };
    void SetLongReadTime(unsigned long micros){ LONG_READ_TIME = micros; };
    void SetUseLongRead(bool useLongRead){ this->useLongRead = true; };
private:
    float MIN_TRIGGER_VALUE;
    unsigned long LONG_READ_TIME;
    unsigned long DEBOUNCE_TIME;
    unsigned long IGNORE_ZEROS;
    float _AnalogValue;
    bool _Detected;
    bool _Inverted;
    bool useLongRead;
    unsigned long lastStatusChange;
};

#endif