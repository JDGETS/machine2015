#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H
#include <Arduino.h>
#include <NewPing.h>

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- ULTRASONIC SENSOR ------------------------------------------------- ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

struct UltrasonicSensor {
    // Max distance is in CM
    UltrasonicSensor(uint8_t TRIGGER_PIN, uint8_t ECHO_PIN, uint8_t MAX_DISTANCE)
            : sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE), 
            _DistanceValue(0.0),
            _Detected(false),
            DEBOUNCE_TIME(100), 
            MIN_TRIGGER_VALUE(10.0),
            lastStatusChange(0),
            consecutiveZeros(0) {};

    void Setup();

    float & ReadInput();
    const float & DistanceValue();
    const bool IsDetected() const;

    void ResetDebouncing();
    void WaitForDetect();
    void WaitForUndetect();

    void SetMinTriggerValue(float min){ MIN_TRIGGER_VALUE = min; };
    void SetDebounceTime(unsigned long millis){ DEBOUNCE_TIME = millis; };
    void SetConsecutiveZerosN(unsigned long n){ CONSECUTIVE_ZEROS_N = n; };
private:
    NewPing sonar;
    unsigned long WAIT_READ_DELAY = 15;
    unsigned long CONSECUTIVE_ZEROS_N = 0; // The number of zeros one must read to truly make it a zero.
    float MIN_TRIGGER_VALUE;
    unsigned long DEBOUNCE_TIME;
    unsigned long consecutiveZeros;
    float _DistanceValue;
    bool _Detected;
    unsigned long lastStatusChange;
};

#endif