#include <Arduino.h>

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- XBEE COMMUNICATION ------------------------------------------------ ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

struct XBeeComm : SoftwareSerial {
    const uint8_t RX_PIN, TX_PIN;
    const long COMM_BAUDRATE;
    XBeeComm(uint8_t RX_PIN, uint8_t TX_PIN, uint16_t COMM_BAUDRATE)
            : SoftwareSerial(RX_PIN, TX_PIN), RX_PIN(RX_PIN), TX_PIN(TX_PIN),
              COMM_BAUDRATE(COMM_BAUDRATE) {};
    void Setup() { begin(COMM_BAUDRATE); delayMicroseconds(100); };
    const char ReadInput() { return ( available() ) ? read() : 0; };
};

