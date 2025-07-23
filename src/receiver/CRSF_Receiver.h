#pragma once

#include <Arduino.h>
#include "CRSFforArduino.hpp"
#include "Receiver_Interface.h"

class CRSF_Receiver : public Receiver_Interface {
public:
    CRSF_Receiver(HardwareSerial *serial);
    ~CRSF_Receiver();

    bool begin() override;
    void update() override;
    uint16_t getChannelUs(uint8_t channel) const override;
    bool isFailsafe() const override;
    bool isReady() const override;

private:
    static void onReceiveRcChannels(serialReceiverLayer::rcChannels_t* rcChannels);
    void handleChannels(serialReceiverLayer::rcChannels_t* rcChannels);

    static CRSF_Receiver* instance;
    CRSFforArduino* crsf;

    uint16_t channelsUs[8];
    bool failsafe;
    bool ready;
};

