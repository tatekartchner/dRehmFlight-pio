#include "CRSF_Receiver.h"

CRSF_Receiver* CRSF_Receiver::instance = nullptr;

CRSF_Receiver::CRSF_Receiver(HardwareSerial *serial)
    : crsf(new CRSFforArduino(serial)), failsafe(true), ready(false)
{
    for (uint8_t i = 0; i < 8; ++i) {
        channelsUs[i] = 1500;
    }
}

CRSF_Receiver::~CRSF_Receiver() {
    delete crsf;
    crsf = nullptr;
}

bool CRSF_Receiver::begin() {
    if (!crsf->begin()) {
        Serial.println("CRSF failed to initialize.");
        return false;
    }

    instance = this;
    crsf->setRcChannelsCallback(onReceiveRcChannels);
    return true;
}

void CRSF_Receiver::update() {
    if (crsf) {
        crsf->update();
    }
}

void CRSF_Receiver::onReceiveRcChannels(serialReceiverLayer::rcChannels_t* rcChannels) {
    if (instance) {
        instance->handleChannels(rcChannels);
    }
}

void CRSF_Receiver::handleChannels(serialReceiverLayer::rcChannels_t* rcChannels) {
    failsafe = rcChannels->failsafe;
    if (!failsafe) {
        for (uint8_t i = 0; i < 8; ++i) {
            channelsUs[i] = crsf->rcToUs(rcChannels->value[i]);
        }
        ready = true;
    }
}

uint16_t CRSF_Receiver::getChannelUs(uint8_t channel) const {
    if (channel < 8) {
        return channelsUs[channel];
    }
    return 1500;
}

bool CRSF_Receiver::isFailsafe() const {
    return failsafe;
}

bool CRSF_Receiver::isReady() const {
    return ready;
}
