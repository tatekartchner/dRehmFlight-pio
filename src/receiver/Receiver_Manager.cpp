#include "Receiver_Manager.h"
#include "Receiver_Factory.h"
#include "config.h"

namespace Receiver_Manager {
    static Receiver_Interface* receiver = nullptr;

    void init() {
        receiver = &createRadioReceiver(&Config::crsf_Serial); // Change Serial4 if needed
        receiver->begin();
    }

    void update() {
        if (receiver) receiver->update();
    }

    bool isReady() {
        return receiver && receiver->isReady();
    }

    bool isFailsafe() {
        return receiver && receiver->isFailsafe();
    }

    uint16_t getChannelUs(uint8_t channel) {
        if (receiver) return receiver->getChannelUs(channel);
        // Use per-channel failsafe value if available
        if (channel < Config::failsafeChannelCount) {
            return Config::failsafeValues[channel]; 
        }

        // Fallback hard default
        return 1500;
    }
} 
