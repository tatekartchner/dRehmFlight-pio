#pragma once
#include <stdint.h>

namespace Receiver_Manager {
    void init();
    void update();

    bool isReady();
    bool isFailsafe();
    uint16_t getChannelUs(uint8_t channel);
}