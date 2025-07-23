#ifndef RECEIVER_INTERFACE_H
#define RECEIVER_INTERFACE_H

#include <stdint.h>

class Receiver_Interface {
public:
    virtual ~Receiver_Interface() {}
    virtual bool begin() = 0;
    virtual void update() = 0;
    virtual uint16_t getChannelUs(uint8_t channel) const = 0;
    virtual bool isFailsafe() const = 0;
    virtual bool isReady() const = 0;
};

#endif
