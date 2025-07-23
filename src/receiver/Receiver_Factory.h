#pragma once

#include "Receiver_Interface.h"
#include "config.h"

#ifdef USE_CRSF_RX
#include "CRSF_Receiver.h"
#endif

#ifdef USE_PWM_RX
#include "PWM_Receiver.h" // Not yet created
#endif


static Receiver_Interface& createRadioReceiver(HardwareSerial* serial) {
    #ifdef USE_CRSF_RX
        static CRSF_Receiver instance(serial);
        return instance;
    #elif defined USE_PWM_RX
        static PWM_Receiver instance;
        return instance;
    #else
        #error "No receiver type defined in config.h"
    #endif
}


