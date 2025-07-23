#pragma once
#include <stdint.h>

// This file includes variables used in functions to track 
// and regulate loop rate, etc.


namespace Timing {
    extern float dt; // microseconds
    extern unsigned long current_time;
    extern unsigned long prev_time;
    extern uint16_t loop_freq;  // Hz

    void begin(); // Just sets up the LED for the loop_blink() function
    void update();
    void loop_rate();
    void set_loop_freq(uint16_t freq);
    void loop_blink();
    void heartbeat_LED();

}