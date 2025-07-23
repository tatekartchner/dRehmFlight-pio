#include "timing.h"
#include <Arduino.h>

namespace Timing {

    float dt = 0.0f;
    unsigned long current_time = 0;
    unsigned long prev_time = 0;
    unsigned long print_counter = 0;
    unsigned long serial_counter = 0;
    unsigned long last_blink_time = 0;
    unsigned long blink_delay = 0;
    static bool led_on = false;
    static int blink_state = 0; // 0,1,2,3 cycle
    static const int led_pin = 13;
    bool blink_alternate = false;
    uint16_t loop_freq = 2000; // 2kHz default
    unsigned long loop_interval = 500;  // default for 2kHz

    void begin() {
        pinMode(13, OUTPUT);
        digitalWrite(13, LOW); // ensure LED is off initially
    }

    void set_loop_freq(uint16_t freq) {
        loop_freq = freq;
        loop_interval = 1000000UL / freq;
    }

    void update() {
        prev_time = current_time;
        current_time = micros();
        dt = (current_time - prev_time) / 1000000.0f; // dt in seconds
    }

    void loop_rate() {
        //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
        /*
        * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
        * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
        * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
        * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
        * and remain above 2kHz, without needing to retune all of our filtering parameters.
        */
        unsigned long checker = micros();
        //Sit in loop until appropriate time has passed
        while ((checker - current_time) < loop_interval) {
            checker = micros();
        }
    }

    void loop_blink() {
        //DESCRIPTION: Blink LED on board to indicate main loop is running
        /*
        * It looks cool.
        * Make sure you've called Timing::begin() in setup() 
        * if you're going to use this function.
        */
        if (micros() - last_blink_time > blink_delay) {
            last_blink_time = micros();

            digitalWrite(13, blink_alternate); //Pin 13 is built in LED

            if (blink_alternate) {
                blink_alternate = false;
                blink_delay = 100000;   // LED off for 100 ms
            } else {
                blink_alternate = true;
                blink_delay = 2000000;  // LED on for 2 sec
            }
        }
    }

    void heartbeat_LED() {
        //DESCRIPTION: Blink LED on board in heartbeat pattern to 
        // indicate main loop is running
        /*
        * It looks cool.
        * Make sure you've called Timing::begin() in setup() 
        * if you're going to use this function.
        */
        unsigned long now = micros();

        switch (blink_state) {
            case 0: // First beat ON
                if (!led_on) {
                    digitalWrite(led_pin, HIGH);
                    led_on = true;
                    last_blink_time = now;
                } else if (now - last_blink_time > 100000) { // 100ms
                    digitalWrite(led_pin, LOW);
                    led_on = false;
                    last_blink_time = now;
                    blink_state = 1;
                }
                break;

            case 1: // Short pause
                if (now - last_blink_time > 100000) { // 100ms
                    blink_state = 2;
                }
                break;

            case 2: // Second beat ON
                if (!led_on) {
                    digitalWrite(led_pin, HIGH);
                    led_on = true;
                    last_blink_time = now;
                } else if (now - last_blink_time > 200000) { // 200ms
                    digitalWrite(led_pin, LOW);
                    led_on = false;
                    last_blink_time = now;
                    blink_state = 3;
                }
                break;

            case 3: // Long pause
                if (now - last_blink_time > 1600000) { // 1.6 sec pause
                    blink_state = 0;
                }
                break;
        }
    }

    
}