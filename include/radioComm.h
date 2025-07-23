#ifndef RADIOCOMM_H
#define RADIOCOMM_H
//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Last Updated: 7/29/2022
//Version: Beta 1.3


#include "config.h"
//put your function headers here



//========================================================================================================================//

//This file contains all necessary functions used for radio communication to avoid cluttering the main code

void radioSetup(); //Sets up radio communication

unsigned long getRadioPWM(int ch_num); //DESCRIPTION: Get current radio commands from interrupt routines 

//For DSM type receivers
void serialEvent3(void);

// //For CRSF type receivers
// void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels);

//========================================================================================================================//

//INTERRUPT SERVICE ROUTINES (for reading PWM and PPM)

void getPPM();

void getCh1(); 

void getCh2();

void getCh3();

void getCh4();

void getCh5();

void getCh6();

#endif