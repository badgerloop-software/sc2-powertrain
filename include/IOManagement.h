#ifndef __IO_MANAGER_H__
#define __IO_MANAGER_H__

#include <Arduino.h>
#include "STM32TimerInterrupt_Generic.h"

// TODO: add macros for digital pins

struct Digital_Data {
    // TODO: add the digital signals
    bool batt_neg_cont : 1;
};

extern volatile Digital_Data digital_data;

// TODO: add extern volatile floats for analog signals.

// initialize digital and analog pins
void initIO();

// read digital and analog inputs
void readIO();

// Set the value of output pins
// TODO: add setter functions for output pins.

#endif
