#ifndef __IO_MANAGEMENT_H__
#define __IO_MANAGEMENT_H__

#include <Arduino.h>
#include <HardwareTimer.h>
// TimerInterrupt_Generic 1.13 expects MICROSEC_FORMAT as a bare name
#ifndef MICROSEC_FORMAT
#define MICROSEC_FORMAT TimerFormat_t::MICROSEC_FORMAT
#endif
#include "STM32TimerInterrupt_Generic.h"
#include "adc.h"
#include "board_config.h"

// ------------- TYPES -------------

struct Digital_Data {
    bool batt_neg_cont : 1;     // input
    bool estop_mcu : 1;         // input
    bool batt_pos_cont : 1;     // input
    bool ppc1_supp_invalid : 1; // input
    bool ppc1_dcdc_invalid : 1; // input
    bool mcu_batt_en : 1;       // output
    bool mppt_cont_mcu : 1;     // input
    bool mc_cont_mcu : 1;       // input
};

// ------------- GLOBALS -------------

extern volatile Digital_Data digital_data;

extern volatile float i_12v;
extern volatile float v_12v;
extern volatile float supp_i;
extern volatile float batt_i;
extern volatile float supp_v;

// ------------- FUNCTIONS -------------

// initialize digital and analog pins
void initIO();

// read digital and analog inputs
void readIO();

// set the value of output pins
void set_mcu_batt_en(bool batt_en);

#endif  // __IO_MANAGEMENT_H__
