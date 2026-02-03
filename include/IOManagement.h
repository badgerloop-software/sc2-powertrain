#ifndef __IO_MANAGER_H__
#define __IO_MANAGER_H__

#include <Arduino.h>
#include "STM32TimerInterrupt_Generic.h"
#include "adc.h"

// Inputs
#define BATT_NEG_CONT_MCU           PB3
#define ESTOP_MCU                   PA9
#define BATT_POS_CONT_MCU           PA10
#define PPC1_SUPP_INVALID           PA12
#define PPC1_DCDC_INVALID           PB0
#define MPPT_CONT_MCU               PB5
#define MC_CONT_MCU                 PB4

// Outputs
#define MCU_BATT_EN                 PA8

#define IO_UPDATE_PERIOD 100000 // us


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

extern volatile Digital_Data digital_data;

// Analog signals
extern volatile float i_12v;
extern volatile float v_12v;
extern volatile float supp_i;
extern volatile float batt_i;
extern volatile float supp_v;

// initialize digital and analog pins
void initIO();

// read digital and analog inputs
void readIO();

// Set the value of output pins
void set_mcu_batt_en(bool batt_en);

#endif
