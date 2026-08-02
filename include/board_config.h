#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

// all board constants so they are easy to change

// ------------- TIMING -------------
#define IO_UPDATE_PERIOD 100000  // us
#define DATA_SEND_PERIOD 50      // ms, CAN TX (+ debug if on)

// ------------- DEBUG -------------
// only enable when serial is connected
// 0 off (not in flash) | 1 human
#define SC2_DEBUG 0

// ------------- IO PINS -------------
// Inputs
#define BATT_NEG_CONT_MCU PB3
#define ESTOP_MCU PA9
#define BATT_POS_CONT_MCU PA10
#define PPC1_SUPP_INVALID PB1
#define PPC1_DCDC_INVALID PB0
#define MPPT_CONT_MCU PB5
#define MC_CONT_MCU PB4

// Outputs
#define MCU_BATT_EN PA8

// ------------- BPS TELEMETRY SCALING -------------
// Each B1/B2 pair is big-endian uint16: B1 MSB, B2 LSB.
// Cell voltages use 0.1 mV/bit (34409 represents 3.4409 V).
#define BPS_CELL_VOLTAGE_SCALE_V 0.0001f
#define BPS_PACK_CURRENT_SCALE_A 0.1f

// Inclusive operating limits
#define BPS_CELL_VOLTAGE_MIN_V 2.52f
#define BPS_CELL_VOLTAGE_MAX_V 4.18f
#define BPS_PACK_CURRENT_MIN_A 0U
#define BPS_PACK_CURRENT_MAX_A 35U
#define BPS_TEMPERATURE_MIN_C 4U
#define BPS_TEMPERATURE_MAX_C 56U

// Ignore BPS telemetry faults while its transmitters finish starting up.
#define BPS_FAULT_STARTUP_GRACE_MS 2000UL

#endif  // __BOARD_CONFIG_H__
