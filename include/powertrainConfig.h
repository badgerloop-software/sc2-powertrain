#ifndef __POWERTRAIN_CONFIG_H__
#define __POWERTRAIN_CONFIG_H__

// BPS and fault CAN IDs, scales, and trip limits for sc2-powertrain

// CAN message definitions
#define BPS_TEMPERATURE_CAN_ID         0x108
#define BPS_TEMPERATURE_CAN_DLC        4
#define BPS_ELECTRICAL_CAN_ID          0x109
#define BPS_ELECTRICAL_CAN_DLC         6
// Shared BPS/estop fault status - high priority, not in BMS 0x10x band
#define POWERTRAIN_FAULT_CAN_ID        0x001
#define POWERTRAIN_STATUS_CAN_DLC      1
#define POWERTRAIN_FAULT_MASK          0x01

#define FAULT_CLEAR_CAN_ID             0x7EB
#define FAULT_CLEAR_CAN_DLC            8

// Each B1/B2 pair is big-endian uint16 - Cell voltage scale is 0.0001 V/bit
#ifndef BPS_CELL_VOLTAGE_SCALE_V
#define BPS_CELL_VOLTAGE_SCALE_V       0.0001f
#endif

// Pack current: midscale 0x8000 is 0 A, each count is 0.1 A
#define BPS_PACK_CURRENT_ZERO_RAW      0x8000U
#define BPS_PACK_CURRENT_SCALE_A       0.1f

// Inclusive operating limits
#define BPS_CELL_VOLTAGE_MIN_V         2.52f
#define BPS_CELL_VOLTAGE_MAX_V         4.18f
#define BPS_PACK_CURRENT_MIN_A          0U
#define BPS_PACK_CURRENT_MAX_A         35U
#define BPS_TEMPERATURE_MIN_C           4U
#define BPS_TEMPERATURE_MAX_C          56U

// Do not trip on BPS telem until this time after the first BPS frame
#define BPS_FAULT_STARTUP_GRACE_MS     2000UL

#endif
