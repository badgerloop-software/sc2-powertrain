#include "canPowertrain.h"

CANPowertrain::CANPowertrain(CAN_TypeDef* canPort, CAN_PINS pins, int frequency)
    : CANManager(canPort, pins, frequency),
      bps_telemetry{},
      bps_fault(false),
      battery_disable_latched(false),
      received_bps_temperature(false),
      received_bps_electrical(false) {};

void CANPowertrain::readHandler(CAN_message_t msg) {
    if (msg.id == BPS_TEMPERATURE_CAN_ID &&
        msg.len == BPS_TEMPERATURE_CAN_DLC) {
        bps_telemetry.lowest_temperature =
            decodeUint16(msg.buf[0], msg.buf[1]);
        bps_telemetry.highest_temperature =
            decodeUint16(msg.buf[2], msg.buf[3]);
        bps_telemetry.charge_relay_on =
            (msg.buf[4] & BPS_CHARGE_RELAY_ON_MASK) != 0;
        bps_telemetry.discharge_relay_on =
            (msg.buf[4] & BPS_DISCHARGE_RELAY_ON_MASK) != 0;
        received_bps_temperature = true;
        updateBpsFault();
    } else if (msg.id == BPS_ELECTRICAL_CAN_ID &&
               msg.len == BPS_ELECTRICAL_CAN_DLC) {
        bps_telemetry.highest_cell_voltage =
            decodeCellVoltage(msg.buf[0], msg.buf[1]);
        bps_telemetry.lowest_cell_voltage =
            decodeCellVoltage(msg.buf[2], msg.buf[3]);
        bps_telemetry.pack_current =
            decodeUint16(msg.buf[4], msg.buf[5]);
        received_bps_electrical = true;
        updateBpsFault();
    }
}

uint16_t CANPowertrain::decodeUint16(uint8_t byte_1, uint8_t byte_2) const {
    return (static_cast<uint16_t>(byte_1) << 8) |
           static_cast<uint16_t>(byte_2);
}

float CANPowertrain::decodeCellVoltage(uint8_t byte_1, uint8_t byte_2) const {
    return static_cast<float>(decodeUint16(byte_1, byte_2)) *
           BPS_CELL_VOLTAGE_SCALE_V;
}

void CANPowertrain::updateBpsFault() {
    // Once asserted, a BPS fault cannot clear in software. The BMS conditions
    // must first become healthy and then the whole system must be restarted.
    bps_fault = bps_fault || isBpsTelemetryOutOfRange();

#if BPS_LATCH_DISABLE_BATT_EN
    if (bps_fault && !battery_disable_latched) {
        set_mcu_batt_en(false);
        battery_disable_latched = true;
    }
#endif
}

bool CANPowertrain::isBpsTelemetryOutOfRange() const {
    bool voltage_fault = false;
    bool current_fault = false;
    if (received_bps_electrical) {
        voltage_fault =
            bps_telemetry.highest_cell_voltage < BPS_CELL_VOLTAGE_MIN_V ||
            bps_telemetry.highest_cell_voltage > BPS_CELL_VOLTAGE_MAX_V ||
            bps_telemetry.lowest_cell_voltage < BPS_CELL_VOLTAGE_MIN_V ||
            bps_telemetry.lowest_cell_voltage > BPS_CELL_VOLTAGE_MAX_V;
        current_fault =
            bps_telemetry.pack_current > BPS_PACK_CURRENT_MAX_A;
    }

    // Current and temperature arrive as unsigned values, so their lower bound
    // is inherently zero.
    bool temperature_fault = false;
    bool relay_fault = false;
    if (received_bps_temperature) {
        temperature_fault =
            bps_telemetry.lowest_temperature > BPS_TEMPERATURE_MAX_C ||
            bps_telemetry.highest_temperature > BPS_TEMPERATURE_MAX_C;
        relay_fault =
            !bps_telemetry.charge_relay_on ||
            !bps_telemetry.discharge_relay_on;
    }

    return voltage_fault || current_fault || temperature_fault || relay_fault;
}

void CANPowertrain::sendPowertrainData() {
    // TODO: send messages with their respective CAN IDs
    this->sendMessage(0x500, (void*)&i_12v, sizeof(float));
    this->sendMessage(0x501, (void*)&v_12v, sizeof(float));
    this->sendMessage(0x502, (void*)&supp_i, sizeof(float));
    this->sendMessage(0x503, (void*)&batt_i, sizeof(float));
    this->sendMessage(0x504, (void*)&supp_v, sizeof(float));
    uint8_t status = (digital_data.estop_mcu || bps_fault) ? 0x00 : 0x01;
    this->sendMessage(POWERTRAIN_FAULT_CAN_ID, (void*)&status, sizeof(status));
}

const BpsTelemetry& CANPowertrain::getBpsTelemetry() const {
    return bps_telemetry;
}

bool CANPowertrain::hasBpsFault() const {
    return bps_fault;
}