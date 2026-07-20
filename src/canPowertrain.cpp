#include "canPowertrain.h"
#include <EEPROM.h>

namespace {
constexpr int FAULT_EEPROM_MAGIC_ADDRESS = 0;
constexpr int FAULT_EEPROM_STATE_ADDRESS = 1;
constexpr uint8_t FAULT_EEPROM_MAGIC = 0xB5;
constexpr uint8_t FAULT_STATE_CLEAR = 0x00;
constexpr uint8_t FAULT_STATE_LATCHED = 0xA5;
constexpr uint8_t FAULT_STATE_CLEAR_ON_RESTART = 0x5A;

constexpr uint8_t FAULT_CLEAR_PREFIX[FAULT_CLEAR_PREFIX_LENGTH] = {
    0x03, 0x7F, 0x20, 0x22
};
}

CANPowertrain::CANPowertrain(CAN_TypeDef* canPort, CAN_PINS pins, int frequency)
    : CANManager(canPort, pins, frequency),
      bps_telemetry{},
      bps_fault(false),
      powertrain_fault_latched(false),
      received_bps_temperature(false),
      received_bps_electrical(false),
      bps_startup_grace_started(false),
      persistent_fault_initialized(false),
      clear_on_restart_armed(false),
      first_bps_message_time(0) {};

void CANPowertrain::initializePersistentFault() {
    const bool storage_initialized =
        EEPROM.read(FAULT_EEPROM_MAGIC_ADDRESS) == FAULT_EEPROM_MAGIC;

    if (!storage_initialized) {
        EEPROM.update(FAULT_EEPROM_STATE_ADDRESS, FAULT_STATE_CLEAR);
        EEPROM.update(FAULT_EEPROM_MAGIC_ADDRESS, FAULT_EEPROM_MAGIC);
        powertrain_fault_latched = false;
    } else {
        const uint8_t stored_state =
            EEPROM.read(FAULT_EEPROM_STATE_ADDRESS);
        if (stored_state == FAULT_STATE_CLEAR_ON_RESTART) {
            EEPROM.update(FAULT_EEPROM_STATE_ADDRESS, FAULT_STATE_CLEAR);
            powertrain_fault_latched = false;
        } else {
            powertrain_fault_latched =
                stored_state == FAULT_STATE_LATCHED;
        }
    }

    clear_on_restart_armed = false;
    persistent_fault_initialized = true;
}

void CANPowertrain::readHandler(CAN_message_t msg) {
    if (isFaultClearCommand(msg)) {
        handleFaultClearCommand();
        return;
    }

    if (msg.id == BPS_TEMPERATURE_CAN_ID &&
        msg.len == BPS_TEMPERATURE_CAN_DLC) {
        bps_telemetry.lowest_temperature =
            decodeUint16(msg.buf[0], msg.buf[1]);
        bps_telemetry.highest_temperature =
            decodeUint16(msg.buf[2], msg.buf[3]);
        if (shouldMonitorBpsFaults()) {
            received_bps_temperature = true;
            updateBpsFault();
        }
    } else if (msg.id == BPS_ELECTRICAL_CAN_ID &&
               msg.len == BPS_ELECTRICAL_CAN_DLC) {
        bps_telemetry.highest_cell_voltage =
            decodeCellVoltage(msg.buf[0], msg.buf[1]);
        bps_telemetry.lowest_cell_voltage =
            decodeCellVoltage(msg.buf[2], msg.buf[3]);
        const uint16_t raw_current = decodeUint16(msg.buf[4], msg.buf[5]);
        const uint16_t current_magnitude =
            raw_current >= BPS_PACK_CURRENT_ZERO_RAW
                ? raw_current - BPS_PACK_CURRENT_ZERO_RAW
                : BPS_PACK_CURRENT_ZERO_RAW - raw_current;
        bps_telemetry.pack_current =
            static_cast<float>(current_magnitude) *
            BPS_PACK_CURRENT_SCALE_A;
        if (shouldMonitorBpsFaults()) {
            received_bps_electrical = true;
            updateBpsFault();
        }
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

bool CANPowertrain::isFaultClearCommand(const CAN_message_t& msg) const {
    if (msg.id != FAULT_CLEAR_CAN_ID ||
        msg.len < FAULT_CLEAR_PREFIX_LENGTH) {
        return false;
    }

    for (uint8_t i = 0; i < FAULT_CLEAR_PREFIX_LENGTH; ++i) {
        if (msg.buf[i] != FAULT_CLEAR_PREFIX[i]) {
            return false;
        }
    }
    return true;
}

bool CANPowertrain::areLiveConditionsHealthy() const {
    const bool estop_released = digital_data.estop_mcu;
    const bool bps_data_ready =
        received_bps_temperature && received_bps_electrical;
    return estop_released &&
           bps_data_ready &&
           !isBpsTelemetryOutOfRange();
}

void CANPowertrain::handleFaultClearCommand() {
    if (!persistent_fault_initialized ||
        !powertrain_fault_latched ||
        !areLiveConditionsHealthy()) {
        return;
    }

    EEPROM.update(
        FAULT_EEPROM_STATE_ADDRESS,
        FAULT_STATE_CLEAR_ON_RESTART
    );
    clear_on_restart_armed = true;
}

void CANPowertrain::latchPowertrainFault() {
    powertrain_fault_latched = true;
    if (persistent_fault_initialized) {
        EEPROM.update(FAULT_EEPROM_STATE_ADDRESS, FAULT_STATE_LATCHED);
    }
    clear_on_restart_armed = false;
}

bool CANPowertrain::shouldMonitorBpsFaults() {
    const unsigned long current_time = millis();
    if (!bps_startup_grace_started) {
        first_bps_message_time = current_time;
        bps_startup_grace_started = true;
        return false;
    }

    return current_time - first_bps_message_time >=
           BPS_FAULT_STARTUP_GRACE_MS;
}

void CANPowertrain::updateBpsFault() {
    bps_fault = isBpsTelemetryOutOfRange();
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
    if (received_bps_temperature) {
        temperature_fault =
            bps_telemetry.lowest_temperature > BPS_TEMPERATURE_MAX_C ||
            bps_telemetry.highest_temperature > BPS_TEMPERATURE_MAX_C;
    }

    return voltage_fault || current_fault || temperature_fault;
}

void CANPowertrain::sendPowertrainData() {
    const bool estop_pressed = !digital_data.estop_mcu;
    const bool active_fault = estop_pressed || bps_fault;
    if (active_fault &&
        (!powertrain_fault_latched || clear_on_restart_armed)) {
        latchPowertrainFault();
    }
    uint8_t status =
        powertrain_fault_latched ? POWERTRAIN_FAULT_MASK : 0x00;

    // TODO: send messages with their respective CAN IDs
    this->sendMessage(0x500, (void*)&i_12v, sizeof(float));
    this->sendMessage(0x501, (void*)&v_12v, sizeof(float));
    this->sendMessage(0x502, (void*)&supp_i, sizeof(float));
    this->sendMessage(0x503, (void*)&batt_i, sizeof(float));
    this->sendMessage(0x504, (void*)&supp_v, sizeof(float));
    this->sendMessage(0x505, (void*)&status, sizeof(uint8_t));
}

const BpsTelemetry& CANPowertrain::getBpsTelemetry() const {
    return bps_telemetry;
}

bool CANPowertrain::hasBpsFault() const {
    return bps_fault;
}