#ifndef __CAN_POWERTRAIN_H__
#define __CAN_POWERTRAIN_H__

#include "canmanager.h"
#include "IOManagement.h"
#include "powertrainConfig.h"

struct BpsTelemetry {
    float highest_cell_voltage;
    float lowest_cell_voltage;
    float pack_current;
    uint16_t lowest_temperature;
    uint16_t highest_temperature;
};

class CANPowertrain : public CANManager {
    private:
        BpsTelemetry bps_telemetry;
        bool bps_fault;
        bool received_bps_temperature;
        bool received_bps_electrical;
        bool bps_startup_grace_started;
        unsigned long first_bps_message_time;

        uint16_t decodeUint16(uint8_t byte_1, uint8_t byte_2) const;
        float decodeCellVoltage(uint8_t byte_1, uint8_t byte_2) const;
        bool isBpsTelemetryOutOfRange() const;
        bool shouldMonitorBpsFaults();
        void updateBpsFault();

    public:
        CANPowertrain(CAN_TypeDef* canPort, CAN_PINS pins, int frequency = DEFAULT_CAN_FREQ);
        void readHandler(CAN_message_t msg);
        void sendPowertrainData();
        const BpsTelemetry& getBpsTelemetry() const;
        bool hasBpsFault() const;
};

#endif