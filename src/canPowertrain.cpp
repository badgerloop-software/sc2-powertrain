#include "canPowertrain.h"


bool send_success;


CANPowertrain::CANPowertrain(CAN_TypeDef* canPort, CAN_PINS pins, int frequency) : CANManager(canPort, pins, frequency) {};

//volatile bool mcu_battery_enable = false;

void CANPowertrain::readHandler(CAN_message_t msg) {
    uint8_t *data = msg.buf;
    switch(msg.id-0x500){
        case 5: // digital_data
            memcpy((void *)&digital_data, msg.buf, sizeof(digital_data));
            digital_data.mcu_batt_en = !digital_data.mcu_batt_en;
            break;
        default:
            break;
    }
}

void CANPowertrain::sendPowertrainData() {
    // TODO: send messages with their respective CAN IDs
    send_success = true;
    send_success &= this->sendMessage(0x500, (void*)&i_12v, sizeof(float));
    send_success &= this->sendMessage(0x501, (void*)&v_12v, sizeof(float));
    send_success &= this->sendMessage(0x502, (void*)&supp_i, sizeof(float));
    send_success &= this->sendMessage(0x503, (void*)&batt_i, sizeof(float));
    send_success &= this->sendMessage(0x504, (void*)&supp_v, sizeof(float));
    send_success &= this->sendMessage(0x505, (void*)&digital_data, sizeof(digital_data));
    printf("Powertrain Data Send Success: %d\n", send_success);
}