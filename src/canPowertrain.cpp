#include "canPowertrain.h"

CANPowertrain::CANPowertrain(CAN_TypeDef* canPort, CAN_PINS pins, int frequency) : CANManager(canPort, pins, frequency) {};

//volatile bool mcu_battery_enable = false;

void CANPowertrain::readHandler(CAN_message_t msg) {
    /*uint8_t *data = msg.buf;
    switch(msg.id){
        case POWERTRAIN_BOOLS:
            mcu_battery_enable = (data[0] >> 5) & 1;
            break;
        default:
            break;
    }*/
}

void CANPowertrain::sendPowertrainData() {
    // TODO: send messages with their respective CAN IDs
    this->sendMessage(0x500, (void*)&i_out_12v, sizeof(float));
    this->sendMessage(0x501, (void*)&v_out_12v, sizeof(float));
    this->sendMessage(0x502, (void*)&supp_i, sizeof(float));
    this->sendMessage(0x503, (void*)&batt_i, sizeof(float));
    this->sendMessage(0x504, (void*)&supp_v, sizeof(float));
    this->sendMessage(0x505, (void*)&digital_data, sizeof(digital_data));

}