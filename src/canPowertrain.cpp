#include "canPowertrain.h"

CANPowertrain::CANPowertrain(CAN_TypeDef* canPort, CAN_PINS pins, int frequency) : CANManager(canPort, pins, frequency) {};

volatile bool mcu_battery_enable = false;

void CANPowertrain::readHandler(CAN_message_t msg) {
    uint8_t *data = msg.buf;
    switch(msg.id){
        case POWERTRAIN_BOOLS:
            mcu_battery_enable = (data[0] >> 5) & 1;
            break;
        default:
            break;
    }
}

void CANPowertrain::sendPowertrainData() {
    // TODO: send messages with their respective CAN IDs
    this->sendMessage(I_OUT_12V_TELEM_ID, (void*)&i_out_12v, sizeof(float));
    this->sendMessage(V_OUT_12V_TELEM_ID, (void*)&v_out_12v, sizeof(float));
    this->sendMessage(SUPP_I_TELEM_ID, (void*)&supp_i, sizeof(float));
    this->sendMessage(BATT_I_TELEM_ID, (void*)&batt_i, sizeof(float));
    this->sendMessage(SUPP_V_TELEM_ID, (void*)&supp_v, sizeof(float));

    u_int8_t boolByte = 0;
    boolByte |= (batt_neg_cont << 0);
    boolByte |= (estop_mcu << 1);
    boolByte |= (batt_pos_cont << 2);
    boolByte |= (ppc1_supp_invalid << 3);
    boolByte |= (ppc1_dcdc_invalid << 4);
    boolByte |= (mppt_cont_mcu << 6);
    boolByte |= (mc_cont_mcu << 7);

    this->sendMessage(POWERTRAIN_BOOLS, (void*)&boolByte, sizeof(u_int8_t));

}