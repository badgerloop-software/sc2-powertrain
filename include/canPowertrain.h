#ifndef __CAN_POWERTRAIN_H__
#define __CAN_POWERTRAIN_H__

#include "canmanager.h"
#include "IOManagement.h"

#define I_OUT_12V_TELEM_ID 0x500
#define V_OUT_12V_TELEM_ID 0x501
#define SUPP_I_TELEM_ID 0x502
#define BATT_I_TELEM_ID 0x503
#define SUPP_V_TELEM_ID 0x504
#define POWERTRAIN_BOOLS 0x505

class CANPowertrain : public CANManager {
    public:
        CANPowertrain(CAN_TypeDef* canPort, CAN_PINS pins, int frequency = DEFAULT_CAN_FREQ);
        void readHandler(CAN_message_t msg);
        void sendPowertrainData();
};

#endif