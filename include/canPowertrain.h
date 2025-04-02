#ifndef __CAN_POWERTRAIN_H__
#define __CAN_POWERTRAIN_H__

#include "canmanager.h"
#include "IOManagement.h"

class CANPowertrain : public CANManager {
    public:
        CANPowertrain(CAN_TypeDef* canPort, CAN_PINS pins, int frequency = DEFAULT_CAN_FREQ);
        void readHandler(CAN_message_t msg);
        void sendPowertrainData();
};

#endif