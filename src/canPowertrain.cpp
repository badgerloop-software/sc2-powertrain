#include "canPowertrain.h"

CANPowertrain::CANPowertrain(CAN_TypeDef* canPort, CAN_PINS pins, int frequency) : CANManager(canPort, pins, frequency) {};

void CANPowertrain::readHandler(CAN_message_t msg) {

}

void CANPowertrain::sendPowertrainData() {
    // TODO: send messages with their respective CAN IDs
}