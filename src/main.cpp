// sc2-powertrain main: battery contactor enable and BPS fault path
// Reads IO and BPS CAN - Latches faults in EEPROM - Publishes status on 0x001
#include <Arduino.h>
#include "canPowertrain.h"
#include "IOManagement.h"

CANPowertrain canPowertrain(CAN1, DEF);

void setup() {
  Serial.begin(115200);
  initIO();
  // Load latched BPS fault from EEPROM before the first status TX
  canPowertrain.initializePersistentFault();
}

void loop() {
  canPowertrain.sendPowertrainData();
  canPowertrain.runQueue(50);
}
