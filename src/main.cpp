#include <Arduino.h>
#include "canPowertrain.h"
#include "IOManagement.h"

CANPowertrain canPowertrain(CAN1, DEF);

void setup() {
  Serial.begin(115200);
  initIO();
}

void loop() {
  canPowertrain.sendPowertrainData();
  canPowertrain.runQueue(50);
}
