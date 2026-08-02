#include <Arduino.h>

#include "board_config.h"
#include "can_powertrain.h"
#include "debug.h"
#include "io_management.h"

// ------------- LOCAL -------------

static CanPowertrain canPowertrain(CAN1, DEF);

// ------------- PUBLIC FUNCTIONS -------------

void setup() {
    debugInit();

    initIO();
    canPowertrain.initializePersistentFault();
}

void loop() {
    debugUpdate();

    canPowertrain.sendPowertrainData();
    canPowertrain.runQueue(DATA_SEND_PERIOD);
}
