#include "debug.h"

#if SC2_DEBUG

#include <Arduino.h>

#include "io_management.h"

// ------------- LOCAL -------------

static int debugCounter;

// ------------- LOCAL FUNCTIONS -------------

static void debugPrint() {
#if SC2_DEBUG == 1
    printf("\033[2J\033[1;1H");
    printf("i_12v %5.2f  v_12v %5.2f  supp_i %5.2f  batt_i %5.2f  supp_v %5.2f\n", i_12v, v_12v,
           supp_i, batt_i, supp_v);
    printf("estop %i  batt_en %i\n", digital_data.estop_mcu, digital_data.mcu_batt_en);
#endif
}

// ------------- PUBLIC FUNCTIONS -------------

void debugInit() {
    Serial.begin(115200);
}

void debugUpdate() {
#if SC2_DEBUG == 1
    if (debugCounter >= (200 / DATA_SEND_PERIOD)) {
        debugPrint();
        debugCounter = 0;
    }
    debugCounter++;
#else
    debugPrint();
#endif
}

void debugError(const char* msg) {
    printf("%s\n", msg);
}

#endif  // SC2_DEBUG
