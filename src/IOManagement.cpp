#include "IOManagement.h"

volatile Digital_Data digital_data;

volatile float i_12v = 0;
volatile float v_12v = 0;
volatile float supp_i = 0;
volatile float batt_i = 0;
volatile float supp_v = 0;

// Ticker to poll input readings at fixed rate
STM32TimerInterrupt IOTimer(TIM2);

void initIO() {
    // Initalize digital pins
    pinMode(ESTOP_MCU, INPUT);
    pinMode(BATT_POS_CONT_MCU, INPUT);
    pinMode(PPC1_SUPP_INVALID, INPUT);
    pinMode(PPC1_DCDC_INVALID, INPUT);
    pinMode(MCU_BATT_EN, OUTPUT);
    pinMode(MPPT_CONT_MCU, INPUT);
    pinMode(MC_CONT_MCU, INPUT);

    // Initialize analog pins
    initADC(ADC1);

    if (IOTimer.attachInterruptInterval(IO_UPDATE_PERIOD, readIO)) {
        printf("IO timer started\n");
    } else {
        printf("Failed to start IO timer\n");
    }
}

void readIO() {
    digital_data.estop_mcu = digitalRead(ESTOP_MCU);
    digital_data.batt_pos_cont = digitalRead(BATT_POS_CONT_MCU);
    digital_data.ppc1_supp_invalid = digitalRead(PPC1_SUPP_INVALID);
    digital_data.ppc1_dcdc_invalid = digitalRead(PPC1_DCDC_INVALID);
    digital_data.mppt_cont_mcu = digitalRead(MPPT_CONT_MCU);
    digital_data.mc_cont_mcu = digitalRead(MC_CONT_MCU);

    i_12v = readADC(ADC_CHANNEL_12);        // PA7
    v_12v = readADC(ADC_CHANNEL_11);        // PA6
    supp_i = readADC(ADC_CHANNEL_9);        // PA4
    batt_i = readADC(ADC_CHANNEL_6);        // PA1
    supp_v = readADC(ADC_CHANNEL_5);        // PA0
}

void set_mcu_batt_en(bool batt_en) {
    digitalWrite(MCU_BATT_EN, batt_en);
    digital_data.mcu_batt_en = batt_en;
}