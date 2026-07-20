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
    pinMode(BATT_NEG_CONT_MCU, INPUT);
    pinMode(ESTOP_MCU, INPUT);
    pinMode(BATT_POS_CONT_MCU, INPUT);
    pinMode(PPC1_SUPP_INVALID, INPUT);
    pinMode(PPC1_DCDC_INVALID, INPUT);
    
    // Initialize MCU_BATT_EN as push-pull output with explicit mode
    pinMode(MCU_BATT_EN, OUTPUT);
    set_mcu_batt_en(true);
    
    pinMode(MPPT_CONT_MCU, INPUT);
    pinMode(MC_CONT_MCU, INPUT);

    // Initialize analog pins AFTER digital outputs are set
    // This prevents ADC initialization from affecting GPIO state
    initADC(ADC1);

    if (IOTimer.attachInterruptInterval(IO_UPDATE_PERIOD, readIO)) {
        printf("IO timer started\n");
        // Ensure the timer interrupt has lower priority than other operations
        // to prevent it from interrupting critical GPIO writes
        NVIC_SetPriority(TIM2_IRQn, 3);
    } else {
        printf("Failed to start IO timer\n");
    }
}

void readIO() {
    // Debug: log when ADC reads happen
    static unsigned long lastReadTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastReadTime > 50) {  // Only print occasionally to avoid spam
        Serial.print("[ADC_READ @ ");
        Serial.print(currentTime);
        Serial.println("ms]");
        lastReadTime = currentTime;
    }
    
    digital_data.batt_neg_cont = digitalRead(BATT_NEG_CONT_MCU);
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