#include <Arduino.h>
#include "IOManagement.h"

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(100);
  
  Serial.println("Starting setup...");
  
  pinMode(MCU_BATT_EN, OUTPUT);
  digitalWrite(MCU_BATT_EN, HIGH);
  
  Serial.println("Setup complete. MCU_BATT_EN is now HIGH and should stay HIGH");
}

void loop() {
  // put your main code here, to run repeatedly:
  // Keep the pin HIGH at all times
  digitalWrite(MCU_BATT_EN, HIGH);
  delay(100);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}