#include <Arduino.h>
#include <CANPowertrain.h>
#include <IOManagement.h>

// put function declarations here:
int myFunction(int, int);
CANPowertrain canPowertrain(CAN1, DEF);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  Serial.begin(115200);
  initIO();
}

void loop() {
  // put your main code here, to run repeatedly:
  readIO();
  canPowertrain.sendPowertrainData();
  canPowertrain.runQueue(50);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}