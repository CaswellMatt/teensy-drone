#include <Arduino.h>

#include "Constants.h"
#include "MotorControlManager.h"
#include "MARG.h"

long timer;

MARG marg;

void setup() {
  #define DEBUG
  #ifdef DEBUG
    while(!Serial);
  #endif

  MotorControlManager::setup();
  timer = micros();

}



float32_t throttleMapStart  = 125;
float32_t throttleMapEnd    = 240;
float32_t throttleMapMiddle = (throttleMapStart + throttleMapEnd) / 2;

float32_t frontLeftPulse  = throttleMapStart;
float32_t frontRightPulse = throttleMapStart;
float32_t backLeftPulse   = throttleMapStart;
float32_t backRightPulse  = throttleMapStart;


void loop() {

  while(micros() - timer < LOOPTIME_US);
  timer = micros();
  marg.read();

  MotorControlManager::frontLeft.trigger(frontLeftPulse);
  MotorControlManager::frontRight.trigger(frontRightPulse);
  MotorControlManager::backLeft.trigger(backLeftPulse);
  MotorControlManager::backRight.trigger(backRightPulse);

}
