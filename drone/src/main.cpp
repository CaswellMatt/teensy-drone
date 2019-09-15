#include <Arduino.h>

#include "Quaternion.h"
#include "OrientationFilter.h"
#include "ReceiverManager.h"
#include "Arduino.h"
#include "MotorControlManager.h"
 
OrientationFilter orientationFilter;

long timer;
float32_t loopTime;

void setup() {
  #define DEBUG
  #ifdef DEBUG
    while(!Serial);
  #endif

  ReceiverManager::setupReceivers();
  ReceiverManager::setupAligners();
  MotorControlManager::setup();
  timer = micros();
}

#define LOOPTIME_US 500
#define LOOPTIME_S LOOPTIME_US / 1000000

float32_t throttleMapStart  = 125;
float32_t throttleMapEnd    = 225;
float32_t throttleMapMiddle = (throttleMapStart + throttleMapEnd) / 2;

float32_t controlPulseStart  = -12.5;
float32_t controlPulseEnd    = 12.5;
float32_t controlPulseMiddle = (controlPulseStart + controlPulseEnd) / 2;

void loop() {
  while(micros() - timer < LOOPTIME_US);
  timer = micros();

  orientationFilter.update(LOOPTIME_S);

  float32_t rollPulse =
    ReceiverManager::rollAligned->getPulseLength(controlPulseStart, controlPulseMiddle, controlPulseEnd);
  float32_t pitchPulse =
    ReceiverManager::pitchAligned->getPulseLength(controlPulseStart, controlPulseMiddle, controlPulseEnd);
  float32_t yawPulse =
    ReceiverManager::yawAligned->getPulseLength(controlPulseStart, controlPulseMiddle, controlPulseEnd);

  float32_t throttlePulse = 
    ReceiverManager::throttleAligned->getPulseLength(throttleMapStart, throttleMapMiddle, throttleMapEnd);

  float32_t frontLeftPulse  = throttlePulse + rollPulse - pitchPulse + yawPulse;
  float32_t frontRightPulse = throttlePulse - rollPulse - pitchPulse - yawPulse;
  float32_t backLeftPulse   = throttlePulse + rollPulse + pitchPulse + yawPulse;
  float32_t backRightPulse  = throttlePulse - rollPulse + pitchPulse - yawPulse;

  auto checkMinMaxOfPulse = [](float32_t& pulse) {
    if (pulse < throttleMapStart) {
      pulse = throttleMapStart;
    } else if (pulse > throttleMapEnd) {
      pulse = throttleMapEnd;
    }
  };

  checkMinMaxOfPulse(frontLeftPulse);
  checkMinMaxOfPulse(frontRightPulse);
  checkMinMaxOfPulse(backLeftPulse);
  checkMinMaxOfPulse(backRightPulse);

  Serial.print(frontLeftPulse);Serial.print(" ");
  Serial.print(frontRightPulse);Serial.print(" ");
  Serial.print(backLeftPulse);Serial.print(" ");
  Serial.print(backRightPulse);Serial.println(" ");

  MotorControlManager::frontLeft.trigger(frontLeftPulse);
  MotorControlManager::frontRight.trigger(frontRightPulse);
  MotorControlManager::backLeft.trigger(backLeftPulse);
  MotorControlManager::backRight.trigger(backRightPulse);

}
