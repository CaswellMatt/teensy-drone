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

  while(!ReceiverManager::isReceiving()) {Serial.println(0);};
  Serial.println("Receiver Active");

  delay(3);

  MotorControlManager::setup();
  timer = micros();
}

const long LOOPTIME_US = 1000;
const float32_t LOOPTIME_S = (float32_t)LOOPTIME_US / 1000000;

float32_t throttleMapStart  = 125;
float32_t throttleMapEnd    = 240;
float32_t throttleMapMiddle = (throttleMapStart + throttleMapEnd) / 2;

float32_t controlPulseStart  = -12.5;
float32_t controlPulseEnd    = 12.5;
float32_t controlPulseMiddle = (controlPulseStart + controlPulseEnd) / 2;

bool motorsAreActive() {
  static bool isActive = false;

  float32_t lowerThrottleThreshold = throttleMapStart +  0.05 * (throttleMapEnd - throttleMapStart);

  bool throttleLowerThanThreshold = 
    ReceiverManager::throttleAligned->getPulseLength(throttleMapStart, 
                                                     throttleMapMiddle, 
                                                     throttleMapEnd) < lowerThrottleThreshold;

  if (throttleLowerThanThreshold) {
    
    float32_t lowerYawThreshold = controlPulseStart +  0.04 * (controlPulseEnd - controlPulseStart);

    float32_t upperYawThreshold = controlPulseStart +  0.96 * (controlPulseEnd - controlPulseStart);

    float32_t currentYawPulse = ReceiverManager::yawAligned->getPulseLength(controlPulseStart, 
                                                                            controlPulseMiddle, 
                                                                            controlPulseEnd);

    bool yawLowerThanThreshold = currentYawPulse <= lowerYawThreshold;
    bool yawAboveThreshold = currentYawPulse >= upperYawThreshold;

    if (yawLowerThanThreshold) {
      isActive = true;
    } else if (yawAboveThreshold) {
      isActive = false;
    }
                                                  
  }

  return isActive;
}


float32_t frontLeftPulse  = throttleMapStart;
float32_t frontRightPulse = throttleMapStart;
float32_t backLeftPulse   = throttleMapStart;
float32_t backRightPulse  = throttleMapStart;


void loop() {

  orientationFilter.update(LOOPTIME_S);

  if (motorsAreActive()) {
    
    float32_t rollPulse =
      ReceiverManager::rollAligned->getPulseLength(controlPulseStart, controlPulseMiddle, controlPulseEnd);
    float32_t pitchPulse =
      ReceiverManager::pitchAligned->getPulseLength(controlPulseStart, controlPulseMiddle, controlPulseEnd);
    float32_t yawPulse =
      ReceiverManager::yawAligned->getPulseLength(controlPulseStart, controlPulseMiddle, controlPulseEnd);

    float32_t throttlePulse = 
      ReceiverManager::throttleAligned->getPulseLength(throttleMapStart, throttleMapMiddle, throttleMapEnd);

    frontLeftPulse  = throttlePulse + rollPulse - pitchPulse + yawPulse;
    frontRightPulse = throttlePulse - rollPulse - pitchPulse - yawPulse;
    backLeftPulse   = throttlePulse + rollPulse + pitchPulse + yawPulse;
    backRightPulse  = throttlePulse - rollPulse + pitchPulse - yawPulse;

    auto checkMinMaxOfPulse = [](float32_t& pulse) {
      float32_t min = throttleMapStart;
      if (pulse < min) {
        pulse = min;
      } else if (pulse > throttleMapEnd) {
        pulse = throttleMapEnd;
      }
    };

    checkMinMaxOfPulse(frontLeftPulse);
    checkMinMaxOfPulse(frontRightPulse);
    checkMinMaxOfPulse(backLeftPulse);
    checkMinMaxOfPulse(backRightPulse);

    // Serial.print(frontLeftPulse);Serial.print(" ");
    // Serial.print(frontRightPulse);Serial.print(" ");
    // Serial.print(backLeftPulse);Serial.print(" ");
    // Serial.print(backRightPulse);Serial.print(" ");

  } else {
    frontLeftPulse  = throttleMapStart;
    frontRightPulse = throttleMapStart;
    backLeftPulse   = throttleMapStart;
    backRightPulse  = throttleMapStart;
  }

  while(micros() - timer < LOOPTIME_US);
  timer = micros();

  MotorControlManager::frontLeft.trigger(frontLeftPulse);
  MotorControlManager::frontRight.trigger(frontRightPulse);
  MotorControlManager::backLeft.trigger(backLeftPulse);
  MotorControlManager::backRight.trigger(backRightPulse);

}
