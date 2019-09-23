#include "MotorControlManager.h"
#include "MotorSignalController.h"

namespace MotorControlManager {
  MotorSignalController frontLeft(FRONT_LEFT_MOTOR_PIN);
  MotorSignalController frontRight(FRONT_RIGHT_MOTOR_PIN);
  MotorSignalController backLeft(BACK_LEFT_MOTOR_PIN);
  MotorSignalController backRight(BACK_RIGHT_MOTOR_PIN);
};

void MotorControlManager::turnFrontLeftLow() { frontLeft.low(); }
void MotorControlManager::turnFrontRightLow() { frontRight.low(); }
void MotorControlManager::turnBackLeftLow() { backLeft.low(); }
void MotorControlManager::turnBackRightLow() { backRight.low(); }

void MotorControlManager::turnFrontLeftHigh() { frontLeft.high(); }
void MotorControlManager::turnFrontRightHigh() { frontRight.high(); }
void MotorControlManager::turnBackLeftHigh() { backLeft.high(); }
void MotorControlManager::turnBackRightHigh() { backRight.high(); }

void MotorControlManager::registerChannelHigh(MotorSignalController& controller, void (*callback)(), int &nextChannelIdForRegistering) {
  controller.channelIdHigh = nextChannelIdForRegistering;
  TeensyDelay::addDelayChannel(callback,  nextChannelIdForRegistering);
  const int delayIncrement = 100;
  //attempt to ensure signals do not overlap
  controller.delay = nextChannelIdForRegistering * delayIncrement;
  nextChannelIdForRegistering++;
}

void MotorControlManager::registerChannelLow(MotorSignalController& controller, void (*callback)(), int &nextChannelIdForRegistering) {
  controller.channelIdLow = nextChannelIdForRegistering;
  TeensyDelay::addDelayChannel(callback,  nextChannelIdForRegistering);
  nextChannelIdForRegistering++;
}

void MotorControlManager::setup() {
  TeensyDelay::begin();
  int nextChannelIdForRegistering = 0;

  registerChannelLow(frontRight, turnFrontRightLow, nextChannelIdForRegistering);
  registerChannelLow(frontLeft , turnFrontLeftLow , nextChannelIdForRegistering);
  registerChannelLow(backRight , turnBackRightLow , nextChannelIdForRegistering);
  registerChannelLow(backLeft  , turnBackLeftLow  , nextChannelIdForRegistering);

  registerChannelHigh(frontRight, turnFrontRightHigh, nextChannelIdForRegistering);
  registerChannelHigh(frontLeft , turnFrontLeftHigh , nextChannelIdForRegistering);
  registerChannelHigh(backRight , turnBackRightHigh , nextChannelIdForRegistering);
  registerChannelHigh(backLeft  , turnBackLeftHigh  , nextChannelIdForRegistering);
}