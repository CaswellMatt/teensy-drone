#include "MotorControlManager.h"

using namespace MotorControlManager;

void MotorControlManager::turnFrontLeftOff() {
  frontLeft.off();
}

void MotorControlManager::turnFrontRightOff() {
  frontRight.off();
}

void MotorControlManager::turnBackLeftOff() {
  backLeft.off();
}

void MotorControlManager::turnBackRightOff() {
  backRight.off();
}

void MotorControlManager::registerChannel(MotorSignalController& controller, void (*callback)(), int &nextChannelIdForRegistering) {
  controller.channelId = nextChannelIdForRegistering;
  TeensyDelay::addDelayChannel(callback,  nextChannelIdForRegistering);
  nextChannelIdForRegistering++;
}

void MotorControlManager::setup() {
  TeensyDelay::begin();
  int nextChannelIdForRegistering = 0;

  registerChannel(frontRight, turnFrontRightOff, nextChannelIdForRegistering);
  registerChannel(frontLeft , turnFrontLeftOff , nextChannelIdForRegistering);
  registerChannel(backRight , turnBackRightOff , nextChannelIdForRegistering);
  registerChannel(backLeft  , turnBackLeftOff  , nextChannelIdForRegistering);
}