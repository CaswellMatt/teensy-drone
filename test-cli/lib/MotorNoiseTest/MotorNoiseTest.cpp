#include "MotorNoiseTest.h"

#define FRONT_LEFT_TEST_INDEX 1
#define FRONT_RIGHT_TEST_INDEX 2
#define BACK_LEFT_TEST_INDEX 3
#define BACK_RIGHT_TEST_INDEX 4

MotorNoiseTest::MotorNoiseTest() {
  message = String("Test motor noise");
}

const String backLeftTestString = "Test back left motor noise";

void MotorNoiseTest::setup() {

  optionIndexToMotorSignalController[FRONT_LEFT_TEST_INDEX]  = &MotorControlManager::frontLeft;
  optionIndexToMotorSignalController[FRONT_RIGHT_TEST_INDEX] = &MotorControlManager::frontRight;
  optionIndexToMotorSignalController[BACK_LEFT_TEST_INDEX]   = &MotorControlManager::backLeft;
  optionIndexToMotorSignalController[BACK_RIGHT_TEST_INDEX]  = &MotorControlManager::backRight;

  auto backLeftMotorFunctionCall = [this]() { runBackLeftMotorTest(); };
  addOption(BACK_LEFT_TEST_INDEX, backLeftMotorFunctionCall, backLeftTestString);

  timer = micros();

} 

void MotorNoiseTest::printTitle() {
  Serial.println("Motor Noise Test");
}

void MotorNoiseTest::runBackLeftMotorTest() {

  while(micros() - timer < LOOPTIME_US);
  timer = micros();
  marg.read();

  MotorControlManager::frontLeft.trigger(frontLeftPulse);
  MotorControlManager::frontRight.trigger(frontRightPulse);
  MotorControlManager::backLeft.trigger(backLeftPulse);
  MotorControlManager::backRight.trigger(backRightPulse);
}

