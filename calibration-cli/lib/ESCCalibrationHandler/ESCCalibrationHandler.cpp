#include "ESCCalibrationHandler.h"
#include <Arduino.h>

#include <arm_math.h>

#include <functional>

#include "MotorControlManager.h"

#include "Constants.h"


#define CALIBRATE_ALL_ESC_INDEX             1

const String calibrateAllESCsOptionTest = "Calibrate ESC's";


ESCCalibrationHandler::ESCCalibrationHandler() {
    message = String("Calibrate ESC's");
}


void ESCCalibrationHandler::printTitle() {
  Serial.println("ESC Calibration Menu");
}


void ESCCalibrationHandler::setup() {

  auto ESCCalibrator = [this](){ calibrate(); };
  
  addOption(CALIBRATE_ALL_ESC_INDEX, ESCCalibrator, calibrateAllESCsOptionTest);

  MotorControlManager::setup();

}


void ESCCalibrationHandler::calibrate() {
  int timer = micros();

  float direction = 1;
  float pulseTime = 250;

  for (int i = 0; i < 5000; i++) {
    while (micros() - timer < LOOPTIME_US);
    timer = micros();

    MotorControlManager::frontLeft.trigger(pulseTime);
    MotorControlManager::frontRight.trigger(pulseTime);
    MotorControlManager::backLeft.trigger(pulseTime);
    MotorControlManager::backRight.trigger(pulseTime);
    
    

    Serial.println(pulseTime);
  }

  pulseTime = 125;

  for (int i = 0; i < 5000; i++) {
    while (micros() - timer < LOOPTIME_US);
    timer = micros();

    MotorControlManager::frontLeft.trigger(pulseTime);
    MotorControlManager::frontRight.trigger(pulseTime);
    MotorControlManager::backLeft.trigger(pulseTime);
    MotorControlManager::backRight.trigger(pulseTime);

    Serial.println(pulseTime);
  }

  delay(6000);
}