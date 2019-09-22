#include <Arduino.h>
#include "MainMenuHandler.h"

#ifndef MOTOR_NOISE_TEST_HANDLER
#define MOTOR_NOISE_TEST_HANDLER

#include "MenuOptionHandler.h"

#include "Constants.h"
#include "MotorControlManager.h"
#include "MARG.h"

class MotorNoiseTest : public MenuOptionHandler {
public:

  MotorNoiseTest() {}

  virtual void setup() override {
    #define DEBUG
    #ifdef DEBUG
      while(!Serial);
    #endif

    MotorControlManager::setup();
    timer = micros();

  } 


  virtual void printTitle() override {

  }


  long timer;

  MARG marg;

  float32_t throttleMapStart  = 125;
  float32_t throttleMapEnd    = 240;
  float32_t throttleMapMiddle = (throttleMapStart + throttleMapEnd) / 2;

  float32_t frontLeftPulse  = throttleMapStart;
  float32_t frontRightPulse = throttleMapStart;
  float32_t backLeftPulse   = throttleMapStart;
  float32_t backRightPulse  = throttleMapStart;

  void start() {
    while(micros() - timer < LOOPTIME_US);
    timer = micros();
    marg.read();

    MotorControlManager::frontLeft.trigger(frontLeftPulse);
    MotorControlManager::frontRight.trigger(frontRightPulse);
    MotorControlManager::backLeft.trigger(backLeftPulse);
    MotorControlManager::backRight.trigger(backRightPulse);
  }

};

#endif 

MainMenuHandler mainMenu;

MotorNoiseTest motorNoiseTest;

void setup() {
  mainMenu.addHandlerOption(0)
}

void loop() {

}
