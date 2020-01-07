#ifndef MOTOR_NOISE_TEST_HANDLER
#define MOTOR_NOISE_TEST_HANDLER

#include "MenuOptionHandler.h"

#include "Constants.h"
#include "MARG.h"

#include "MotorControlManager.h"
#include "MotorSignalController.h"

#include <functional>

class MotorNoiseTest : public MenuOptionHandler {
private:
    long timer;
    MARG* m_marg;

public:

  MotorNoiseTest();
  ~MotorNoiseTest() {
    delete m_marg;
  }

  virtual void setup() override;
  virtual void printTitle() override;

  float32_t frontLeftPulse  = throttleMapStart;
  float32_t frontRightPulse = throttleMapStart;
  float32_t backLeftPulse   = throttleMapStart;
  float32_t backRightPulse  = throttleMapStart;

  void runTestOnMotors(MotorSignalController* motorArray, int motorCount, Vector (MARG::*sensor)());

};

#endif 