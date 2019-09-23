#ifndef MOTOR_NOISE_TEST_HANDLER
#define MOTOR_NOISE_TEST_HANDLER

#include "MenuOptionHandler.h"

#include "Constants.h"
#include "MARG.h"

#include "MotorControlManager.h"
#include "MotorSignalController.h"



class MotorNoiseTest : public MenuOptionHandler {
private:
    long timer;
    MARG marg;

public:

  MotorNoiseTest();

  virtual void setup() override;
  virtual void printTitle() override;

  float32_t frontLeftPulse  = throttleMapStart;
  float32_t frontRightPulse = throttleMapStart;
  float32_t backLeftPulse   = throttleMapStart;
  float32_t backRightPulse  = throttleMapStart;

  void runMotorTest(MotorSignalController& signalController);

};

#endif 