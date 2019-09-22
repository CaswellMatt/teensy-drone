#ifndef MOTOR_NOISE_TEST_HANDLER
#define MOTOR_NOISE_TEST_HANDLER

#include "MenuOptionHandler.h"

#include "Constants.h"
#include "MARG.h"

#include "MotorControlManager.h"

#include <map>

class MotorNoiseTest : public MenuOptionHandler {
private:
    long timer;
    MARG marg;

    #include "MotorControlManager.h"
    std::map<int, MotorSignalController*> optionIndexToMotorSignalController;

public:

  MotorNoiseTest();

  virtual void setup() override;
  virtual void printTitle() override;

  float32_t frontLeftPulse  = throttleMapStart;
  float32_t frontRightPulse = throttleMapStart;
  float32_t backLeftPulse   = throttleMapStart;
  float32_t backRightPulse  = throttleMapStart;

  void runBackLeftMotorTest();

};

#endif 