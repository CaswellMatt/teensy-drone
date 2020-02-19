#ifndef MOTOR_NOISE_TEST_HANDLER
#define MOTOR_NOISE_TEST_HANDLER

#include "MenuOptionHandler.h"

#include "Constants.h"
#include "MARG.h"

// #include "MotorControlManager.h"
// #include "MotorSignalController.h"
#include "DShotMotorController.h"
#include <arm_math.h>

class MotorNoiseTest : public MenuOptionHandler {
private:
    long timer;
    MARG* m_marg;
    DShotMotorController m_motorController;

public:

  MotorNoiseTest();
  ~MotorNoiseTest() {
    delete m_marg;
  }

  virtual void setup() override;
  virtual void printTitle() override;

  float32_t m_speed  = THROTTLE_MAP_START;

  void runTestOnMotors(Vector (MARG::*sensor)());
  void runAllMotorTest();
};

#endif 