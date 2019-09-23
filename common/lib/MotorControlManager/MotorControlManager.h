#ifndef MOTOR_CONTROL_MANAGER
#define MOTOR_CONTROL_MANAGER

#include "MotorSignalController.h"

#define FRONT_LEFT_MOTOR_PIN 20
#define FRONT_RIGHT_MOTOR_PIN 21
#define BACK_LEFT_MOTOR_PIN 22
#define BACK_RIGHT_MOTOR_PIN 23

namespace MotorControlManager {
  
  extern MotorSignalController frontLeft;
  extern MotorSignalController frontRight;
  extern MotorSignalController backLeft;
  extern MotorSignalController backRight;

  extern void turnFrontLeftLow();
  extern void turnFrontRightLow();
  extern void turnBackLeftLow();
  extern void turnBackRightLow();

  extern void turnFrontLeftHigh();
  extern void turnFrontRightHigh();
  extern void turnBackLeftHigh();
  extern void turnBackRightHigh();

  extern void registerChannelHigh(MotorSignalController& controller, void (*callback)(), int &nextChannelIdForRegistering);
  extern void registerChannelLow(MotorSignalController& controller, void (*callback)(), int &nextChannelIdForRegistering);
  
  extern void setup();
};

#endif 