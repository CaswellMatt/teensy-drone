#ifndef MOTOR_CONTROL_MANAGER
#define MOTOR_CONTROL_MANAGER

#include "MotorSignalController.h"

#define FRONT_LEFT_MOTOR_PIN 20
#define FRONT_RIGHT_MOTOR_PIN 21
#define BACK_LEFT_MOTOR_PIN 22
#define BACK_RIGHT_MOTOR_PIN 23

namespace MotorControlManager {

  static MotorSignalController frontLeft(FRONT_LEFT_MOTOR_PIN);
  static MotorSignalController frontRight(FRONT_RIGHT_MOTOR_PIN);
  static MotorSignalController backLeft(BACK_LEFT_MOTOR_PIN);
  static MotorSignalController backRight(BACK_RIGHT_MOTOR_PIN);

  void turnFrontLeftOff();
  void turnFrontRightOff();
  void turnBackLeftOff();
  void turnBackRightOff();
  void registerChannel(MotorSignalController& controller, void (*callback)(), int &nextChannelIdForRegistering);
  void setup();
};

#endif 