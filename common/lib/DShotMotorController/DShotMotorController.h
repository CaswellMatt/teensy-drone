#ifndef DSHOT_MOTOR_CONTROLLER_H
#define DSHOT_MOTOR_CONTROLLER_H

#include <Arduino.h>

constexpr uint8_t FRONT_LEFT_MOTOR_PIN  = 20;
constexpr uint8_t FRONT_RIGHT_MOTOR_PIN = 21;
constexpr uint8_t BACK_LEFT_MOTOR_PIN   = 22;
constexpr uint8_t BACK_RIGHT_MOTOR_PIN  = 23;

class DShotMotorController {
private:
  uint16_t prepareDShotPacket(const uint16_t value, uint16_t telemBit = 0);
  void sendPacket(
    uint16_t frontLeftPacket, 
    uint16_t frontRightPacket, 
    uint16_t backLeftPacket, 
    uint16_t backRightPacket);

public:
  void setup();
  void setSpeed(
    const uint16_t frontLeftSpeed, 
    const uint16_t frontRightSpeed, 
    const uint16_t backLeftSpeed, 
    const uint16_t backRightSpeed);
};

#endif