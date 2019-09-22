#include <TeensyDelay.h>

class MotorSignalController {
public:
  MotorSignalController(int outputPin) : 
    pin(outputPin) {
      pinMode(outputPin, OUTPUT);
    }

  int pin;
  int channelId;

  void on() {
    digitalWriteFast(pin, HIGH);
  }

  void off() {
    digitalWriteFast(pin, LOW);
  }

  void trigger(float32_t pulseTime) {
    on();
    TeensyDelay::trigger(pulseTime, channelId);
  }

};

#define FRONT_LEFT_MOTOR_PIN 20
#define FRONT_RIGHT_MOTOR_PIN 21
#define BACK_LEFT_MOTOR_PIN 22
#define BACK_RIGHT_MOTOR_PIN 23

namespace MotorControlManager {

  int nextChannelIdForRegistering = 0;

  MotorSignalController frontLeft(FRONT_LEFT_MOTOR_PIN);
  MotorSignalController frontRight(FRONT_RIGHT_MOTOR_PIN);
  MotorSignalController backLeft(BACK_LEFT_MOTOR_PIN);
  MotorSignalController backRight(BACK_RIGHT_MOTOR_PIN);

  void turnFrontLeftOff() {
    frontLeft.off();
  }

  void turnFrontRightOff() {
    frontRight.off();
  }

  void turnBackLeftOff() {
    backLeft.off();
  }

  void turnBackRightOff() {
    backRight.off();
  }

  void registerChannel(MotorSignalController& controller, void (*callback)()) {
    controller.channelId = nextChannelIdForRegistering;
    TeensyDelay::addDelayChannel(callback,  nextChannelIdForRegistering);
    nextChannelIdForRegistering++;
  }

  void setup() {
    TeensyDelay::begin();
    registerChannel(frontRight, turnFrontRightOff);
    registerChannel(frontLeft , turnFrontLeftOff);
    registerChannel(backRight , turnBackRightOff);
    registerChannel(backLeft  , turnBackLeftOff);
  }
};