#ifndef MOTOR_SIGNAL_CONTROLLER
#define MOTOR_SIGNAL_CONTROLLER

#include <TeensyDelay.h>
#include <arm_math.h>

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

#endif