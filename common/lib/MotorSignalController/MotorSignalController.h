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
  int channelIdHigh;
  int channelIdLow;
  int delay;

  void high() {
    digitalWriteFast(pin, HIGH);
  }

  void low() {
    digitalWriteFast(pin, LOW);
  }

  void trigger(float32_t pulseTime) {
    TeensyDelay::trigger(delay, channelIdHigh);
    TeensyDelay::trigger(pulseTime + delay, channelIdLow);
  }

};

#endif