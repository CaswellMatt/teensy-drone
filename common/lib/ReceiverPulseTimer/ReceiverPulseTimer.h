#ifndef RECEIVER_PULSE_TIMER
#define RECEIVER_PULSE_TIMER

#include <Arduino.h>

class ReceiverPulseTimer {
public:
  ReceiverPulseTimer(uint8_t inputPin) : pin(inputPin) {}

  void onPulseStateChange() {
    if (digitalReadFast(pin)) {
      timer = micros();
    } else {
      pulseLength = (int)(micros() - timer);
    }
  }

  uint8_t getInputPin() {
    return pin;
  }

  int getPulseLength() {
    return pulseLength;
  }

  void setupInterruptForThisChannel(void (*interruptFunction)(void)) {
    attachInterrupt(pin, interruptFunction, CHANGE);
  }

  long timer;
  int pulseLength;
  uint8_t pin;

};

#endif