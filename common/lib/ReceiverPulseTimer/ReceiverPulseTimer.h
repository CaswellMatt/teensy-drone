#ifndef RECEIVER_PULSE_TIMER
#define RECEIVER_PULSE_TIMER

#include <Arduino.h>

class ReceiverPulseTimer {
public:
  ReceiverPulseTimer(uint8_t inputPin) : 
    pin(inputPin), 
    pulseLength(0), 
    hasReceivedInterrupt(false) {}

  void onPulseStateChange() {
    if (!hasReceivedInterrupt) {
      hasReceivedInterrupt = true;
    } 

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

  int isWorking() {
    return hasReceivedInterrupt;
  }

  void setupInterruptForThisChannel(void (*interruptFunction)(void)) {
    attachInterrupt(pin, interruptFunction, CHANGE);
  }

private:
  long timer;
  int pulseLength;
  uint8_t pin;
  bool hasReceivedInterrupt;

};

#endif