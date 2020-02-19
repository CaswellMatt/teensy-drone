#ifndef RECEIVER_PULSE_TIMER
#define RECEIVER_PULSE_TIMER

#include <Arduino.h>

static constexpr uint32_t CPU_FREQUENCY_Hz = 600000000;
static constexpr float32_t TICKS_TO_MICROS = (float32_t)1000000/CPU_FREQUENCY_Hz;

class ReceiverPulseTimer {
public:
  ReceiverPulseTimer(uint8_t inputPin) : 
    m_cycleTimer(0),
    m_pin(inputPin),
    m_clockTicksInPulse(0), 
    m_hasReceivedInterrupt(false) {}

  void onPulseStateChange() {
    if (!m_hasReceivedInterrupt) {
      m_hasReceivedInterrupt = true;
    } 

    if (digitalReadFast(m_pin)) {
      m_cycleTimer = ARM_DWT_CYCCNT;
    } else {
      m_clockTicksInPulse = (ARM_DWT_CYCCNT - m_cycleTimer);
    }
  }

  uint8_t getInputPin() const {
    return m_pin;
  }
  
  uint32_t getPulseLengthMicros() const {
    return ((float32_t)(m_clockTicksInPulse) * TICKS_TO_MICROS);
  }

  bool isWorking() {
    return m_hasReceivedInterrupt;
  }

  void setupInterruptForThisChannel(void (*interruptFunction)(void)) {
    attachInterrupt(m_pin, interruptFunction, CHANGE);
  }

private:
  uint32_t m_cycleTimer;
  uint8_t m_pin;
  uint32_t m_clockTicksInPulse;
  bool m_hasReceivedInterrupt;

};

#endif