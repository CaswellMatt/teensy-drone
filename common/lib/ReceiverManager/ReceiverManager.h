#ifndef RECEIVER_MANAGER
#define RECEIVER_MANAGER
#include <Arduino.h>
#include "ReceiverPulseTimer.h"
#include "MemoryLocations.h"
#include "Math.h"
#include "ReceiverAligner.h"

namespace ReceiverManager
{
  #define ROLL_PIN     4
  #define PITCH_PIN    5
  #define THROTTLE_PIN 6
  #define YAW_PIN      7
  
  ReceiverPulseTimer rollInput(ROLL_PIN);
  ReceiverPulseTimer pitchInput(PITCH_PIN);
  ReceiverPulseTimer throttleInput(THROTTLE_PIN);
  ReceiverPulseTimer yawInput(YAW_PIN);
  
  ReceiverAligner* rollAligned;
  ReceiverAligner* pitchAligned;
  ReceiverAligner* throttleAligned;
  ReceiverAligner* yawAligned;

  void rollInputInterrupt() {
    rollInput.onPulseStateChange();
  }

  void pitchInputInterrupt() {
    pitchInput.onPulseStateChange();
  }

  void throttleInputInterrupt() {
    throttleInput.onPulseStateChange();
  }

  void yawInputInterrupt() {
    yawInput.onPulseStateChange();
  }

  void setupReceivers() {
    
    rollInput.setupInterruptForThisChannel(rollInputInterrupt);
    pitchInput.setupInterruptForThisChannel(pitchInputInterrupt);
    throttleInput.setupInterruptForThisChannel(throttleInputInterrupt);
    yawInput.setupInterruptForThisChannel(yawInputInterrupt);

  }

  bool isReceiving() {
    return rollInput.isWorking() && 
           pitchInput.isWorking() &&
           throttleInput.isWorking() &&
           yawInput.isWorking();
  }

  void updateMinAndMaxForTimer(float32_t& min, float32_t& max, float32_t& mid, int startAddress) {

    int eeAddress = startAddress;
    EEPROM.get(eeAddress, max);

    eeAddress += sizeof(float32_t);
    EEPROM.get(eeAddress, min);

    eeAddress += sizeof(float32_t);
    EEPROM.get(eeAddress, mid);

  }

  void setupAligners() {

    float32_t max;
    float32_t min;
    float32_t mid;

    updateMinAndMaxForTimer(min, max, mid, ROLL_START);
    rollAligned = new ReceiverAligner(&rollInput, min, max, mid);

    updateMinAndMaxForTimer(min, max, mid, PITCH_START);
    pitchAligned = new ReceiverAligner(&pitchInput, min, max, mid);

    updateMinAndMaxForTimer(min, max, mid, THROTTLE_START);
    throttleAligned  = new ReceiverAligner(&throttleInput, min, max, mid);

    updateMinAndMaxForTimer(min, max, mid, YAW_START);
    yawAligned  = new ReceiverAligner(&yawInput, min, max, mid);

  }

   void destroyAligners() {
    delete rollAligned;
    delete pitchAligned;
    delete throttleAligned;
    delete yawAligned;
  }

  void printPulseLength(String channel, int pulseLength) {
    Serial.print(channel);Serial.print(" ");
    Serial.print(pulseLength);Serial.print(" ");
  }

  void printAllPulseLengths() {
    printPulseLength("roll",     rollInput.getPulseLength());
    printPulseLength("pitch",    pitchInput.getPulseLength());
    printPulseLength("throttle", throttleInput.getPulseLength());
    printPulseLength("yaw",      yawInput.getPulseLength());
  }

  void printAlignedPulses() 
  {
    printPulseLength("roll",     rollAligned->getPulseLength());
    printPulseLength("pitch",    pitchAligned->getPulseLength());
    printPulseLength("throttle", throttleAligned->getPulseLength());
    printPulseLength("yaw",      yawAligned->getPulseLength());
  }

}

#endif