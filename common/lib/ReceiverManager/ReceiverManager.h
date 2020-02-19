#ifndef RECEIVER_MANAGER
#define RECEIVER_MANAGER
#include <Arduino.h>
#include "ReceiverPulseTimer.h"
#include "MemoryLocations.h"
#include "Math.h"
#include "ReceiverAligner.h"

namespace ReceiverManager
{

  #define ROLL_PIN 2
  #define PITCH_PIN 3
  #define THROTTLE_PIN 4
  #define YAW_PIN 5
  #define TOP_LEFT_SWITCH 6
  #define TOP_RIGHT_SWITCH 7

  
  ReceiverPulseTimer rollInput(ROLL_PIN);
  ReceiverPulseTimer pitchInput(PITCH_PIN);
  ReceiverPulseTimer throttleInput(THROTTLE_PIN);
  ReceiverPulseTimer yawInput(YAW_PIN);
  ReceiverPulseTimer topLeftSwitchInput(TOP_LEFT_SWITCH);
  ReceiverPulseTimer topRightSwitchInput(TOP_RIGHT_SWITCH);
  
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

  void topLeftSwitchInputInterrupt() {
    topLeftSwitchInput.onPulseStateChange();
  }

  void topRightSwitchInputInterrupt() {
    topRightSwitchInput.onPulseStateChange();
  }

  void setupReceivers() {
    
    rollInput.setupInterruptForThisChannel(rollInputInterrupt);
    pitchInput.setupInterruptForThisChannel(pitchInputInterrupt);
    throttleInput.setupInterruptForThisChannel(throttleInputInterrupt);
    yawInput.setupInterruptForThisChannel(yawInputInterrupt);
    topLeftSwitchInput.setupInterruptForThisChannel(topLeftSwitchInputInterrupt);
    topRightSwitchInput.setupInterruptForThisChannel(topRightSwitchInputInterrupt);

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

    constexpr int lowerLimitForCalibrationLogError = 800;
    constexpr int upperLimitForCalibrationLogError = 2200;
    auto logger = [&] () {
      if (min < lowerLimitForCalibrationLogError || max < lowerLimitForCalibrationLogError || mid < lowerLimitForCalibrationLogError || 
          min > upperLimitForCalibrationLogError || min > upperLimitForCalibrationLogError || min > upperLimitForCalibrationLogError) {
        Serial.println("recalibrate receiver endpoints not setup");
      }
    };


    updateMinAndMaxForTimer(min, max, mid, ROLL_START);
    logger();
    rollAligned = new ReceiverAligner(&rollInput, min, max, mid);

    updateMinAndMaxForTimer(min, max, mid, PITCH_START);
    logger();
    pitchAligned = new ReceiverAligner(&pitchInput, min, max, mid);

    updateMinAndMaxForTimer(min, max, mid, THROTTLE_START); 
    logger();
    throttleAligned  = new ReceiverAligner(&throttleInput, min, max, mid);

    updateMinAndMaxForTimer(min, max, mid, YAW_START); 
    logger();
    yawAligned  = new ReceiverAligner(&yawInput, min, max, mid);
    
  }

   void destroyAligners() {
    delete rollAligned;
    delete pitchAligned;
    delete throttleAligned;
    delete yawAligned;;
  }

  void printPulseLength(String channel, int pulseLength) {
    DEBUG_SERIAL.print(channel);DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print(pulseLength);DEBUG_SERIAL.print(" ");
  }

  void printAllPulseLengths() {
    printPulseLength("roll"            , rollInput.getPulseLengthMicros());
    printPulseLength("pitch"           , pitchInput.getPulseLengthMicros());
    printPulseLength("throttle"        , throttleInput.getPulseLengthMicros());
    printPulseLength("yaw"             , yawInput.getPulseLengthMicros());
    printPulseLength("top left switch" , topLeftSwitchInput.getPulseLengthMicros());
    printPulseLength("top right switch", topRightSwitchInput.getPulseLengthMicros());
  }

  void printAlignedPulses() 
  {
    printPulseLength("roll"            , rollAligned->getPulseLengthAligned());
    printPulseLength("pitch"           , pitchAligned->getPulseLengthAligned());
    printPulseLength("throttle"        , throttleAligned->getPulseLengthAligned());
    printPulseLength("yaw"             , yawAligned->getPulseLengthAligned());
  }

}

#endif