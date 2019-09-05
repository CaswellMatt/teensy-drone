#ifndef CALIBRATION_HANDLER
#define CALIBRATION_HANDLER
#include <Arduino.h>
#include <map>
#include "CalibrationOption.h"
#include "EEPROM.h"
#include "MemoryLocations.h"

class CalibrationHandler {
public:
  std::map<int, CalibrationOption*> optionsMap;

  void printMessage() {
    Serial.println(message);
  }

  virtual void printTitle() = 0;

  virtual void setup() = 0;

  void start();
protected:
  String message;

};

#endif