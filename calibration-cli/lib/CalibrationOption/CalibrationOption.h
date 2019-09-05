#ifndef CALIBRATION_OPTION
#define CALIBRATION_OPTION

#include <functional>
const String a = "hellllooo";
class CalibrationOption {
public:
  CalibrationOption(
    std::function<void()> calibrationCallback, 
    const String optionMessage) : 
      callback(calibrationCallback),
      message(optionMessage)
  {
     
  }
  
  std::function<void()> callback;
  const String message;
  
  void printMessage() {
    Serial.println(message);
  }
};

#endif