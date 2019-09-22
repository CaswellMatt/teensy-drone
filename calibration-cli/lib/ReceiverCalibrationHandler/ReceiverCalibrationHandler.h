#ifndef RECEIVER_CALIBRATION_HANDLER
#define RECEIVER_CALIBRATION_HANDLER

#include <arm_math.h>
#include "MenuOptionHandler.h"
#include "ReceiverPulseTimer.h"

class ReceiverCalibrationHandler : public MenuOptionHandler {
public:
  ReceiverCalibrationHandler();

  virtual void setup() override;
  virtual void printTitle() override;

  void calibrateReceiver(ReceiverPulseTimer* timer, int eepromStartAddress);

};

#endif 