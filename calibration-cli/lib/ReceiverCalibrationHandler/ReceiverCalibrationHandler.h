#ifndef RECEIVER_CALIBRATION_HANDLER
#define RECEIVER_CALIBRATION_HANDLER

#include <arm_math.h>
#include "CalibrationHandler.h"
#include "ReceiverPulseTimer.h"

class ReceiverCalibrationHandler : public CalibrationHandler {
public:
  ReceiverCalibrationHandler();

  virtual void setup() override;
  virtual void printTitle() override;

  void calibrateReceiver(ReceiverPulseTimer* timer, int eepromStartAddress);

};

#endif 