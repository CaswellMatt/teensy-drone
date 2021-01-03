#ifndef RECEIVER_CALIBRATION_HANDLER
#define RECEIVER_CALIBRATION_HANDLER

#include <arm_math.h>
#include "MenuOptionHandler.h"
#include "IReceiverChannel.h"

class ReceiverCalibrationHandler : public MenuOptionHandler {
public:
  ReceiverCalibrationHandler();

  virtual void setup() override;
  virtual void printTitle() override;

  void calibrateReceiver(IReceiverChannel* channel, int eepromStartAddress);
  void printReceiver(String receiverName, int startAddress);

  void rollCalibrator();
  void pitchCalibrator();
  void yawCalibrator();
  void throttleCalibrator();
  void printAll();
  void printSaved();
};

#endif 