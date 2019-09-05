#ifndef MARG_CALIBRATION_HANDLER
#define MARG_CALIBRATION_HANDLER

#include <SPI.h>
#include <MPU9250.h>
#include <arm_math.h>
#include <map>
#include "CalibrationHandler.h"

#define SPI_CLOCK 8000000  // 8MHz clock works.

#define SS_PIN   10 
#define INT_PIN  3
#define LED      13

class MARGCalibrationHandler : public CalibrationHandler {
public:
  MPU9250 mpu;

  MARGCalibrationHandler();

  virtual void setup() override;
  virtual void printTitle() override;
  
  void checkMaxAndMinAndSet(float32_t input, float32_t& max, float32_t& min);

  void calibrate(float* dataArray, int eepromStartAddress);
  void readCalibrationValues();
  void printTestValues();
  void writeManualValuesToCalibration();

};

#endif 