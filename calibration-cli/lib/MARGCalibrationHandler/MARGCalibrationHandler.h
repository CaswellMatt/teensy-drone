#ifndef MARG_CALIBRATION_HANDLER
#define MARG_CALIBRATION_HANDLER

#include <SPI.h>
#include <MARG.h>
#include <arm_math.h>
#include <map>
#include "MenuOptionHandler.h"

#define SPI_CLOCK 8000000  // 8MHz clock works.

#define SS_PIN   10 
#define INT_PIN  3
#define LED      13

class MARGCalibrationHandler : public MenuOptionHandler {
public:
  MARG m_marg;

  MARGCalibrationHandler();

  virtual void setup() override;
  virtual void printTitle() override;
  
  void checkMaxAndMinAndSet(float32_t input, float32_t& max, float32_t& min);
  void checkMaxAndMinAndSetForOneValue(float32_t input, float32_t& maxOrMin);

  void calibrate(Vector (MARG::*data)(), int eepromStartAddress);
  void calibrateOneValue(int index, Vector (MARG::*data)(), int eepromStartAddress);
  void calibrateAcceleration();
  void readCalibrationValues();
  void printTestValues();
  void writeManualValuesToCalibration();
  void calibrateMagnitometer();
};

#endif 