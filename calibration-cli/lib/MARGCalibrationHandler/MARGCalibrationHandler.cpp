#include "MARGCalibrationHandler.h"
#include <Arduino.h>
#include <SPI.h>
#include <MPU9250.h>

#include <arm_math.h>

#include <functional>

#define ACCELERATION_INDEX             1
#define MAGNETICS_INDEX                2
#define PRINT_TEST_INDEX               3
#define PRINT_SAVED_CALIBRATION_INDEX  4
#define WRITE_MANUAL_TEST_VALUES_INDEX 5

const String accelerationOptionText = "Calibrate Acceleration";
const String magneticsOptionText = "Calibrate Magnetics";
const String printTestOptionText = "Print values to find optimal endpoints for calibration";
const String printWrittenCalibrationValues = "Print written calibration values \n" \
                                              "   acceleration min x, max x, min y, max y, min z, max z, \n" \
                                              "   magnetics min x, max x, min y, max y, min z, max z";
String writeManualCalibrationText = "Write manual values";

MARGCalibrationHandler::MARGCalibrationHandler() : 
  m_marg() {
    m_message = String("Calibrate MARG");
}


void MARGCalibrationHandler::checkMaxAndMinAndSet(float32_t input, float32_t& max, float32_t& min) {
  DEBUG_SERIAL.println(input);
  if (input > max) max = input;
  if (input < min) min = input;
};

void MARGCalibrationHandler::checkMaxAndMinAndSetForOneValue(float32_t input, float32_t& maxOrMin) {
  DEBUG_SERIAL.println(input);
  if (input > maxOrMin) maxOrMin = input;
  if (input < maxOrMin) maxOrMin = input;
};


void MARGCalibrationHandler::printTitle() {
  DEBUG_SERIAL.println("MARG Calibration Menu");
}


void MARGCalibrationHandler::setup() {

  addExit(this);
  addOption(this, &MARGCalibrationHandler::calibrateAcceleration, accelerationOptionText);
  addOption(this, &MARGCalibrationHandler::calibrateMagnitometer, magneticsOptionText);
  addOption(this, &MARGCalibrationHandler::printTestValues      , printTestOptionText);
  addOption(this, &MARGCalibrationHandler::readCalibrationValues         , printWrittenCalibrationValues);
  addOption(this, &MARGCalibrationHandler::writeManualValuesToCalibration, writeManualCalibrationText);

  delay(20);

}

void MARGCalibrationHandler::calibrateAcceleration() {

  int eepromAddress = ACCEL_START;

  auto runCalibrationAndIncrementEeprom = [&](int accelIndex, String printString) {
    delay(1000);
    DEBUG_SERIAL.println(printString);
    delay(3000);
    calibrateOneValue(accelIndex, &MARG::getAccelerationRaw, eepromAddress);

    eepromAddress+=sizeof(float32_t);
  };

  DEBUG_SERIAL.println("calibrate x...");

  runCalibrationAndIncrementEeprom(0, "pitch nose up");
  runCalibrationAndIncrementEeprom(0, "pitch nose down");

  DEBUG_SERIAL.println("calibrate y...");
    
  runCalibrationAndIncrementEeprom(1, "roll right side down");
  runCalibrationAndIncrementEeprom(1, "roll right side up");
  
  DEBUG_SERIAL.println("calibrate z...");

  runCalibrationAndIncrementEeprom(2, "turn top to face down");
  runCalibrationAndIncrementEeprom(2, "turn back upright");

}

void MARGCalibrationHandler::calibrateOneValue(int index, Vector (MARG::*dataGetter)(), int eepromStartAddress) {

  DEBUG_SERIAL.println("started calibration");

  float32_t minOrMax = 0;
 
  float32_t acculumator = 0;

  int count = 400;

  for (int j = 0; j < 10; ++j) {

    DEBUG_SERIAL.println(j);

    for (int i = 0; i < count; ++i) {
      m_marg.readRaw();
      Vector data = (m_marg.*(dataGetter))();
      acculumator += data[index];

      delayMicroseconds(500);
    }

    float32_t average = acculumator / count;

    checkMaxAndMinAndSetForOneValue(average, minOrMax);

    acculumator = 0;

  }

  int eeAddress = eepromStartAddress;

  EEPROM.put(eeAddress, minOrMax);

  DEBUG_SERIAL.print("value ");DEBUG_SERIAL.println(minOrMax);

  DEBUG_SERIAL.println("done");

}

void MARGCalibrationHandler::readCalibrationValues() {
  float32_t valueToPrint = 0;
  for (int i = 0; i < 6; ++i) {
    EEPROM.get(ACCEL_START + i * sizeof(float32_t), valueToPrint);
    DEBUG_SERIAL.println(valueToPrint);
  }

  for (int i = 0; i < 6; ++i) {
    EEPROM.get(MAG_START + i * sizeof(float32_t), valueToPrint);
    DEBUG_SERIAL.println(valueToPrint);
  }
}



void MARGCalibrationHandler::calibrate(Vector (MARG::*dataGetter)(), int eepromStartAddress) {

  DEBUG_SERIAL.println("started calibration");

  float32_t xMin = 0;
  float32_t yMin = 0; 
  float32_t zMin = 0;

  float32_t xMax = 0; 
  float32_t yMax = 0; 
  float32_t zMax = 0;

  float32_t xAcculumator = 0;
  float32_t yAcculumator = 0;
  float32_t zAcculumator = 0;

  int count = 400;

  for (int j = 0; j < 180; ++j) {

    DEBUG_SERIAL.println(j);

    for (int i = 0; i < count; ++i) {
      m_marg.readRaw();
      Vector data = (m_marg.*(dataGetter))();
      xAcculumator += data.v0;
      yAcculumator += data.v1;
      zAcculumator += data.v2;

      delayMicroseconds(500);
    }

    float32_t xAverage = xAcculumator / count;
    float32_t yAverage = yAcculumator / count;
    float32_t zAverage = zAcculumator / count;

    checkMaxAndMinAndSet(xAverage, xMax, xMin);
    checkMaxAndMinAndSet(yAverage, yMax, yMin);
    checkMaxAndMinAndSet(zAverage, zMax, zMin);

    xAcculumator = 0;
    yAcculumator = 0;
    zAcculumator = 0;

  }


  int eeAddress = eepromStartAddress;

  EEPROM.put(eeAddress, xMin);

  eeAddress += sizeof(float32_t);
  EEPROM.put(eeAddress, xMax);
  
  eeAddress += sizeof(float32_t);
  EEPROM.put(eeAddress, yMin);
  
  eeAddress += sizeof(float32_t);
  EEPROM.put(eeAddress, yMax);
  
  eeAddress += sizeof(float32_t);
  EEPROM.put(eeAddress, zMin);
  
  eeAddress += sizeof(float32_t);
  EEPROM.put(eeAddress, zMax);
  

  DEBUG_SERIAL.print("x max ");DEBUG_SERIAL.println(xMax);
  DEBUG_SERIAL.print("y max ");DEBUG_SERIAL.println(yMax);
  DEBUG_SERIAL.print("z max ");DEBUG_SERIAL.println(zMax);

  DEBUG_SERIAL.print("x min ");DEBUG_SERIAL.println(xMin);
  DEBUG_SERIAL.print("y min ");DEBUG_SERIAL.println(yMin);
  DEBUG_SERIAL.print("z min ");DEBUG_SERIAL.println(zMin);

  DEBUG_SERIAL.println(eeAddress);

  DEBUG_SERIAL.println("done");

}


void MARGCalibrationHandler::printTestValues() {

  DEBUG_SERIAL.println("readings");
  for (int i = 0; i < 10000; ++i) {
    m_marg.readRaw();
    DEBUG_SERIAL.print(" accel x ");DEBUG_SERIAL.print(m_marg.getAccelerationRaw().v0, 6);DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print(" accel y ");DEBUG_SERIAL.print(m_marg.getAccelerationRaw().v1, 6);DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print(" accel z ");DEBUG_SERIAL.print(m_marg.getAccelerationRaw().v2, 6);DEBUG_SERIAL.print("       ");
    DEBUG_SERIAL.print(" mag x ");DEBUG_SERIAL.print(m_marg.getMagneticsRaw().v0, 6);DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print(" mag y ");DEBUG_SERIAL.print(m_marg.getMagneticsRaw().v1, 6);DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print(" mag z ");DEBUG_SERIAL.print(m_marg.getMagneticsRaw().v2, 6);DEBUG_SERIAL.print("       ");
    DEBUG_SERIAL.print(" gyro x ");DEBUG_SERIAL.print(m_marg.getRotationalRatesRaw().v0, 6);DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print(" gyro y ");DEBUG_SERIAL.print(m_marg.getRotationalRatesRaw().v1, 6);DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print(" gyro z ");DEBUG_SERIAL.print(m_marg.getRotationalRatesRaw().v2, 6);DEBUG_SERIAL.print(" ");

    DEBUG_SERIAL.println();
  
    delay(2);
  }
}

void MARGCalibrationHandler::writeManualValuesToCalibration() {

  int eeAddress = ACCEL_START;

  EEPROM.put(eeAddress, -1.0);

  eeAddress += sizeof(float32_t);
  EEPROM.put(eeAddress, 1.00);
  
  eeAddress += sizeof(float32_t);
  EEPROM.put(eeAddress, -1.01);
  
  eeAddress += sizeof(float32_t);
  EEPROM.put(eeAddress, 1.02);
  
  eeAddress += sizeof(float32_t);
  EEPROM.put(eeAddress, -1.03);
  
  eeAddress += sizeof(float32_t);
  EEPROM.put(eeAddress, 0.97);



  eeAddress = MAG_START;
  EEPROM.put(eeAddress, -36.395508);
  
  eeAddress += sizeof(float32_t);
  EEPROM.put(eeAddress, 52.374023);
  
  eeAddress += sizeof(float32_t);
  EEPROM.put(eeAddress, -86.74688);
  
  eeAddress += sizeof(float32_t);
  EEPROM.put(eeAddress, 6.4125);
  
  eeAddress += sizeof(float32_t);
  EEPROM.put(eeAddress, -31.245703);
  
  
  eeAddress += sizeof(float32_t);
  EEPROM.put(eeAddress, 60.087891);

  DEBUG_SERIAL.println("manual values written");


}

void MARGCalibrationHandler::calibrateMagnitometer() {
  calibrate(&MARG::getMagneticsRaw, MAG_START);  
}