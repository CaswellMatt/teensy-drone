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
  mpu(SPI_CLOCK, SS_PIN) {
    message = String("Calibrate MARG");
}


void MARGCalibrationHandler::checkMaxAndMinAndSet(float32_t input, float32_t& max, float32_t& min) {
  Serial.println(input);
  if (input > max) max = input;
  if (input < min) min = input;
};


void MARGCalibrationHandler::printTitle() {
  Serial.println("MARG Calibration Menu");
}


void MARGCalibrationHandler::setup() {

  auto accelerationCalibrator = [this](){ calibrate(mpu.accel_data, 0); };

  optionsMap[ACCELERATION_INDEX] = new CalibrationOption(
    accelerationCalibrator, 
    accelerationOptionText
  );

  auto magneticsCalibrator = [this](){ calibrate(mpu.mag_data, 6 * sizeof(float32_t));  };
  
  optionsMap[MAGNETICS_INDEX] =  new CalibrationOption(
    magneticsCalibrator, 
    magneticsOptionText
  );

  auto printerAllValues = [this](){ printTestValues();  };

  optionsMap[PRINT_TEST_INDEX] = new CalibrationOption(
    printerAllValues, 
    printTestOptionText
  );


  auto printSavedCalibration = [this](){ readCalibrationValues();  };

  optionsMap[PRINT_SAVED_CALIBRATION_INDEX] = new CalibrationOption(
    printSavedCalibration, 
    printWrittenCalibrationValues
  );


  auto writeManualCalibrations = [this](){ writeManualValuesToCalibration();  };

  optionsMap[WRITE_MANUAL_TEST_VALUES_INDEX] = new CalibrationOption(
    writeManualCalibrations, 
    writeManualCalibrationText
  );



  SPI.begin();

  delay(20);

  mpu.init(true);

  uint8_t wai = mpu.whoami();
  if (wai == 0x71){
    Serial.println("Successful connection");
  }
  else{
    Serial.print("Failed connection: ");
    Serial.println(wai, HEX);
  }

  uint8_t wai_AK8963 = mpu.AK8963_whoami();
  if (wai_AK8963 == 0x48){
    Serial.println("Successful connection to mag");
  }
  else{
    Serial.print("Failed connection to mag: ");
    Serial.println(wai_AK8963, HEX);
  }
  mpu.calib_acc();

}


void MARGCalibrationHandler::calibrate(float* dataArray, int eepromStartAddress) {

  Serial.println("started calibration");

  float32_t xMin = 0;
  float32_t yMin = 0; 
  float32_t zMin = 0;

  float32_t xMax = 0; 
  float32_t yMax = 0; 
  float32_t zMax = 0;

  float32_t xAcculumator = 0;
  float32_t yAcculumator = 0;
  float32_t zAcculumator = 0;

  int count = 50;

  for (int j = 0; j < 1000; ++j) {

    Serial.println(j);

    for (int i = 0; i < count; ++i) {
      mpu.read_all();

      xAcculumator += *(dataArray);
      yAcculumator += *(dataArray + 1);
      zAcculumator += *(dataArray + 2);

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
  

  Serial.print("x max ");Serial.println(xMax);
  Serial.print("y max ");Serial.println(yMax);
  Serial.print("z max ");Serial.println(zMax);

  Serial.print("x min ");Serial.println(xMin);
  Serial.print("y min ");Serial.println(yMin);
  Serial.print("z min ");Serial.println(zMin);

  Serial.println(eeAddress);

  Serial.println("done");

}

void MARGCalibrationHandler::readCalibrationValues() {
  float32_t valueToPrint = 0;
  for (int i = 0; i < 12; ++i) {
    EEPROM.get(i * sizeof(float32_t), valueToPrint);
    Serial.println(valueToPrint);
  }
}


void MARGCalibrationHandler::printTestValues() {

  Serial.println("readings");
  for (int i = 0; i < 10000; ++i) {
    mpu.read_all();
    Serial.print(mpu.accel_data[0], 6);Serial.print(" ");
    Serial.print(mpu.accel_data[1], 6);Serial.print(" ");
    Serial.print(mpu.accel_data[2], 6);Serial.print("  ");
    // Serial.print(mpu.mag_data[0], 6);Serial.print(" ");
    // Serial.print(mpu.mag_data[1], 6);Serial.print(" ");
    // Serial.print(mpu.mag_data[2], 6);Serial.print(" ");
    Serial.println();
  
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

  Serial.println("manual values written");


}