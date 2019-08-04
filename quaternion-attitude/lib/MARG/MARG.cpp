#include "MARG.h"

MARG::MARG() : IMU(SPI, 10) {


  IMU.begin();
  IMU.setSrd(1);
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ);

  this->readValuesForCalibration();
  this->calculateCalibrations();

  IMU.calibrateGyro();
}


void MARG::read() {
  IMU.readSensor();

  rotationalRates.v0 = IMU.getGyroX_rads();
  rotationalRates.v1 = IMU.getGyroY_rads();
  rotationalRates.v2 = IMU.getGyroZ_rads();

  acceleration.v0 = map(IMU.getAccelX_mss(), accelerationCalbrationMin.v0, accelerationCalbrationMax.v0, -g, g);
  acceleration.v1 = map(IMU.getAccelY_mss(), accelerationCalbrationMin.v1, accelerationCalbrationMax.v1, -g, g);
  acceleration.v2 = map(IMU.getAccelZ_mss(), accelerationCalbrationMin.v2, accelerationCalbrationMax.v2, -g, g);

  magnetics.v0 = map(IMU.getMagX_uT(), magneticsCalibrationMin.v0, magneticsCalibrationMax.v0, -1, 1);
  magnetics.v1 = map(IMU.getMagY_uT(), magneticsCalibrationMin.v1, magneticsCalibrationMax.v1, -1, 1);
  magnetics.v2 = map(IMU.getMagZ_uT(), magneticsCalibrationMin.v2, magneticsCalibrationMax.v2, -1, 1);

}

void MARG::calculateCalibrations() {

//   Vector magneticCalibrations   = magneticsCalibrationMax.minus(magneticsCalibrationMin);
//   Vector accelerationCalibrations = accelerationCalbrationMax.minus(accelerationCalbrationMin);

//   Vector magneticsScale =
//     Vector( 1 / magneticCalibrations.v0, 
//             1 / magneticCalibrations.v1, 
//             1 / magneticCalibrations.v2 );

//   Vector accelerationScale =
//     Vector( 1 / accelerationCalibrations.v0, 
//             1 / accelerationCalibrations.v1, 
//             1 / accelerationCalibrations.v2 );

//   Vector accelerationOffset = accelerationCalibrations.divide(2);
//   Vector magneticOffset   = magneticCalibrations.divide(2);

//   IMU.setAccelCalX(accelerationOffset.v0, accelerationScale.v0);
//   IMU.setAccelCalY(accelerationOffset.v1, accelerationScale.v1);
//   IMU.setAccelCalZ(accelerationOffset.v2, accelerationScale.v2);

//   IMU.setMagCalX(magneticOffset.v0, magneticsScale.v0);
//   IMU.setMagCalY(magneticOffset.v1, magneticsScale.v1);
//   IMU.setMagCalZ(magneticOffset.v2, magneticsScale.v2);

}


void MARG::readValuesForCalibration() {
  int eeAddress = 0;

  EEPROM.get(eeAddress, accelerationCalbrationMin.v0);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, accelerationCalbrationMax.v0);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, accelerationCalbrationMin.v1);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, accelerationCalbrationMax.v1);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, accelerationCalbrationMin.v2);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, accelerationCalbrationMax.v2);


  
  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, magneticsCalibrationMin.v0);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, magneticsCalibrationMax.v0);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, magneticsCalibrationMin.v1);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, magneticsCalibrationMax.v1);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, magneticsCalibrationMin.v2);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, magneticsCalibrationMax.v2);

}