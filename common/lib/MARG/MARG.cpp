#include "MARG.h"

#include <SPI.h>
#include <MPU9250.h>
#include "MemoryLocations.h"

#define ONE_MHZ 1000000
#define SPI_CLOCK 18 * ONE_MHZ 

#define SS_PIN   10 

namespace  {

  const int xAxisIndex = 0;
  const int yAxisIndex = 1;
  const int zAxisIndex = 2;

  const int rollAxisIndex = 0;
  const int pitchAxisIndex = 1;
  const int yawAxisIndex = 2;

  const int direction = 1;

  //roll positive right wing down
  //pitch positive nose down 
  //yaw positive nose left
  const int gyroXDirection = direction;
  const int gyroYDirection = direction;
  const int gyroZDirection = direction;

  const int accelXDirection = direction;
  const int accelYDirection = direction;
  const int accelZDirection = direction;

}

MARG::MARG(bool accelerationSoftwareFiltersOn, bool gyroSoftwareFiltersOn) : 
  m_mpu(SPI_CLOCK, SS_PIN),
  m_accelerationSoftwareFiltersOn(accelerationSoftwareFiltersOn),
  m_gyroSoftwareFiltersOn(gyroSoftwareFiltersOn) {

  delay(5000);
  SPI.begin();

  m_mpu.init();

  uint8_t wai = m_mpu.whoami();
  if (wai == 0x71) {
    DEBUG_SERIAL.println("Successful connection");
  } else{
    DEBUG_SERIAL.print("Failed connection: ");
    DEBUG_SERIAL.println(wai, HEX);
  }


  uint8_t wai_AK8963 = m_mpu.AK8963_whoami();
  if (wai_AK8963 == 0x48){
    DEBUG_SERIAL.println("Successful connection to mag");
  } else{
    DEBUG_SERIAL.print("Failed connection to mag: ");
    DEBUG_SERIAL.println(wai_AK8963, HEX);
  }
  
  float32_t xSum = 0;
  float32_t ySum = 0;
  float32_t zSum = 0;
  const int GYRO_CALIBRATION_COUNT = 3000;

  for (int i = 0; i < GYRO_CALIBRATION_COUNT; ++i)
  {
    m_mpu.read_all();
    xSum += m_mpu.gyro_data[rollAxisIndex];
    ySum += m_mpu.gyro_data[pitchAxisIndex];
    zSum += m_mpu.gyro_data[yawAxisIndex];
    delay(1);
  }

  m_gyroscopeError.v0 = xSum / GYRO_CALIBRATION_COUNT;
  m_gyroscopeError.v1 = ySum / GYRO_CALIBRATION_COUNT;
  m_gyroscopeError.v2 = zSum / GYRO_CALIBRATION_COUNT;

  delay(100);

  readValuesForCalibration();
  
}


void MARG::read() {

  static const float32_t TO_RADIANS = 0.01745329251;
	m_mpu.read_all();

  m_rotationalRatesRaw.v0 = gyroXDirection * (m_mpu.gyro_data[rollAxisIndex]  - m_gyroscopeError.v0) * TO_RADIANS;
  m_rotationalRatesRaw.v1 = gyroYDirection * (m_mpu.gyro_data[pitchAxisIndex] - m_gyroscopeError.v1) * TO_RADIANS;
  m_rotationalRatesRaw.v2 = gyroZDirection * (m_mpu.gyro_data[yawAxisIndex]   - m_gyroscopeError.v2) * TO_RADIANS;

  m_accelerationRaw.v0 = accelXDirection * map(m_mpu.accel_data[xAxisIndex], m_accelerationCalbrationMin.v0, m_accelerationCalbrationMax.v0, -G, G);
  m_accelerationRaw.v1 = accelYDirection * map(m_mpu.accel_data[yAxisIndex], m_accelerationCalbrationMin.v1, m_accelerationCalbrationMax.v1, -G, G);
  m_accelerationRaw.v2 = accelZDirection * map(m_mpu.accel_data[zAxisIndex], m_accelerationCalbrationMin.v2, m_accelerationCalbrationMax.v2, -G, G);

  if (m_accelerationSoftwareFiltersOn)
  {
    m_accelerometerFilterX.update(m_accelerationRaw.v0);
    m_acceleration.v0 = m_accelerometerFilterX.get();

    m_accelerometerFilterY.update(m_accelerationRaw.v1);
    m_acceleration.v1 = m_accelerometerFilterY.get();
    
    m_accelerometerFilterZ.update(m_accelerationRaw.v2);
    m_acceleration.v2 = m_accelerometerFilterZ.get();
  }  else {
    m_acceleration = m_accelerationRaw;
  }

  if (m_gyroSoftwareFiltersOn) {
    m_gyroscopeFilterX.update(m_rotationalRatesRaw.v0);
    m_rotationalRates.v0 = m_gyroscopeFilterX.get();
    
    m_gyroscopeFilterY.update(m_rotationalRatesRaw.v1);
    m_rotationalRates.v1 = m_gyroscopeFilterY.get();
    
    m_gyroscopeFilterZ.update(m_rotationalRatesRaw.v2);
    m_rotationalRates.v2 = m_gyroscopeFilterZ.get();
  }  else {
    m_rotationalRates = m_rotationalRatesRaw;
  }
  
  //TODO: can use data read interrupts to avoid reading mag data too often when not changed?
  m_magneticsRaw.v0 = map(m_mpu.mag_data[xAxisIndex], m_magneticsCalibrationMin.v0, m_magneticsCalibrationMax.v0, -1, 1);
  m_magneticsRaw.v1 = map(m_mpu.mag_data[yAxisIndex], m_magneticsCalibrationMin.v1, m_magneticsCalibrationMax.v1, -1, 1);
  m_magneticsRaw.v2 = map(m_mpu.mag_data[zAxisIndex], m_magneticsCalibrationMin.v2, m_magneticsCalibrationMax.v2, -1, 1);

  m_magnetics = m_magneticsRaw;
}

void MARG::readRaw() {
  m_mpu.read_all();
  
  m_rotationalRatesRaw.v0 = m_mpu.gyro_data[rollAxisIndex];
  m_rotationalRatesRaw.v1 = m_mpu.gyro_data[pitchAxisIndex];
  m_rotationalRatesRaw.v2 = m_mpu.gyro_data[yawAxisIndex];

  m_accelerationRaw.v0 = m_mpu.accel_data[xAxisIndex];
  m_accelerationRaw.v1 = m_mpu.accel_data[yAxisIndex];
  m_accelerationRaw.v2 = m_mpu.accel_data[zAxisIndex];

  //TODO: can use data read interrupts to avoid reading mag data too often when not changed?
  m_magneticsRaw.v0 = m_mpu.mag_data[xAxisIndex];
  m_magneticsRaw.v1 = m_mpu.mag_data[yAxisIndex];
  m_magneticsRaw.v2 = m_mpu.mag_data[zAxisIndex];

}

void MARG::readValuesForCalibration() {
  int eeAddress = ACCEL_START;

  EEPROM.get(eeAddress, m_accelerationCalbrationMin.v0);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, m_accelerationCalbrationMax.v0);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, m_accelerationCalbrationMin.v1);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, m_accelerationCalbrationMax.v1);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, m_accelerationCalbrationMin.v2);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, m_accelerationCalbrationMax.v2);


  
  eeAddress = MAG_START;
  EEPROM.get(eeAddress, m_magneticsCalibrationMin.v0);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, m_magneticsCalibrationMax.v0);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, m_magneticsCalibrationMin.v1);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, m_magneticsCalibrationMax.v1);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, m_magneticsCalibrationMin.v2);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, m_magneticsCalibrationMax.v2);

  // DEBUG_SERIAL.print("accelX max ");DEBUG_SERIAL.println(m_accelerationCalbrationMax.v0);
  // DEBUG_SERIAL.print("accelY max ");DEBUG_SERIAL.println(m_accelerationCalbrationMax.v1);
  // DEBUG_SERIAL.print("accelZ max ");DEBUG_SERIAL.println(m_accelerationCalbrationMax.v2);

  // DEBUG_SERIAL.print("accelX min ");DEBUG_SERIAL.println(m_accelerationCalbrationMin.v0);
  // DEBUG_SERIAL.print("accelY min ");DEBUG_SERIAL.println(m_accelerationCalbrationMin.v1);
  // DEBUG_SERIAL.print("accelZ min ");DEBUG_SERIAL.println(m_accelerationCalbrationMin.v2);


  // DEBUG_SERIAL.print("magX max ");DEBUG_SERIAL.println(m_magneticsCalibrationMax.v0);
  // DEBUG_SERIAL.print("magY max ");DEBUG_SERIAL.println(m_magneticsCalibrationMax.v1);
  // DEBUG_SERIAL.print("magZ max ");DEBUG_SERIAL.println(m_magneticsCalibrationMax.v2);

  // DEBUG_SERIAL.print("magX min ");DEBUG_SERIAL.println(m_magneticsCalibrationMin.v0);
  // DEBUG_SERIAL.print("magY min ");DEBUG_SERIAL.println(m_magneticsCalibrationMin.v1);
  // DEBUG_SERIAL.print("magZ min ");DEBUG_SERIAL.println(m_magneticsCalibrationMin.v2);

}