#include "MARG.h"

#include <SPI.h>
#include <MPU9250.h>
#include "MemoryLocations.h"

#define SPI_CLOCK 16000000  // 8MHz clock works.

#define SS_PIN   10 


MARG::MARG(bool accelerationSoftwareFiltersOn) : 
  m_mpu(SPI_CLOCK, SS_PIN),
  m_accelerationSoftwareFiltersOn(accelerationSoftwareFiltersOn)
  {

  delay(5000);

  SPI.begin();

	m_mpu.init(true);

	uint8_t wai = m_mpu.whoami();
	if (wai == 0x71) {
		Serial.println("Successful connection");
	}
	else{
		Serial.print("Failed connection: ");
		Serial.println(wai, HEX);
	}


	uint8_t wai_AK8963 = m_mpu.AK8963_whoami();
	if (wai_AK8963 == 0x48){
		Serial.println("Successful connection to mag");
	}
	else{
		Serial.print("Failed connection to mag: ");
		Serial.println(wai_AK8963, HEX);
	}

	m_mpu.calib_acc();
	m_mpu.calib_mag();

  float32_t xSum = 0;
  float32_t ySum = 0;
  float32_t zSum = 0;
  const int GYRO_CALIBRATION_COUNT = 1000;

  for (int i = 0; i < GYRO_CALIBRATION_COUNT; ++i)
  {
    m_mpu.read_all();
    xSum += m_mpu.gyro_data[0];
    ySum += m_mpu.gyro_data[1];
    zSum += m_mpu.gyro_data[2];
  }

  m_gyroscopeError.v0 = xSum / GYRO_CALIBRATION_COUNT;
  m_gyroscopeError.v1 = ySum / GYRO_CALIBRATION_COUNT;
  m_gyroscopeError.v2 = zSum / GYRO_CALIBRATION_COUNT;

  delay(100);

  readValuesForCalibration();
  
  // IMU.calibrateGyro();
}


void MARG::read() {

static const float32_t TO_RADIANS = 0.01745329251;
	m_mpu.read_all();

  m_rotationalRates.v0 = (m_mpu.gyro_data[0] - m_gyroscopeError.v0) * TO_RADIANS;
  m_rotationalRates.v1 = (m_mpu.gyro_data[1] - m_gyroscopeError.v1) * TO_RADIANS;
  m_rotationalRates.v2 = (m_mpu.gyro_data[2] - m_gyroscopeError.v2) * TO_RADIANS;

  // filt0.doing(map(m_mpu.accel_data[0], m_accelerationCalbrationMin.v0, m_accelerationCalbrationMax.v0, -G, G));
  // m_acceleration.v0 = filt0.getCurrent();
  
  // filt1.doing(map(m_mpu.accel_data[1], m_accelerationCalbrationMin.v1, m_accelerationCalbrationMax.v1, G, -G));
  // m_acceleration.v1 = filt1.getCurrent();

  // filt2.doing(map(m_mpu.accel_data[2], m_accelerationCalbrationMin.v2, m_accelerationCalbrationMax.v2, -G, G));
  // m_acceleration.v2 = filt2.getCurrent();

  m_acceleration.v0 = map(m_mpu.accel_data[0], m_accelerationCalbrationMin.v0, m_accelerationCalbrationMax.v0, -G, G);
  m_acceleration.v1 = map(m_mpu.accel_data[1], m_accelerationCalbrationMin.v1, m_accelerationCalbrationMax.v1, G, -G);
  m_acceleration.v2 = map(m_mpu.accel_data[2], m_accelerationCalbrationMin.v2, m_accelerationCalbrationMax.v2, -G, G);
  
  if (m_accelerationSoftwareFiltersOn)
  {
    m_filterX.update(m_acceleration.v0);
    m_acceleration.v0 = m_filterX.get();

    m_filterY.update(m_acceleration.v1);
    m_acceleration.v1 = m_filterY.get();
    
    m_filterZ.update(m_acceleration.v2);
    m_acceleration.v2 = m_filterZ.get();
  }
  
  //TODO: can use data read interrupts to avoid reading mag data too often when not changed?
  m_magnetics.v0 = map(m_mpu.mag_data[0], m_magneticsCalibrationMin.v0, m_magneticsCalibrationMax.v0, -1, 1);
  m_magnetics.v1 = map(m_mpu.mag_data[1], m_magneticsCalibrationMin.v1, m_magneticsCalibrationMax.v1, -1, 1);
  m_magnetics.v2 = map(m_mpu.mag_data[2], m_magneticsCalibrationMin.v2, m_magneticsCalibrationMax.v2, -1, 1);

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

  // Serial.print("accelX max ");Serial.println(m_accelerationCalbrationMax.v0);
  // Serial.print("accelY max ");Serial.println(m_accelerationCalbrationMax.v1);
  // Serial.print("accelZ max ");Serial.println(m_accelerationCalbrationMax.v2);

  // Serial.print("accelX min ");Serial.println(m_accelerationCalbrationMin.v0);
  // Serial.print("accelY min ");Serial.println(m_accelerationCalbrationMin.v1);
  // Serial.print("accelZ min ");Serial.println(m_accelerationCalbrationMin.v2);


  // Serial.print("magX max ");Serial.println(m_magneticsCalibrationMax.v0);
  // Serial.print("magY max ");Serial.println(m_magneticsCalibrationMax.v1);
  // Serial.print("magZ max ");Serial.println(m_magneticsCalibrationMax.v2);

  // Serial.print("magX min ");Serial.println(m_magneticsCalibrationMin.v0);
  // Serial.print("magY min ");Serial.println(m_magneticsCalibrationMin.v1);
  // Serial.print("magZ min ");Serial.println(m_magneticsCalibrationMin.v2);

}