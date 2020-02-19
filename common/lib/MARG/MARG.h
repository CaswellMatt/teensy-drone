#ifndef _MARG_H
#define _MARG_H
#include <Arduino.h>
#include <EEPROM.h>

#include <SPI.h>
#include "MPU9250.h"
#include "AccelerometerFilter.h"
#include "GyroscopeFilter.h"

#include "Vector.h"
#define G 9.807f


class MARG {

  public:
    MARG(bool accelerationSoftwareFilterOn = false, bool gyroSoftwareFiltersOn = false);
    void read();
    void readRaw();

    Vector getMagneticsRaw() { return m_magneticsRaw; }
    Vector getAccelerationRaw() { return m_accelerationRaw; }
    Vector getRotationalRatesRaw() { return m_rotationalRatesRaw; }

    Vector getMagnetics() { return m_magnetics; }
    Vector getAcceleration() { return m_acceleration; }
    Vector getRotationalRates() { return m_rotationalRates; }

  private:

    void readValuesForCalibration();

    MPU9250 m_mpu;

    AccelerometerFilter m_accelerometerFilterX;
    AccelerometerFilter m_accelerometerFilterY;
    AccelerometerFilter m_accelerometerFilterZ;

    GyroscopeFilter m_gyroscopeFilterX;
    GyroscopeFilter m_gyroscopeFilterY;
    GyroscopeFilter m_gyroscopeFilterZ;

    Vector m_accelerationCalbrationMin;
    Vector m_accelerationCalbrationMax;

    Vector m_magneticsCalibrationMin;
    Vector m_magneticsCalibrationMax;

    Vector m_gyroscopeError;

    Vector m_magneticsRaw;
    Vector m_accelerationRaw;
    Vector m_rotationalRatesRaw;

    Vector m_magnetics;
    Vector m_acceleration;
    Vector m_rotationalRates;

    const bool m_accelerationSoftwareFiltersOn;
    const bool m_gyroSoftwareFiltersOn;

};

#endif
