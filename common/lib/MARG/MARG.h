#ifndef _MARG_H
#define _MARG_H
#include <Arduino.h>
#include <EEPROM.h>

#include <SPI.h>
#include "MPU9250.h"
#include "AccelerometerFilter.h"

#include "Vector.h"
#define G 9.807f


class MARG {

  public:
    MARG(bool accelerationSoftwareFilterOn = false);
    void read();

    Vector getMagnetics() { return m_magnetics; }
    Vector getAcceleration() { return m_acceleration; }
    Vector getRotationalRates() { return m_rotationalRates; }

  private:

    void readValuesForCalibration();

    MPU9250 m_mpu;

    AccelerometerFilter m_filterX;
    AccelerometerFilter m_filterY;
    AccelerometerFilter m_filterZ;

    Vector m_accelerationCalbrationMin;
    Vector m_accelerationCalbrationMax;

    Vector m_magneticsCalibrationMin;
    Vector m_magneticsCalibrationMax;

    Vector m_gyroscopeError;

    Vector m_magnetics;
    Vector m_acceleration;
    Vector m_rotationalRates;

    const bool m_accelerationSoftwareFiltersOn;

};

#endif
