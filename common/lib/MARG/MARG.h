#ifndef _MARG_H
#define _MARG_H
#include <Arduino.h>
#include <EEPROM.h>

#include <SPI.h>
#include "MPU9250.h"

#include "Vector.h"
#define g 9.807f

class MARG {

  public:
    MARG();
    void read();

    Vector getMagnetics() { return magnetics; }
    Vector getAcceleration() { return acceleration; }
    Vector getRotationalRates() { return rotationalRates; }

  private:

    void readValuesForCalibration();

    MPU9250 mpu;

    Vector accelerationCalbrationMin;
    Vector accelerationCalbrationMax;

    Vector magneticsCalibrationMin;
    Vector magneticsCalibrationMax;

    Vector gyroscopeError;

    Vector magnetics;
    Vector acceleration;
    Vector rotationalRates;

};

#endif
