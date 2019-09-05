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

    void readValuesForCalibration();
    void calculateCalibrations();

    Vector accelerationCalbrationMin;
    Vector accelerationCalbrationMax;

    Vector magneticsCalibrationMin;
    Vector magneticsCalibrationMax;

    Vector gyroscopeError;

    MPU9250 mpu;

    Vector magnetics;
    Vector acceleration;
    Vector rotationalRates;

};

#endif
