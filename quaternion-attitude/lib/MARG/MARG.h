#ifndef _MARG_H
#define _MARG_H
#include <Arduino.h>
#include <EEPROM.h>
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

    MPU9250 IMU;

    Vector magnetics;
    Vector acceleration;
    Vector rotationalRates;

};

#endif
