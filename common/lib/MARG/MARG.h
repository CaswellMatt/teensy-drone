#ifndef _MARG_H
#define _MARG_H
#include <Arduino.h>
#include <EEPROM.h>

#include <SPI.h>
#include "MPU9250.h"

#include "Vector.h"
#define g 9.807f


class filter {
public:
  const static int size = 2;
  float inputCoefficientsB[size] = { 0.086364,   0.086364 };
  float outputCoefficientsA[size] = { 1.00000,  -0.82727 };

  float filterInputs[size];
  float filterOutputs[size];
  int currentIndexInCircular = 0;
  bool hasFilled = false;

  float currentOutput = 0;

  void doing(float input) {

    int filterOrder = size;

    filterInputs[currentIndexInCircular] = input;

    if (hasFilled) {
        for(int i = 0; i < filterOrder; i++) {
          int j = (currentIndexInCircular + i) % size;
          currentOutput += outputCoefficientsA[j] * filterOutputs[-j];
          currentOutput = -currentOutput;
        }

        for(int i = 0; i < filterOrder; i++) {
          int j = (currentIndexInCircular + i) % size;
          currentOutput += inputCoefficientsB[j] * filterInputs[-j];
        }

        filterOutputs[currentIndexInCircular] = currentOutput;

    } else {
      filterOutputs[currentIndexInCircular] = input;
      if (currentIndexInCircular == size) {
        hasFilled = true;
      }
      currentOutput = input;
    }

    currentIndexInCircular++;
    currentIndexInCircular %= size;

  }

  float getCurrent() {
    return currentOutput;
  }
};


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

    filter filt0;
    filter filt1;
    filter filt2;

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
