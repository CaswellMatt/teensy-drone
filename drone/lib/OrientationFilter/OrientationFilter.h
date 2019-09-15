#ifndef _ORIENTATION_FILTER_H
#define _ORIENTATION_FILTER_H
#include "MARG.h"
#include <Arduino.h>
#include "Quaternion.h"
#include <arm_math.h>

class OrientationFilter {

  public:
    OrientationFilter();

    void update(float deltat);

  private:
    MARG marg;

    Quaternion orientationGyro;
    Quaternion orientation;

    Vector magneticsPrevious;
    Quaternion calculateAccelerationQuaternion(Vector acceleration);

    Quaternion calculateDeltaAccelerationQuaternion(Vector gravityUnit);
    Quaternion calculateDeltaMagneticsQuaternion(Vector magneticsUnit);

    Quaternion linearInterpolation(Quaternion q, float32_t alpha);
    Quaternion sphericalInterpolation(Quaternion q, float32_t alpha);

    Quaternion calculateComplementaryQuaternion(Quaternion q, float32_t gainFactor) ;

};
#endif
