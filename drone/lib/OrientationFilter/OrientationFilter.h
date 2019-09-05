#ifndef _ORIENTATION_FILTER_H
#define _ORIENTATION_FILTER_H
#include "MARG.h"
#include "Quaternion.h"
#include <Arduino.h>
#include <arm_math.h>

class OrientationFilter {

  public:
    OrientationFilter();

    MARG marg;

    void update(float deltat);
    Quaternion calculateAccelerationQuaternion(Vector acceleration);

    Quaternion calculateDeltaAccelerationQuaternion(Vector gravityUnit);
    Quaternion calculateDeltaMagneticsQuaternion(Vector magneticsUnit);

    Quaternion linearInterpolation(Quaternion q, float32_t alpha);
    Quaternion sphericalInterpolation(Quaternion q, float32_t alpha);

    Quaternion calculateComplementaryQuaternion(Quaternion q, float32_t gainFactor) ;

    Quaternion orientationGyro;
    Quaternion orientation;

    Vector magneticsPrevious;

};
#endif
