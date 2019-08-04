#ifndef _QUATERNION_H
#define _QUATERNION_H
#include "Vector.h"
#include <arm_math.h>
#include "Math.h"
class Vector;
class Quaternion {
  public:
    float32_t q0 = 1;
    float32_t q1 = 0;
    float32_t q2 = 0;
    float32_t q3 = 0;
    Quaternion();
    Quaternion(float32_t _q0, float32_t _q1, float32_t _q2, float32_t _q3);

    float32_t magnitude();
    Quaternion add(Quaternion q);
    Quaternion minus(Quaternion q);
    Quaternion scale(float32_t scaleFactor);
    Quaternion asUnit();
    Quaternion  multiply(Quaternion q);
    Quaternion  conjugate();
    Vector toVector();
    Vector rotate(Vector v);
    Quaternion  AngularRatesToQuaternion(Vector w, float32_t dt);

};

#endif
