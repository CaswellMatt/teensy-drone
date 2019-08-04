#ifndef _VECTOR_H
#define _VECTOR_H
#include "Math.h"
#include "Quaternion.h"
#include <arm_math.h>
class Quaternion;
class Vector {
  public:
    float32_t v0;
    float32_t v1;
    float32_t v2;

    Vector();
    Vector(float32_t _v0, float32_t _v1, float32_t _v2);

    Vector minus(Vector v);
    Vector add(Vector v);
    Vector divide(float32_t a);
    Vector multiply(float32_t a);
    float32_t magnitude();
    Vector asUnit();
    Quaternion toQuaternion();

    bool operator == (const Vector& rhs) {
      return v0 == rhs.v0 &&
             v1 == rhs.v1 &&
             v2 == rhs.v2;
    };

    bool operator != (const Vector& rhs) {
      return v0 != rhs.v0 &&
             v1 != rhs.v1 &&
             v2 != rhs.v2;
    };

};

#endif
