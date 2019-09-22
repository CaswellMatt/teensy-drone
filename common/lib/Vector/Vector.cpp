#include "Vector.h"
#include "Quaternion.h"

Vector::Vector() : v0(0), v1(0), v2(0) {}

Vector::Vector(float32_t _v0, float32_t _v1, float32_t _v2) {
  v0 = _v0;
  v1 = _v1;
  v2 = _v2;
}

Vector Vector::minus(Vector v) {
  return Vector(v0 - v.v0, v1 - v.v1, v2 - v.v2);
}

Vector Vector::add(Vector v) {
  return Vector(v0 + v.v0, v1 + v.v1, v2 + v.v2);
}

Vector Vector::divide(float32_t a) {
  return Vector(v0 / a, v1 / a, v2 / a);
}

Vector Vector::multiply(float32_t a) {
  return Vector(v0 * a, v1 * a, v2 * a);
}

float32_t Vector::magnitude() {
  return sqrt(v0 * v0 + v1 * v1 + v2 * v2);
}

Vector Vector::asUnit() {
  float32_t inverseMagnitude = Math::fastInverseSquareRoot(v0 * v0 + v1 * v1 + v2 * v2);

  return Vector(v0 * inverseMagnitude, v1 * inverseMagnitude, v2 * inverseMagnitude);
}

Quaternion Vector::toQuaternion() {
  return Quaternion(0, v0, v1, v2);
};
