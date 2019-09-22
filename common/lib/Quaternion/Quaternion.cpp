#include "Quaternion.h"
#include "Vector.h"
#include "math.h"

Quaternion::Quaternion() : q0(1), q1(0), q2(0), q3(0) {}

Quaternion::Quaternion(float32_t _q0, float32_t _q1, float32_t _q2, float32_t _q3) {
  q0 = _q0;
  q1 = _q1;
  q2 = _q2;
  q3 = _q3;
}

float32_t Quaternion::magnitude() {
  return(
    sqrt(
      q0 * q0 +
      q1 * q1 +
      q2 * q2 +
      q3 * q3
    )
  );
}

Quaternion Quaternion::scale(float32_t scaleFactor) {
  return Quaternion(
    q0 * scaleFactor,
    q1 * scaleFactor,
    q2 * scaleFactor,
    q3 * scaleFactor
  );
}

Quaternion Quaternion::add(Quaternion q) {
  return Quaternion(
    q0 + q.q0,
    q1 + q.q1,
    q2 + q.q2,
    q3 + q.q3
  );
}

Quaternion Quaternion::minus(Quaternion q) {
  return Quaternion(
    q0 - q.q0,
    q1 - q.q1,
    q2 - q.q2,
    q3 - q.q3
  );
}

Quaternion Quaternion::asUnit() {
  float32_t l = Math::fastInverseSquareRoot(
    q0 * q0 +
    q1 * q1 +
    q2 * q2 +
    q3 * q3
  );
  return Quaternion(
    q0 * l,
    q1 * l,
    q2 * l,
    q3 * l
  );
}

Quaternion  Quaternion::multiply(Quaternion q) {
  Quaternion r;

  r.q0 = q0 * q.q0 - q1 * q.q1 - q2 * q.q2 - q3 * q.q3;
  r.q1 = q0 * q.q1 + q1 * q.q0 + q2 * q.q3 - q3 * q.q2;
  r.q2 = q0 * q.q2 - q1 * q.q3 + q2 * q.q0 + q3 * q.q1;
  r.q3 = q0 * q.q3 + q1 * q.q2 - q2 * q.q1 + q3 * q.q0;

  return r;
}

Quaternion  Quaternion::conjugate() {
  return Quaternion(q0, -q1, -q2, -q3);
}

Vector Quaternion::toVector() {
  return Vector(q1, q2, q3);
}

Vector Quaternion::rotate(Vector v) {
  return this->multiply(v.toQuaternion()).multiply(this->conjugate()).toVector();
}

Quaternion  Quaternion::AngularRatesToQuaternion(Vector w, float32_t dt)
{

  float32_t wx = w.v0;
  float32_t wy = w.v1;
  float32_t wz = w.v2;

  float32_t angular_Rate_Norm = sqrt(wx * wx + wy * wy + wz * wz);
  float32_t trig_Input = dt * angular_Rate_Norm / 2;

  Quaternion r;

  r.q0 = cos(trig_Input);
  r.q1 = sin(trig_Input) * wx / angular_Rate_Norm;
  r.q2 = sin(trig_Input) * wy / angular_Rate_Norm;
  r.q3 = sin(trig_Input) * wz / angular_Rate_Norm;

  return r;
}
