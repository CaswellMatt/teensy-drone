#include "OrientationFilter.h"

namespace {
  Quaternion rotationalOffset = {1.0, 0.0, 0.0, 0.0};
}

OrientationFilter::OrientationFilter(MARG* marg) :
  m_marg(marg),
	m_orientation(1, 0, 0, 0) {

}

void OrientationFilter::update(float32_t deltaT) {
  m_marg->read();

  Quaternion rotationalRates = m_marg->getRotationalRates().toQuaternion();

  Quaternion changeInOrientationForGyro
             = rotationalRates.multiply(m_orientation).scale(-0.5 * deltaT);

  Quaternion orientationGyro = m_orientation.add(changeInOrientationForGyro).asUnit();

  // rotate acceleration back relative to ground with conjugate
  Vector gravity = orientationGyro.conjugate().rotate(m_marg->getAcceleration());

  Quaternion orientationChangeAcceleration
             = calculateDeltaAccelerationQuaternion(gravity.asUnit());

  float32_t error = abs(m_marg->getAcceleration().magnitude() - G) / G;

  // gain factor is linear between two bounds and reaches a minimum at the upper bound, otherwise constant
  auto gainFactor = [](float32_t error, float32_t lowerBound, float32_t upperBound) -> float32_t {
    if (error < lowerBound) {
      return error;
    } else if (error >= lowerBound && error < upperBound) {

      float32_t dy = 1;
      float32_t dx = upperBound - lowerBound;
      float32_t m = -dy/dx;
      float32_t c = -m * upperBound;
      float32_t y = m * error +  c;

      return y;

    } else {
      return 0;
    }
  };

  static float32_t previousGain = 0;
  float32_t gain = gainFactor(error, 0.1, 0.3);

  //check for error when in high acceleration movement
  //so you dont trust incorrect values if the previous value
  //was not trusted and has low gain
  if (gain > 0.6) {
   if (previousGain < 0.3) gain = 0;
  }
  previousGain = gain;

  
  float32_t alpha = 0.05;
  float32_t alphaAdaptive = gain * alpha;

  Quaternion complimentaryOrientationForChangeInAcceleration
             = calculateComplementaryQuaternion(orientationChangeAcceleration,
                                                alphaAdaptive);

  Quaternion orientationPrime = orientationGyro.multiply(
    complimentaryOrientationForChangeInAcceleration
  ).asUnit();

  bool magneticHasChanged = m_magneticsPrevious != m_marg->getMagnetics();
  static const bool USE_MAG = false;
  if (magneticHasChanged && USE_MAG) {

    float32_t error = abs(m_marg->getMagnetics().magnitude() - 1);

    Vector magneticNorth = orientationPrime.conjugate().rotate(m_marg->getMagnetics().asUnit());

    Quaternion orientationChangeMagnetics
              = calculateDeltaMagneticsQuaternion(magneticNorth);

  float32_t alpha = 0.3;
  float32_t alphaAdaptive = gainFactor(error, 0.1, 0.2) * alpha;
    Quaternion complimentaryOrientationForChangeInMagnetics
              = calculateComplementaryQuaternion(orientationChangeMagnetics, alphaAdaptive);


    m_orientation = orientationPrime.multiply(
      complimentaryOrientationForChangeInMagnetics
    ).asUnit();

  } else {
    m_orientation = orientationPrime;
  }

  m_magneticsPrevious = m_marg->getMagnetics();
  
  // debugPrint();
}

float32_t OrientationFilter::getRoll() {
  // roll (x-axis rotation)
  Quaternion q = this->m_orientation.multiply(rotationalOffset).conjugate();
  float32_t sinr_cosp = 2 * (q.q0 * q.q1 + q.q2 * q.q3);
  float32_t cosr_cosp = 1 - 2 * (q.q1 * q.q1 + q.q2 * q.q2);
  float32_t roll = atan2(sinr_cosp, cosr_cosp);
  return -roll;
}


float32_t OrientationFilter::getPitch() {
  // pitch (y-axis rotation)
  Quaternion q = this->m_orientation.multiply(rotationalOffset).conjugate();
  float32_t sinp = 2 * (q.q0 * q.q2 - q.q3 * q.q1);
  
  float32_t pitch;
  if (abs(sinp) >= 1) {
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  } else {
    pitch = asin(sinp);    
  }

  return -pitch;
}


float32_t OrientationFilter::getYaw() {
  // yaw (z-axis rotation)
  // double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  // double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  // angles.yaw = std::atan2(siny_cosp, cosy_cosp);
  return 0;
}

Quaternion OrientationFilter::calculateAccelerationQuaternion(Vector acceleration) {
  if (acceleration.v2 >= 0) {
    float32_t twoSqrtAzPlus1;
    arm_sqrt_f32(2*(acceleration.v2 + 1), &twoSqrtAzPlus1);

    return Quaternion(
      twoSqrtAzPlus1 / 2,
     -acceleration.v1 / twoSqrtAzPlus1,
      acceleration.v0 / twoSqrtAzPlus1,
      0
    );

  } else {

    float32_t twoSqrtOneMinusAz;
    arm_sqrt_f32(2*(1 - acceleration.v2), &twoSqrtOneMinusAz);

    return Quaternion(
     -acceleration.v1 / twoSqrtOneMinusAz,
      twoSqrtOneMinusAz / 2,
      0,
      acceleration.v0 / twoSqrtOneMinusAz
    );
  }
};

Quaternion OrientationFilter::calculateDeltaAccelerationQuaternion(Vector gravityUnit) {
  float32_t twoSqrtAzPlus1;
  arm_sqrt_f32(2*(gravityUnit.v2 + 1), &twoSqrtAzPlus1);

  return Quaternion(
    twoSqrtAzPlus1 / 2,
   -gravityUnit.v1 / twoSqrtAzPlus1,
    gravityUnit.v0 / twoSqrtAzPlus1,
    0
  );
};

Quaternion OrientationFilter::calculateDeltaMagneticsQuaternion(Vector magneticsUnit) {
  float32_t xSquaredPlusYSquared = magneticsUnit.v0 * magneticsUnit.v0 + magneticsUnit.v1 * magneticsUnit.v1;

  float32_t sqrtXSquaredPlusYSquared;
  arm_sqrt_f32(xSquaredPlusYSquared, &sqrtXSquaredPlusYSquared);

  float32_t l = 2 * (xSquaredPlusYSquared + magneticsUnit.v0 * sqrtXSquaredPlusYSquared);
  float32_t sqrtL;

  arm_sqrt_f32(l, &sqrtL);

  return Quaternion(
    sqrtL / (2 * sqrtXSquaredPlusYSquared),
    0,
    0,
    magneticsUnit.v1 / sqrtL
  );
};



Quaternion OrientationFilter::linearInterpolation(Quaternion q, float32_t alpha) {
  Quaternion identityQuaternion(1, 0, 0, 0);
  return identityQuaternion.scale(1 - alpha)
                            .add(q.scale(alpha))
                            .asUnit();
};

Quaternion OrientationFilter::sphericalInterpolation(Quaternion q, float32_t alpha) {
  // quaterion dot product = cos of angle between quaternion
  // and rotationless and is equal to first element
  // quaternion has one element the rest are 0
  float32_t sinTheta;
  arm_sqrt_f32(1 - q.q0 * q.q0, &sinTheta); //1 - cos^2 = sin^2

  float32_t angleBetweenQuaternions = asin(sinTheta);
  Quaternion identityQuaternion(sin((1 - alpha)*angleBetweenQuaternions)/sinTheta, 0, 0, 0);
  return identityQuaternion.add(
    q.scale(sin(alpha*angleBetweenQuaternions)/sinTheta)
  );

};

Quaternion OrientationFilter::calculateComplementaryQuaternion(Quaternion q, float32_t alpha) {
  if (q.q0 < 0.9) {
    return sphericalInterpolation(q, alpha);
  } else {
    return linearInterpolation(q, alpha);
  }
};

void OrientationFilter::debugPrint() {
  static long printTimer = micros();

  if (micros() - printTimer > 32000) {
    printTimer = micros();

  // DEBUG_SERIAL.print(gain);;DEBUG_SERIAL.print(" ");
  //   DEBUG_SERIAL.print(magneticNorth.v0, 5);DEBUG_SERIAL.print(" ");
  //   DEBUG_SERIAL.print(magneticNorth.v1, 5);DEBUG_SERIAL.print(" ");
  //   DEBUG_SERIAL.print(magneticNorth.v2, 5);DEBUG_SERIAL.print(" ");

  //   DEBUG_SERIAL.print(complimentaryOrientationForChangeInAcceleration.q0, 5);DEBUG_SERIAL.print(" ");
  //   DEBUG_SERIAL.print(complimentaryOrientationForChangeInAcceleration.q1, 5);DEBUG_SERIAL.print(" ");
  //   DEBUG_SERIAL.print(complimentaryOrientationForChangeInAcceleration.q2, 5);DEBUG_SERIAL.print(" ");
  //   DEBUG_SERIAL.print(complimentaryOrientationForChangeInAcceleration.q3, 5);DEBUG_SERIAL.print(" ");

    // DEBUG_SERIAL.print(m_marg->getAcceleration().v0, 5);DEBUG_SERIAL.print(" ");
    // DEBUG_SERIAL.print(m_marg->getAcceleration().v1, 5);DEBUG_SERIAL.print(" ");
    // DEBUG_SERIAL.print(m_marg->getAcceleration().v2, 5);DEBUG_SERIAL.print(" ");

    // DEBUG_SERIAL.print(m_marg->getMagnetics().v0, 5);DEBUG_SERIAL.print(" ");
    // DEBUG_SERIAL.print(m_marg->getMagnetics().v1, 5);DEBUG_SERIAL.print(" ");
    // DEBUG_SERIAL.print(m_marg->getMagnetics().v2, 5);DEBUG_SERIAL.print(" ");

    // DEBUG_SERIAL.print(m_marg->getRotationalRates().v0, 5);DEBUG_SERIAL.print(" ");
    // DEBUG_SERIAL.print(m_marg->getRotationalRates().v1, 5);DEBUG_SERIAL.print(" ");
    // DEBUG_SERIAL.print(m_marg->getRotationalRates().v2, 5);DEBUG_SERIAL.print(" ");

    DEBUG_SERIAL.print(m_orientation.q0, 5);DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print(m_orientation.q1, 5);DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print(m_orientation.q2, 5);DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print(m_orientation.q3, 5);DEBUG_SERIAL.println();

    // DEBUG_SERIAL.print(m_marg->getRotationalRates().v0, 5);DEBUG_SERIAL.print(" ");
    // DEBUG_SERIAL.print(m_marg->getRotationalRates().v1, 5);DEBUG_SERIAL.print(" ");
    // DEBUG_SERIAL.print(m_marg->getRotationalRates().v2, 5);DEBUG_SERIAL.print(" ");

    // DEBUG_SERIAL.print(m_marg->getAcceleration().v0, 5);DEBUG_SERIAL.print(" ");
    // DEBUG_SERIAL.print(m_marg->getAcceleration().v1, 5);DEBUG_SERIAL.print(" ");
    // DEBUG_SERIAL.print(m_marg->getAcceleration().v2, 5);DEBUG_SERIAL.print(" ");

    // DEBUG_SERIAL.print(m_marg->getMagnetics().v0, 5);DEBUG_SERIAL.print(" ");
    // DEBUG_SERIAL.print(m_marg->getMagnetics().v1, 5);DEBUG_SERIAL.print(" ");
    // DEBUG_SERIAL.print(m_marg->getMagnetics().v2, 5);DEBUG_SERIAL.print(" ");

    // DEBUG_SERIAL.print(getRoll());DEBUG_SERIAL.print(" ");
    // DEBUG_SERIAL.print(getPitch());DEBUG_SERIAL.print(" ");

    // DEBUG_SERIAL.println();
  }
}