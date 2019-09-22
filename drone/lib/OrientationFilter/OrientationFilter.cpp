#include "OrientationFilter.h"

OrientationFilter::OrientationFilter() : 
	orientation(1, 0, 0, 0) {

}


void OrientationFilter::update(float32_t deltaT) {
  marg.read();

  Quaternion rotationalRates = marg.getRotationalRates().toQuaternion();

  Quaternion changeInOrientationForGyro
             = rotationalRates.multiply(orientation).scale(-0.5 * deltaT);

  Quaternion orientationGyro = orientation.add(changeInOrientationForGyro).asUnit();

  // rotate acceleration back relative to ground with conjugate
  Vector gravity = orientationGyro.conjugate().rotate(marg.getAcceleration());

  Quaternion orientationChangeAcceleration
             = calculateDeltaAccelerationQuaternion(gravity.asUnit());

  float32_t error = abs(marg.getAcceleration().magnitude() - g) / g;

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

  // Serial.println(error);

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

  bool magneticHasChanged = magneticsPrevious != marg.getMagnetics();
  if (magneticHasChanged) {

    float32_t error = abs(marg.getMagnetics().magnitude() - 1);

    Vector magneticNorth = orientationPrime.conjugate().rotate(marg.getMagnetics().asUnit());

    Quaternion orientationChangeMagnetics
              = calculateDeltaMagneticsQuaternion(magneticNorth);

  float32_t alpha = 0.3;
  float32_t alphaAdaptive = gainFactor(error, 0.1, 0.2) * alpha;
    Quaternion complimentaryOrientationForChangeInMagnetics
              = calculateComplementaryQuaternion(orientationChangeMagnetics, alphaAdaptive);


    orientation = orientationPrime.multiply(
      complimentaryOrientationForChangeInMagnetics
    ).asUnit();

  } else {
    orientation = orientationPrime;
  }


  // static long printTimer = micros();

  // if (micros() - printTimer > 100000) {
  //   printTimer = micros();

  // Serial.print(gain);;Serial.print(" ");
  //   Serial.print(magneticNorth.v0, 5);Serial.print(" ");
  //   Serial.print(magneticNorth.v1, 5);Serial.print(" ");
  //   Serial.print(magneticNorth.v2, 5);Serial.print(" ");

  //   Serial.print(complimentaryOrientationForChangeInAcceleration.q0, 5);Serial.print(" ");
  //   Serial.print(complimentaryOrientationForChangeInAcceleration.q1, 5);Serial.print(" ");
  //   Serial.print(complimentaryOrientationForChangeInAcceleration.q2, 5);Serial.print(" ");
  //   Serial.print(complimentaryOrientationForChangeInAcceleration.q3, 5);Serial.print(" ");

  //   Serial.print(marg.getAcceleration().v0, 5);Serial.print(" ");
  //   Serial.print(marg.getAcceleration().v1, 5);Serial.print(" ");
  //   Serial.print(marg.getAcceleration().v2, 5);Serial.print(" ");

  //   Serial.print(marg.getMagnetics().v0, 5);Serial.print(" ");
  //   Serial.print(marg.getMagnetics().v1, 5);Serial.print(" ");
  //   Serial.print(marg.getMagnetics().v2, 5);Serial.print(" ");

  //   Serial.print(marg.getRotationalRates().v0, 5);Serial.print(" ");
  //   Serial.print(marg.getRotationalRates().v1, 5);Serial.print(" ");
  //   Serial.print(marg.getRotationalRates().v2, 5);Serial.print(" ");

  // Serial.print(orientation.q0, 5);Serial.print(" ");
  // Serial.print(orientation.q1, 5);Serial.print(" ");
  // Serial.print(orientation.q2, 5);Serial.print(" ");
  // Serial.print(orientation.q3, 5);Serial.print(" ");

    // Serial.print(marg.getRotationalRates().v0, 5);Serial.print(" ");
    // Serial.print(marg.getRotationalRates().v1, 5);Serial.print(" ");
    // Serial.print(marg.getRotationalRates().v2, 5);Serial.print(" ");

    // Serial.print(marg.getAcceleration().v0, 5);Serial.print(" ");
    // Serial.print(marg.getAcceleration().v1, 5);Serial.print(" ");
    // Serial.print(marg.getAcceleration().v2, 5);Serial.print(" ");

    // Serial.print(marg.getMagnetics().v0, 5);Serial.print(" ");
    // Serial.print(marg.getMagnetics().v1, 5);Serial.print(" ");
    // Serial.print(marg.getMagnetics().v2, 5);Serial.print(" ");

    // Serial.print(getRoll());Serial.print(" ");
    // Serial.print(getPitch());Serial.print(" ");

    // Serial.println();
  // }

  magneticsPrevious = marg.getMagnetics();
}


float32_t OrientationFilter::getRoll() {
  Vector yRelativeToDrone = orientation.rotate(Vector(0, 1, 0));
  
  float32_t xSquaredPlusYSquared = yRelativeToDrone.v0 * yRelativeToDrone.v0 + yRelativeToDrone.v1 * yRelativeToDrone.v1;
  
  float32_t xAndY;
  arm_sqrt_f32(xSquaredPlusYSquared, &xAndY);
  
  float32_t roll = atan2(yRelativeToDrone.v2, xAndY);
  return roll;
}


float32_t OrientationFilter::getPitch() {
  Vector xRelativeToDrone = orientation.rotate(Vector(1, 0, 0));
  
  float32_t xSquaredPlusYSquared = xRelativeToDrone.v0 * xRelativeToDrone.v0 + xRelativeToDrone.v1 * xRelativeToDrone.v1;
  
  float32_t xAndY;
  arm_sqrt_f32(xSquaredPlusYSquared, &xAndY);

  float32_t pitch = atan2(xRelativeToDrone.v2, xAndY);
  return pitch;
}


float32_t OrientationFilter::getYaw() {
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
