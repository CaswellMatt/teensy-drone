#include "Drone.h"
#include "ReceiverManager.h"
#include "Constants.h"
#include "PIDController.h"

namespace {
  float32_t anglularRateLimitForControlDegrees = 200;
  float32_t angularRateForControlRadiansStart  = -anglularRateLimitForControlDegrees * PI/ 180;
  float32_t angularRateForControlRadiansEnd    = anglularRateLimitForControlDegrees * PI/ 180;

  float32_t angularRateForControlRadiansMiddle = 
    (angularRateForControlRadiansStart + angularRateForControlRadiansEnd) / 2;

  float32_t frontLeftPulse  = throttleMapStart;
  float32_t frontRightPulse = throttleMapStart;
  float32_t backLeftPulse   = throttleMapStart;
  float32_t backRightPulse  = throttleMapStart;

  const bool ACCELERATION_FILTER_ON = true;
}

Drone::Drone() :
  m_marg(ACCELERATION_FILTER_ON),
  m_orientationFilter(&m_marg) {}  

Drone::~Drone() {
  ReceiverManager::destroyAligners();
}

void Drone::setup() {
  Serial.begin(1);
  // #define DEBUG
  #ifdef DEBUG
    while(!Serial);
  #endif

  ReceiverManager::setupReceivers();
  ReceiverManager::setupAligners();

  while(!ReceiverManager::isReceiving()) {
    delay(1000);
    // Serial.println(ReceiverManager::isReceiving());
  };

  MotorControlManager::setup();
  m_timer = micros();
}

bool Drone::motorsAreActive() {
  static bool isActive = false;

  float32_t lowerThrottleThreshold = throttleMapStart +  0.05 * (throttleMapEnd - throttleMapStart);

  bool throttleLowerThanThreshold = 
    ReceiverManager::throttleAligned->getPulseLength(throttleMapStart, 
                                                    throttleMapMiddle, 
                                                    throttleMapEnd) < lowerThrottleThreshold;

  if (throttleLowerThanThreshold) {
    
    float32_t lowerYawThreshold = 
      angularRateForControlRadiansStart +  0.04 * (angularRateForControlRadiansEnd - angularRateForControlRadiansStart);

    float32_t upperYawThreshold = 
      angularRateForControlRadiansStart +  0.96 * (angularRateForControlRadiansEnd - angularRateForControlRadiansStart);

    float32_t currentYawPulse = ReceiverManager::yawAligned->getPulseLength(angularRateForControlRadiansStart, 
                                                                            angularRateForControlRadiansMiddle, 
                                                                            angularRateForControlRadiansEnd);

    bool yawLowerThanThreshold = currentYawPulse <= lowerYawThreshold;
    bool yawAboveThreshold = currentYawPulse >= upperYawThreshold;
    if (yawLowerThanThreshold) {
      isActive = true;
    } else if (yawAboveThreshold) {
      isActive = false;
    }
                                                  
  }

  return isActive;
}

void Drone::start() {
  const float32_t outputLimit = 50;
  const float32_t integralLimit = 30;

  const float32_t Kp = 10;
  const float32_t Ki = 0.01;
  const float32_t Kd = 1;
  PIDController rollRotationalRateController(Kp, Ki, Kd, outputLimit, integralLimit);
  PIDController pitchRotationalRateController(Kp, Ki, Kd, outputLimit, integralLimit);
  PIDController yawRotationalRateController(Kp * 2, Ki, Kd, outputLimit, integralLimit);

  const float32_t KpAngle = 30;
  const float32_t KiAngle = 0.00;
  const float32_t KdAngle = 0;
  PIDController rollAngleController(KpAngle, KiAngle, KdAngle, outputLimit, integralLimit);
  PIDController pitchAngleController(KpAngle, KiAngle, KdAngle, outputLimit, integralLimit);
  
  while(1) {

    m_orientationFilter.update(LOOPTIME_S);
    if (motorsAreActive()) {

      auto pulseToControlSetpoint = [&](ReceiverAligner* aligner) {
        return aligner->getPulseLength(
          angularRateForControlRadiansStart, 
          angularRateForControlRadiansMiddle, 
          angularRateForControlRadiansEnd);
      };
      
      float32_t rollControlInput  = pulseToControlSetpoint(ReceiverManager::rollAligned);
      float32_t pitchControlInput = pulseToControlSetpoint(ReceiverManager::pitchAligned);
      float32_t yawControlInput   = pulseToControlSetpoint(ReceiverManager::yawAligned);

      float32_t rollRatePositiveRightSideDown =  m_marg.getRotationalRates().v1;
      //TODO: double check pitch
      float32_t pitchRatePositiveNoseDown     = -m_marg.getRotationalRates().v0;
      float32_t yawRatePositiveNoseLeft       =  -m_marg.getRotationalRates().v2;

      rollRotationalRateController.update(rollControlInput, rollRatePositiveRightSideDown);
      pitchRotationalRateController.update(pitchControlInput, pitchRatePositiveNoseDown);
      yawRotationalRateController.update(yawControlInput, yawRatePositiveNoseLeft);

      rollAngleController.update(0, m_orientationFilter.getRoll());
      pitchAngleController.update(0, m_orientationFilter.getPitch());

      // static long printTimer = micros();

      // if (micros() - printTimer > 2000) {
      //   printTimer = micros();
      //   Serial.print("pitch "); Serial.print(m_orientationFilter.getPitch(),5); Serial.print(" ");
      //   Serial.print("roll "); Serial.print(m_orientationFilter.getRoll(),5); Serial.print(" ");
      //   Serial.println();
      // }

      float32_t rollOutputPID  = rollRotationalRateController.getOutput() + rollAngleController.getOutput();
      float32_t pitchOutputPID = pitchRotationalRateController.getOutput() + pitchAngleController.getOutput();
      float32_t yawOutputPID   = yawRotationalRateController.getOutput();

      float32_t throttlePulse = 
        ReceiverManager::throttleAligned->getPulseLength(throttleMapStart, throttleMapMiddle, throttleMapEnd);

      frontLeftPulse  = throttlePulse + rollOutputPID - pitchOutputPID - yawOutputPID;
      frontRightPulse = throttlePulse - rollOutputPID - pitchOutputPID + yawOutputPID;
      backLeftPulse   = throttlePulse + rollOutputPID + pitchOutputPID + yawOutputPID;
      backRightPulse  = throttlePulse - rollOutputPID + pitchOutputPID - yawOutputPID;

      auto checkMinMaxOfPulse = [](float32_t& pulse) {
        float32_t min = throttleMapStart;
        if (pulse < min) {
          pulse = min;
        } else if (pulse > throttleMapEnd) {
          pulse = throttleMapEnd;
        }
      };

      checkMinMaxOfPulse(frontLeftPulse);
      checkMinMaxOfPulse(frontRightPulse);
      checkMinMaxOfPulse(backLeftPulse);
      checkMinMaxOfPulse(backRightPulse);

    } else {

      rollRotationalRateController.reset();
      pitchRotationalRateController.reset();
      yawRotationalRateController.reset();

      rollAngleController.reset();
      pitchAngleController.reset();

      frontLeftPulse  = throttleMapStart;
      frontRightPulse = throttleMapStart;
      backLeftPulse   = throttleMapStart;
      backRightPulse  = throttleMapStart;
    }

    while(micros() - m_timer < LOOPTIME_US);
    m_timer = micros();

    MotorControlManager::frontLeft.trigger(frontLeftPulse);
    MotorControlManager::frontRight.trigger(frontRightPulse);
    MotorControlManager::backLeft.trigger(backLeftPulse);
    MotorControlManager::backRight.trigger(backRightPulse);
  }

}