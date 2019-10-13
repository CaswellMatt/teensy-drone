#include "Drone.h"
#include "ReceiverManager.h"
#include "Constants.h"
#include "PIDController.h"

namespace {
  float32_t controlPulseStart  = -12.5;
  float32_t controlPulseEnd    = 12.5;
  float32_t controlPulseMiddle = (controlPulseStart + controlPulseEnd) / 2;

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
  // Serial.begin(1);
  // #define DEBUG
  // #ifdef DEBUG
  //   while(!Serial);
  // #endif

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
    
    float32_t lowerYawThreshold = controlPulseStart +  0.04 * (controlPulseEnd - controlPulseStart);

    float32_t upperYawThreshold = controlPulseStart +  0.96 * (controlPulseEnd - controlPulseStart);

    float32_t currentYawPulse = ReceiverManager::yawAligned->getPulseLength(controlPulseStart, 
                                                                            controlPulseMiddle, 
                                                                            controlPulseEnd);

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
  const float32_t Kp = 6;
  const float32_t Ki = 0.001;
  const float32_t Kd = 5;
  PIDController rollRotationalRateController(Kp, Ki, Kd);
  PIDController pitchRotationalRateController(Kp, Ki, Kd);
  PIDController yawRotationalRateController(Kp * 2, Ki, Kd);

  const float32_t KpAngle = 80;
  const float32_t KiAngle = 0.001;
  PIDController rollAngleController(KpAngle, KiAngle, 0);
  PIDController pitchAngleController(KpAngle, 0, 0);
  
  while(1) {

    m_orientationFilter.update(LOOPTIME_S);



    if (motorsAreActive()) {
      
      float32_t rollPulse =
        ReceiverManager::rollAligned->getPulseLength(controlPulseStart, controlPulseMiddle, controlPulseEnd);
      float32_t pitchPulse =
        ReceiverManager::pitchAligned->getPulseLength(controlPulseStart, controlPulseMiddle, controlPulseEnd);
      float32_t yawPulse =
        ReceiverManager::yawAligned->getPulseLength(controlPulseStart, controlPulseMiddle, controlPulseEnd);

      float32_t rollRatePositiveRightWingDown  =  m_marg.getRotationalRates().v1;
      float32_t pitchRatePositiveNoseDown      = -m_marg.getRotationalRates().v0;
      float32_t yawRatePositiveNoseLeft        =  m_marg.getRotationalRates().v2;

      rollRotationalRateController.update(rollPulse/4, rollRatePositiveRightWingDown);
      pitchRotationalRateController.update(pitchPulse/4, pitchRatePositiveNoseDown);
      yawRotationalRateController.update(yawPulse/4, yawRatePositiveNoseLeft);

      static long printTimer = micros();

      if (micros() - printTimer > 2000) {
        printTimer = micros();
        // Serial.print(m_marg.getRotationalRates().v0);Serial.print(" ");
        // Serial.print(rollRotationalRateController.getOutput()); Serial.print(" ");
        // Serial.print(rollPulse); Serial.print(" ");
        Serial.print("pitch ");Serial.print(m_marg.getRotationalRates().v0); Serial.print(" ");
        Serial.print("roll ");Serial.print(m_marg.getRotationalRates().v1); Serial.print(" ");
          Serial.print("yaw ");Serial.print(m_marg.getRotationalRates().v2); Serial.print(" ");
        Serial.println();
      }


      // rollAngleController.update(0, m_orientationFilter.getRoll());
      // pitchAngleController.update(0, m_orientationFilter.getPitch());

      float32_t rollOutputPID  = rollRotationalRateController.getOutput() + rollAngleController.getOutput();
      float32_t pitchOutputPID = pitchRotationalRateController.getOutput() + pitchAngleController.getOutput();
      float32_t yawOutputPID   = yawRotationalRateController.getOutput();

      float32_t throttlePulse = 
        ReceiverManager::throttleAligned->getPulseLength(throttleMapStart, throttleMapMiddle, throttleMapEnd);

      frontLeftPulse  = throttlePulse + rollOutputPID - pitchOutputPID + yawOutputPID;
      frontRightPulse = throttlePulse - rollOutputPID - pitchOutputPID - yawOutputPID;
      backLeftPulse   = throttlePulse + rollOutputPID + pitchOutputPID - yawOutputPID;
      backRightPulse  = throttlePulse - rollOutputPID + pitchOutputPID + yawOutputPID;

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