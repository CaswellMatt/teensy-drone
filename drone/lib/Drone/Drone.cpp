#include "Drone.h"
#include "ReceiverManager.h"
#include "Constants.h"
#include "PIDController.h"
#include "DShotMotorController.h"

namespace {

  float32_t anglularRateLimitForControlDegrees = 200;
  float32_t angularRateForControlRadiansStart  = -anglularRateLimitForControlDegrees * DEG_TO_RAD;
  float32_t angularRateForControlRadiansEnd    = anglularRateLimitForControlDegrees * DEG_TO_RAD;

  float32_t angularRateForControlRadiansMiddle = 
    (angularRateForControlRadiansStart + angularRateForControlRadiansEnd) / 2;

  float32_t frontLeftPulse  = THROTTLE_MAP_START;
  float32_t frontRightPulse = THROTTLE_MAP_START;
  float32_t backLeftPulse   = THROTTLE_MAP_START;
  float32_t backRightPulse  = THROTTLE_MAP_START;

  const bool ACCELERATION_FILTER_ON = true;
  const bool GYRO_FILTER_ON = true;

  const int MIDPOINT_FOR_RECEIVER_PULSE = 1500;

}

Drone::Drone() :
  m_marg(ACCELERATION_FILTER_ON, GYRO_FILTER_ON),
  m_orientationFilter(&m_marg) {}  

Drone::~Drone() {}

DShotMotorController m_motorController;

void Drone::setup() {
  #ifdef BLUETOOTH
    DEBUG_SERIAL.begin(230400);
  #else 
    DEBUG_SERIAL.begin(1);
  #endif

  // #define DEBUG
  #ifdef DEBUG
    while(!Serial);
  #endif

  m_motorController.setup();
  m_receiverManager.setup();
  m_receiverManager.setupAligners();

  while(!m_receiverManager.isReceiving()) {
    DEBUG_SERIAL.println("Waiting for receiver to start");
    delay(1000);
  };
  
  m_timer = micros();
}

bool Drone::motorsAreActive() {
  static bool isActive = false;
  static bool yawTriggerTripped = false;

  float32_t lowerThrottleThreshold = THROTTLE_MAP_START + 0.05 * (THROTTLE_MAP_END - THROTTLE_MAP_START);

  bool throttleLowerThanThreshold = 
    m_receiverManager.getThrottleAligned(
      THROTTLE_MAP_START, 
      THROTTLE_MAP_MIDDLE,
      THROTTLE_MAP_END) < lowerThrottleThreshold;
  bool armMotorsSwitchIsOn = m_receiverManager.getTopLeftSwitch() > MIDPOINT_FOR_RECEIVER_PULSE;
  if (armMotorsSwitchIsOn) {
    if (throttleLowerThanThreshold) {
      
      const float32_t lowerYawThreshold = 
        angularRateForControlRadiansStart +  0.04 * (angularRateForControlRadiansEnd - angularRateForControlRadiansStart);

      const float32_t middleYawThreshold = 
        angularRateForControlRadiansStart +  0.50 * (angularRateForControlRadiansEnd - angularRateForControlRadiansStart);

      const float32_t upperYawThreshold = 
        angularRateForControlRadiansStart +  0.96 * (angularRateForControlRadiansEnd - angularRateForControlRadiansStart);

      float32_t currentYaw = // m_receiverManager.getYaw();
        m_receiverManager.getYawAligned(
          angularRateForControlRadiansStart,
          angularRateForControlRadiansMiddle,
          angularRateForControlRadiansEnd);
      Serial.println(currentYaw, 4);
      bool yawLowerThanThreshold = currentYaw <= lowerYawThreshold;
      bool yawAboveThreshold = currentYaw >= upperYawThreshold;
      if (yawLowerThanThreshold) {
        yawTriggerTripped = true;
      } else if (yawAboveThreshold) {
        isActive = false;
      }


      if (yawTriggerTripped) {
        float32_t differenceBetweenYawAndCentrePulse = currentYaw - middleYawThreshold;
        float32_t absoluteDifferenceBetweenYawAndCentrePulse;
        arm_abs_f32(
          &differenceBetweenYawAndCentrePulse, 
          &absoluteDifferenceBetweenYawAndCentrePulse, 
          1
        );
        
        static const float32_t thresholdForComparison = 0.1;
        if (absoluteDifferenceBetweenYawAndCentrePulse < thresholdForComparison) {
          isActive = true;
          yawTriggerTripped = false;
        }
      }
    }
  } else {
    isActive = false;
  }
  
  return isActive;
}


#define DEBUG_PRINT_START \
  static long printTimer = micros(); \
  if (micros() - printTimer > 1000) { \
    printTimer = micros(); \

#define DEBUG_PRINT_END \
    DEBUG_SERIAL.println(); \
  } \

void Drone::start() {
  
  constexpr float32_t factor = 10;
  constexpr float32_t outputLimit = 75 * factor;
  constexpr float32_t integralLimit = 75 * factor;
  static_assert(outputLimit >= integralLimit, "cannot have and output limit for pid less than integral limit");

  constexpr float32_t Kp = 30 * factor;
  constexpr float32_t Ki = 0.003 * factor;
  constexpr float32_t Kd = 3 * factor;

  PIDController rollRotationalRateController (Kp, Ki, Kd, outputLimit, integralLimit);
  PIDController pitchRotationalRateController(Kp, Ki, Kd, outputLimit, integralLimit);
  PIDController yawRotationalRateController  (Kp, Ki, Kd, outputLimit, integralLimit);

  constexpr float32_t KpAngle = 50 * factor;
  constexpr float32_t KiAngle = 0.005 * factor;
  constexpr float32_t KdAngle = 300 * factor;
  
  PIDController rollAngleController (KpAngle, KiAngle, KdAngle, outputLimit, integralLimit);
  PIDController pitchAngleController(KpAngle, KiAngle, KdAngle, outputLimit, integralLimit);

  float32_t previousRollControlInput  = 0;
  float32_t previousPitchControlInput = 0;
  float32_t previousYawControlInput   = 0;

  float32_t rollControlInput  = 0;
  float32_t pitchControlInput = 0;
  float32_t yawControlInput   = 0;

  while(1) {

    if (m_receiverManager.read()) {
      m_orientationFilter.update(LOOPTIME_S);
      // debugPrint();

      if (motorsAreActive()) {

        Serial.println("motors are active");

        rollControlInput  = pulseToRadiansPerSecondControlInput(m_receiverManager.getAligner(ROLL_CHANNEL_INDEX));
        pitchControlInput = pulseToRadiansPerSecondControlInput(m_receiverManager.getAligner(PITCH_CHANNEL_INDEX));
        yawControlInput   = pulseToRadiansPerSecondControlInput(m_receiverManager.getAligner(YAW_CHANNEL_INDEX));

        if (rollControlInput  > angularRateForControlRadiansEnd) rollControlInput  = previousRollControlInput;
        if (pitchControlInput > angularRateForControlRadiansEnd) pitchControlInput = previousPitchControlInput;
        if (yawControlInput   > angularRateForControlRadiansEnd) yawControlInput   = previousYawControlInput;

        previousRollControlInput  = rollControlInput;
        previousPitchControlInput = pitchControlInput;
        previousYawControlInput   = yawControlInput;

        float32_t rollRatePositiveRightSideDown = m_marg.getRotationalRates().v0;
        float32_t pitchRatePositiveNoseDown     = m_marg.getRotationalRates().v1;
        float32_t yawRatePositiveNoseLeft       = m_marg.getRotationalRates().v2;

        rollRotationalRateController.update(rollControlInput  , rollRatePositiveRightSideDown);
        pitchRotationalRateController.update(pitchControlInput, pitchRatePositiveNoseDown);
        yawRotationalRateController.update(yawControlInput    , yawRatePositiveNoseLeft);
        
        bool shouldAutoLevel = m_receiverManager.getTopRightSwitch() < MIDPOINT_FOR_RECEIVER_PULSE;
        if (shouldAutoLevel) {
          rollAngleController.update(0, m_orientationFilter.getRoll());
          pitchAngleController.update(0, m_orientationFilter.getPitch());
        } else {
          rollAngleController.reset();
          pitchAngleController.reset();
        }

        float32_t rollOutputPID  = rollRotationalRateController.getOutput()  + rollAngleController.getOutput();
        float32_t pitchOutputPID = pitchRotationalRateController.getOutput() + pitchAngleController.getOutput();
        float32_t yawOutputPID   = yawRotationalRateController.getOutput();

        static constexpr float32_t smallChangeToKeepMotorsSpinning = 128;
        static constexpr float32_t throttleToKeepMotorsSpinning = THROTTLE_MAP_START + smallChangeToKeepMotorsSpinning;

        float32_t throttlePulse = m_receiverManager.getThrottleAligned(throttleToKeepMotorsSpinning, THROTTLE_MAP_MIDDLE, THROTTLE_MAP_END);

        frontLeftPulse  = throttlePulse + rollOutputPID - pitchOutputPID + yawOutputPID;
        frontRightPulse = throttlePulse - rollOutputPID - pitchOutputPID - yawOutputPID;
        backLeftPulse   = throttlePulse + rollOutputPID + pitchOutputPID - yawOutputPID;
        backRightPulse  = throttlePulse - rollOutputPID + pitchOutputPID + yawOutputPID;

        auto checkMinMaxOfPulse = [](float32_t& pulse) {
          float32_t min = THROTTLE_MAP_START;
          if (pulse < min) {
            pulse = min;
          } else if (pulse > THROTTLE_MAP_END) {
            pulse = THROTTLE_MAP_END;
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

        frontLeftPulse  = THROTTLE_MAP_START;
        frontRightPulse = THROTTLE_MAP_START;
        backLeftPulse   = THROTTLE_MAP_START;
        backRightPulse  = THROTTLE_MAP_START;
      }
    } else {
      rollRotationalRateController.reset();
      pitchRotationalRateController.reset();
      yawRotationalRateController.reset();

      rollAngleController.reset();
      pitchAngleController.reset();

      frontLeftPulse  = THROTTLE_MAP_START;
      frontRightPulse = THROTTLE_MAP_START;
      backLeftPulse   = THROTTLE_MAP_START;
      backRightPulse  = THROTTLE_MAP_START;
    }

    while(micros() - m_timer < LOOPTIME_US);
    m_timer = micros();
    m_motorController.setSpeed(frontLeftPulse, frontRightPulse, backLeftPulse, backRightPulse);
  }

}

void Drone::debugPrint() {
  static long printTimer = micros();
  if (micros() - printTimer > 30000) {
    printTimer = micros();

    // DEBUG_SERIAL.print(m_orientationFilter.getPitch(),5); DEBUG_SERIAL.print(" ");
    // DEBUG_SERIAL.print(m_orientationFilter.getRoll(),5); DEBUG_SERIAL.print(" ");

    // Serial.print(m_marg.getRotationalRates().v0,5); DEBUG_SERIAL.print(" ");
    // Serial.print(m_marg.getRotationalRates().v1,5); DEBUG_SERIAL.print(" ");
    // Serial.print(m_marg.getRotationalRates().v2,5); DEBUG_SERIAL.print(" ");

    // Serial.print(m_marg.getAcceleration().v0,5); DEBUG_SERIAL.print(" ");
    // Serial.print(m_marg.getAcceleration().v1,5); DEBUG_SERIAL.print(" ");
    // Serial.print(m_marg.getAcceleration().v2,5); DEBUG_SERIAL.print(" ");

    // DEBUG_SERIAL.print(pitchAngleController.getDerivative(),5); DEBUG_SERIAL.print(" ");
    // DEBUG_SERIAL.print(pitchAngleController.getProportional(),5); DEBUG_SERIAL.print(" ");

    DEBUG_SERIAL.print((uint16_t)m_receiverManager.getRoll()); DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print(m_receiverManager.getPitch()); DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print(m_receiverManager.getYaw()); DEBUG_SERIAL.print(" ");



    DEBUG_SERIAL.println();
  }
}

float32_t Drone::pulseToRadiansPerSecondControlInput(ChannelAligner* aligner) {
  return aligner->getAlignedData(
    angularRateForControlRadiansStart, 
    angularRateForControlRadiansMiddle, 
    angularRateForControlRadiansEnd);
}
