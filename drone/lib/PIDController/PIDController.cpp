#include "PIDController.h"

PIDController::PIDController(
  const float32_t proportionalConstant, 
  const float32_t integralConstant,
  const float32_t derivativeConstant,
  const float32_t outputLimit,
  const float32_t integralLimit) :
    m_proportionalConstant(proportionalConstant),
    m_integralConstant(integralConstant),
    m_derivativeConstant(derivativeConstant),
    m_currentIntegral(0),
    m_previousError(0),
    m_outputLimit(outputLimit),
    m_integralLimit(integralLimit),
    m_output(0) {

}

void PIDController::update(float32_t setPoint, float32_t actual) {
  float32_t error = setPoint - actual;
  float32_t proportional = error * m_proportionalConstant;
  m_currentIntegral += m_integralConstant * error;
  float32_t derivative = m_derivativeConstant * (error - m_previousError);

  m_previousError = error;
  
  auto checkAndLimitValue = [](float32_t &value, float32_t limit) {
    if (value > limit) {
      value = limit;
    }

    if (value < -limit) {
      value = -limit;
    }
  };

  checkAndLimitValue(m_currentIntegral, m_integralLimit);
  
  m_output = proportional + m_currentIntegral + derivative;
  checkAndLimitValue(m_output, m_outputLimit);

}

void PIDController::reset()
{
  m_previousError   = 0;
  m_currentIntegral = 0;
}

float32_t PIDController::getOutput() {
  return m_output;
}
