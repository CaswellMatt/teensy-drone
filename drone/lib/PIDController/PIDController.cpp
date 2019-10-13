#include "PIDController.h"

PIDController::PIDController(const float32_t proportionalConstant, 
              const float32_t integralConstant,
              const float32_t derivativeConstant) :
  m_proportionalConstant(proportionalConstant),
  m_integralConstant(integralConstant),
  m_derivativeConstant(derivativeConstant),
  m_currentIntegral(0),
  m_previousError(0),
  m_output(0) {

}

void PIDController::update(float32_t setPoint, float32_t actual) {
  float32_t error = setPoint - actual;
  float32_t proportional = error * m_proportionalConstant;
  m_currentIntegral += m_integralConstant * error;
  float32_t derivative = m_derivativeConstant * (error - m_previousError);

  m_previousError = error;
  
  const float32_t maximumIntegralValue = 10;
  
  if (m_currentIntegral > maximumIntegralValue) {
    m_currentIntegral = maximumIntegralValue;
  }

  m_output = proportional + m_currentIntegral + derivative;

  const float32_t maxOutput = 50;

  if (m_output > maxOutput) {
    m_output = maxOutput;
  }
  
  if (m_output < -maxOutput) {
    m_output = -maxOutput;
  }
}

float32_t PIDController::getOutput() {
  return m_output;
}
