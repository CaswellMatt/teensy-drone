#ifndef PID_CONTROLLER
#define PID_CONTROLLER
#include "arm_math.h"

class PIDController {
public:
  PIDController(
    const float32_t proportionalConstant, 
    const float32_t integralConstant,
    const float32_t derivativeConstant,
    const float32_t outputLimit,
    const float32_t integralLimit);

  void update(float32_t setPoint, float32_t actual);
  
  void reset();
  
  float32_t getOutput();

private:
  const float32_t m_proportionalConstant;
  const float32_t m_integralConstant;
  const float32_t m_derivativeConstant;

  float32_t m_currentIntegral;
  float32_t m_previousError;

  const float32_t m_outputLimit;
  const float32_t m_integralLimit;

  float32_t m_output;
};

#endif