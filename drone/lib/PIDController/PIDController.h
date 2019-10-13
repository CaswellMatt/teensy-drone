#ifndef PID_CONTROLLER
#define PID_CONTROLLER
#include "arm_math.h"

class PIDController {
public:
  PIDController(const float32_t proportionalConstant, 
                const float32_t integralConstant,
                const float32_t derivativeConstant);

  void update(float32_t setPoint, float32_t actual);
  
  float32_t getOutput();

private:
  const float32_t m_proportionalConstant;
  const float32_t m_integralConstant;
  const float32_t m_derivativeConstant;

  float32_t m_currentIntegral;
  float32_t m_previousError;

  float32_t m_output;
};

#endif