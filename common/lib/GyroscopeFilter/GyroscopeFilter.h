#ifndef GYROSCOPE_FILTER
#define GYROSCOPE_FILTER
#include "IIRFilter.h"
class GyroscopeFilter {
public:
  GyroscopeFilter();
  ~GyroscopeFilter();

  float32_t get();
  void update(float32_t input);

private:

  float32_t* m_inputCoefficientsBNotchFilter90hz;
  float32_t* m_outputCoefficientsANotchFilter90hz;

  float32_t* m_inputCoefficientsBNotchFilter265hz;
  float32_t* m_outputCoefficientsANotchFilter265hz;

  IIRFilter* m_ellipticalLowPassFilterNotchFilter90hz;
  IIRFilter* m_ellipticalLowPassFilterNotchFilter265hz;
  
};
#endif