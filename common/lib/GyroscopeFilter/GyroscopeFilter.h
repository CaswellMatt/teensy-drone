#ifndef GYROSCOPE_FILTER
#define GYROSCOPE_FILTER
#include "IIRFilter.h"
class GyroscopeFilter {
public:
  GyroscopeFilter();
  ~GyroscopeFilter();

  float32_t get();
  void update(float32_t input);

  void updateLow(float32_t input);
  float32_t getLow();

private:

  float32_t* m_inputCoefficientsBNotchFilter60hz;
  float32_t* m_outputCoefficientsANotchFilter60hz;

  float32_t* m_inputCoefficientsBNotchFilter265hz;
  float32_t* m_outputCoefficientsANotchFilter265hz;

  IIRFilter* m_ellipticalFilterNotchFilter60hz;
  IIRFilter* m_ellipticalFilterNotchFilter265hz;

  float32_t* m_inputCoefficientsBLowFilter90hz;
  float32_t* m_outputCoefficientsALowFilter90hz;

  IIRFilter* m_ellipticalFilterLowFilter90hz;

  
};
#endif