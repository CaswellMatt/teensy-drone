#ifndef ACCELEROMETER_FILTER
#define ACCELEROMETER_FILTER
#include "IIRFilter.h"
class AccelerometerFilter {
public:
  AccelerometerFilter();
  ~AccelerometerFilter();

  float32_t get();
  void update(float32_t input);

private:

  float32_t* m_inputCoefficientsBLowPass;
  float32_t* m_outputCoefficientsALowPass;

  IIRFilter* m_ellipticalLowPassFilter30hz;

  
};
#endif