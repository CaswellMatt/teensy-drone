#include "AccelerometerFilter.h"

AccelerometerFilter::AccelerometerFilter() {
  const int filterSizeLowPass = 6;

  m_inputCoefficientsBLowPass = new float32_t[filterSizeLowPass] {
    0.053527,  -0.034868,   0.053584,   0.053584,  -0.034868,   0.053527
  };

  m_outputCoefficientsALowPass = new float32_t[filterSizeLowPass] {
    1.00000,  -3.02889,   4.82197,  -4.41558,   2.38256,  -0.61558
  };

  
  m_ellipticalLowPassFilter30hz     
    = new IIRFilter(m_inputCoefficientsBLowPass, m_outputCoefficientsALowPass, filterSizeLowPass);

}

AccelerometerFilter::~AccelerometerFilter() {
  delete m_ellipticalLowPassFilter30hz;

  delete m_inputCoefficientsBLowPass;
  delete m_outputCoefficientsALowPass;
}

float32_t AccelerometerFilter::get() {
  return m_ellipticalLowPassFilter30hz->getCurrent();
}

void AccelerometerFilter::update(float32_t input) {
  m_ellipticalLowPassFilter30hz->update(input);
}