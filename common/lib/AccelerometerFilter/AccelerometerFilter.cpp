#include "AccelerometerFilter.h"

AccelerometerFilter::AccelerometerFilter() {
  const int filterSizeLowPass = 6;

  m_inputCoefficientsBLowPass = new float32_t[filterSizeLowPass] {
    0.00059831,	-0.0016424,	0.0010525,	0.0010525,	-0.0016424,	0.00059831,
  };

  m_outputCoefficientsALowPass = new float32_t[filterSizeLowPass] {
    1,	-4.86228,	9.50417,	-9.33446,	4.60614,	-0.91355,
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