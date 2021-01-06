#include "GyroscopeFilter.h"

GyroscopeFilter::GyroscopeFilter() {
  const int filterSize = 7;

  m_inputCoefficientsBNotchFilter60hz = new float32_t[filterSize] {
    0.80285,	-4.50637,	10.83703,	-14.26563,	10.83703,	-4.50637,	0.80285,
  };

  m_outputCoefficientsANotchFilter60hz = new float32_t[filterSize] {
    1,	-5.21716,	11.64907,	-14.22139,	10.00145,	-3.83982,	0.62922,
  };

  m_ellipticalFilterNotchFilter60hz     
    = new IIRFilter(m_inputCoefficientsBNotchFilter60hz, m_outputCoefficientsANotchFilter60hz, filterSize);

  m_inputCoefficientsBNotchFilter265hz = new float32_t[filterSize] {
    0.78801,	0.26142,	2.3923,	0.52384,	2.3923,	0.26142,	0.78801,
  };

  m_outputCoefficientsANotchFilter265hz = new float32_t[filterSize] {
    1,	0.30735,	2.5894,	0.5223,	2.18131,	0.21703,	0.58991,
  };

  m_ellipticalFilterNotchFilter265hz     
    = new IIRFilter(m_inputCoefficientsBNotchFilter265hz, m_outputCoefficientsANotchFilter265hz, filterSize);


  const int filterSizeLowPass = 5;

  m_inputCoefficientsBLowFilter90hz = new float32_t[filterSizeLowPass] {
    0.097093,	-0.323142,	0.460314,	-0.323142,	0.097093,
  };

  m_outputCoefficientsALowFilter90hz = new float32_t[filterSizeLowPass] {
    1,	-3.50587,	4.77485,	-2.9772,	0.71743,
  };

  m_ellipticalFilterLowFilter90hz     
    = new IIRFilter(m_inputCoefficientsBLowFilter90hz, m_outputCoefficientsALowFilter90hz, filterSizeLowPass);

}

GyroscopeFilter::~GyroscopeFilter() {
  delete m_inputCoefficientsBNotchFilter60hz;
  delete m_outputCoefficientsANotchFilter60hz;

  delete m_inputCoefficientsBNotchFilter265hz;
  delete m_outputCoefficientsANotchFilter265hz;

  delete m_ellipticalFilterNotchFilter60hz;
  delete m_ellipticalFilterNotchFilter265hz;
}

float32_t GyroscopeFilter::get() {
  return m_ellipticalFilterNotchFilter60hz->getCurrent();
}

void GyroscopeFilter::update(float32_t input) {
  m_ellipticalFilterNotchFilter60hz->update(input);
  // m_ellipticalFilterNotchFilter265hz->update(
  //   m_ellipticalFilterNotchFilter60hz->getCurrent());
}

float32_t GyroscopeFilter::getLow() {
  return m_ellipticalFilterLowFilter90hz->getCurrent();
}

void GyroscopeFilter::updateLow(float32_t input) {
  m_ellipticalFilterLowFilter90hz->update(input);
}