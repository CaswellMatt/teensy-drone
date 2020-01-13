#include "GyroscopeFilter.h"

GyroscopeFilter::GyroscopeFilter() {
  const int filterSize = 7;

  m_inputCoefficientsBNotchFilter90hz = new float32_t[filterSize] {
    0.90775,	-4.6384,	10.62126,	-13.75821,	10.62126,	-4.6384,	0.90775,
  };

  m_outputCoefficientsANotchFilter90hz = new float32_t[filterSize] {
    1,	-4.94827,	10.9688,	-13.75039,	10.26914,	-4.33633,	0.82009,
  };

  m_ellipticalFilterNotchFilter90hz     
    = new IIRFilter(m_inputCoefficientsBNotchFilter90hz, m_outputCoefficientsANotchFilter90hz, filterSize);

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
  delete m_inputCoefficientsBNotchFilter90hz;
  delete m_outputCoefficientsANotchFilter90hz;

  delete m_inputCoefficientsBNotchFilter265hz;
  delete m_outputCoefficientsANotchFilter265hz;

  delete m_ellipticalFilterNotchFilter90hz;
  delete m_ellipticalFilterNotchFilter265hz;
}

float32_t GyroscopeFilter::get() {
  return m_ellipticalFilterNotchFilter265hz->getCurrent();
}

void GyroscopeFilter::update(float32_t input) {
  m_ellipticalFilterNotchFilter90hz->update(input);
  m_ellipticalFilterNotchFilter265hz->update(
    m_ellipticalFilterNotchFilter90hz->getCurrent());
}

float32_t GyroscopeFilter::getLow() {
  return m_ellipticalFilterLowFilter90hz->getCurrent();
}

void GyroscopeFilter::updateLow(float32_t input) {
  m_ellipticalFilterLowFilter90hz->update(input);
}