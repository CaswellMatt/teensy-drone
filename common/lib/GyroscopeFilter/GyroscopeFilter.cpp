#include "GyroscopeFilter.h"

GyroscopeFilter::GyroscopeFilter() {
  const int filterSizeLowPass = 7;

  m_inputCoefficientsBNotchFilter90hz = new float32_t[filterSizeLowPass] {
    0.90775,	-4.6384,	10.62126,	-13.75821,	10.62126,	-4.6384,	0.90775,
  };

  m_outputCoefficientsANotchFilter90hz = new float32_t[filterSizeLowPass] {
    1,	-4.94827,	10.9688,	-13.75039,	10.26914,	-4.33633,	0.82009,
  };

  m_ellipticalLowPassFilterNotchFilter90hz     
    = new IIRFilter(m_inputCoefficientsBNotchFilter90hz, m_outputCoefficientsANotchFilter90hz, filterSizeLowPass);

  m_inputCoefficientsBNotchFilter265hz = new float32_t[filterSizeLowPass] {
    0.78801,	0.26142,	2.3923,	0.52384,	2.3923,	0.26142,	0.78801,
  };

  m_outputCoefficientsANotchFilter265hz = new float32_t[filterSizeLowPass] {
    1,	0.30735,	2.5894,	0.5223,	2.18131,	0.21703,	0.58991,
  };

  m_ellipticalLowPassFilterNotchFilter265hz     
    = new IIRFilter(m_inputCoefficientsBNotchFilter265hz, m_outputCoefficientsANotchFilter265hz, filterSizeLowPass);

}

GyroscopeFilter::~GyroscopeFilter() {
  delete m_inputCoefficientsBNotchFilter90hz;
  delete m_outputCoefficientsANotchFilter90hz;

  delete m_inputCoefficientsBNotchFilter265hz;
  delete m_outputCoefficientsANotchFilter265hz;

  delete m_ellipticalLowPassFilterNotchFilter90hz;
  delete m_ellipticalLowPassFilterNotchFilter265hz;
}

float32_t GyroscopeFilter::get() {
  return m_ellipticalLowPassFilterNotchFilter265hz->getCurrent();
}

void GyroscopeFilter::update(float32_t input) {
  m_ellipticalLowPassFilterNotchFilter90hz->update(input);
  m_ellipticalLowPassFilterNotchFilter265hz->update(
    m_ellipticalLowPassFilterNotchFilter90hz->getCurrent());
}