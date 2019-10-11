#ifndef IIR_FILTER
#define IIR_FILTER

#include "CircularBuffer.h"
#include "arm_math.h"

class IIRFilter {
private:
  const int m_size;
  float32_t const * m_inputCoefficientsB;
  float32_t const * m_outputCoefficientsA;
  CircularBuffer* m_filterInX;
  CircularBuffer* m_filterOutY;


public:

  IIRFilter(const float32_t inputCoeffsB[], const float32_t outputCoeffsA[], const int size) : 
    m_size(size),
    m_inputCoefficientsB(inputCoeffsB),
    m_outputCoefficientsA(outputCoeffsA),
    m_filterInX(new CircularBuffer(size)),
    m_filterOutY(new CircularBuffer(size)) {

  }
  
  ~IIRFilter() {
    delete m_filterInX;
    delete m_filterOutY;
  }

  //  = {  0.0025851,   0.0155104,   0.0387760,   0.0517013,   0.0387760,   0.0155104,   0.0025851 };
  //  = {  1.000000,  -2.379721,   2.910407,  -2.055131,   0.877924,  -0.209865,   0.021832 };
  
  void update(float32_t input) {

    m_filterInX->add(input);
    
    float32_t sumOfXAndB = 0.0f;
    float32_t sumOfYAndA = 0.0f;

    // y(n)=b0x(n)+b1x(n−1)+⋯+bMx(n−M)−a1y(n−1)−⋯−aNy(n−N)

    for(int i = 1; i < m_size; ++i) {
      float32_t yNMinusI = m_filterOutY->get(-i + 1);
      sumOfYAndA += m_outputCoefficientsA[i] * yNMinusI;
    }

    for(int i = 0; i < m_size; ++i) {
      //because x is appended to at the begining it 
      //has different indices than the y output buffer
      float32_t xNMinusI = m_filterInX->get(-i);
      sumOfXAndB += m_inputCoefficientsB[i] * xNMinusI;
    }

    float32_t currentOutput = sumOfXAndB - sumOfYAndA;
    // Serial.print(" sumOfXAndB "); Serial.print(sumOfXAndB); Serial.print(" sumOfYAndA "); Serial.println(sumOfYAndA);
    
    // m_filterInX->printIndex();

    m_filterOutY->add(currentOutput);
  }

  float32_t getCurrent() {
    return m_filterOutY->get(0);
  }
};

#endif