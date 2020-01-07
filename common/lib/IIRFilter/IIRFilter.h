#ifndef IIR_FILTER
#define IIR_FILTER

#include "CircularBuffer.h"
#include "arm_math.h"

class IIRFilter {
private:
  const int m_size;
  float32_t const * m_inputCoefficientsB;
  float32_t const * m_outputCoefficientsA;
  CircularBuffer* m_accelerometerFilterInputsX;
  CircularBuffer* m_accelerometerFilterOutputsY;


public:

  IIRFilter(const float32_t inputCoeffsB[], const float32_t outputCoeffsA[], const int size) : 
    m_size(size),
    m_inputCoefficientsB(inputCoeffsB),
    m_outputCoefficientsA(outputCoeffsA),
    m_accelerometerFilterInputsX(new CircularBuffer(size)),
    m_accelerometerFilterOutputsY(new CircularBuffer(size)) {

  }
  
  ~IIRFilter() {
    delete m_accelerometerFilterInputsX;
    delete m_accelerometerFilterOutputsY;
  }

  void update(float32_t input) {

    m_accelerometerFilterInputsX->add(input);
    
    float32_t sumOfXAndB = 0.0f;
    float32_t sumOfYAndA = 0.0f;

    // y(n)=b0x(n)+b1x(n−1)+⋯+bMx(n−M)−a1y(n−1)−⋯−aNy(n−N)

    for(int i = 1; i < m_size; ++i) {
      float32_t yNMinusI = m_accelerometerFilterOutputsY->get(-i + 1);
      sumOfYAndA += m_outputCoefficientsA[i] * yNMinusI;
    }

    for(int i = 0; i < m_size; ++i) {
      //because x is appended to at the begining it 
      //has different indices than the y output buffer
      float32_t xNMinusI = m_accelerometerFilterInputsX->get(-i);
      sumOfXAndB += m_inputCoefficientsB[i] * xNMinusI;
    }

    float32_t currentOutput = sumOfXAndB - sumOfYAndA;
    // DEBUG_SERIAL.print(" sumOfXAndB "); DEBUG_SERIAL.print(sumOfXAndB); DEBUG_SERIAL.print(" sumOfYAndA "); DEBUG_SERIAL.println(sumOfYAndA);
    
    // m_accelerometerFilterInputsX->printIndex();

    m_accelerometerFilterOutputsY->add(currentOutput);
  }

  float32_t getCurrent() {
    return m_accelerometerFilterOutputsY->get(0);
  }
};

#endif