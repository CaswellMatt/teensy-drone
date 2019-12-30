#ifndef CIRCULAR_BUFFER
#define CIRCULAR_BUFFER

#include <Arduino.h>
#include "arm_math.h"
#include "Constants.h"

class CircularBuffer {
private:
  int size;
  float32_t* data;
  bool m_hasFilledArray;
  int m_currentIndex;

public:
  CircularBuffer(int arraySize);
  ~CircularBuffer();

  void add(float32_t value);
  float32_t get(int index);

  bool hasFilled();
  void printIndex();
  void testBuffer();
};

#endif