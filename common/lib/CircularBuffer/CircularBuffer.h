#ifndef CIRCULAR_BUFFER
#define CIRCULAR_BUFFER

#include <Arduino.h>
#include "arm_math.h"

class CircularBuffer {
private:
  int size;
  float32_t* data;
  bool m_hasFilledArray;
  int m_currentIndex;

public:
  CircularBuffer(int arraySize) :
   size(arraySize), m_hasFilledArray(false), m_currentIndex(0) {
    data = new float32_t[size];

    for (int i = 0; i < size; ++i) {
      data[i] = 0;
    }
  }

  ~CircularBuffer() { delete data; }

  void add(float32_t value) {
    m_currentIndex++;

    if (m_currentIndex == size) {
      m_currentIndex %= size;
      if (!m_hasFilledArray) {
        m_hasFilledArray = true;
      }
    }

    data[m_currentIndex] = value;

  }

  float32_t get(int index) {
    int arrayValue = m_currentIndex + index;
    if (arrayValue >= size || arrayValue < 0) arrayValue %= size;
    
    bool arrayValueStillSmallerThanZero = arrayValue < 0;
    if (arrayValueStillSmallerThanZero) arrayValue += size;
    return data[arrayValue];
  }

  bool hasFilled() {
    return m_hasFilledArray;
  }

  void printIndex() {
    Serial.println(m_currentIndex);
  }

  void testBuffer() {
    CircularBuffer buffer(100);
    for (int i = 0; i < 100; ++i) {
      buffer.add(i);
    }

    Serial.println();

    for (int i = 1; i < 100; ++i) {
      Serial.println(buffer.get(-i+1));
    }

    delay(20000);
  }

};

#endif