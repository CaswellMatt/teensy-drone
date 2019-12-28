#include "CircularBuffer.h"

CircularBuffer::CircularBuffer(int arraySize) : 
size(arraySize), 
m_hasFilledArray(false),
m_currentIndex(0)
{
  data = new float32_t[size];

  for (int i = 0; i < size; ++i)
  {
    data[i] = 0;
  }
}

CircularBuffer::~CircularBuffer() { delete data; }

void CircularBuffer::add(float32_t value) {
  m_currentIndex++;

  if (m_currentIndex == size)
  {
    m_currentIndex %= size;
    if (!m_hasFilledArray)
    {
      m_hasFilledArray = true;
    }
  }

  data[m_currentIndex] = value;
}

float32_t CircularBuffer::get(int index) {
  int arrayValue = m_currentIndex + index;
  if (arrayValue >= size || arrayValue < 0)
    arrayValue %= size;

  bool arrayValueStillSmallerThanZero = arrayValue < 0;
  if (arrayValueStillSmallerThanZero)
    arrayValue += size;
  return data[arrayValue];
}

bool CircularBuffer::hasFilled() {
  return m_hasFilledArray;
}

void CircularBuffer::printIndex() {
  Serial.println(m_currentIndex);
}

void CircularBuffer::testBuffer() {
  CircularBuffer buffer(100);
  for (int i = 0; i < 100; ++i) {
    buffer.add(i);
  }

  Serial.println();

  for (int i = 1; i < 100; ++i)
  {
    Serial.println(buffer.get(-i + 1));
  }

  delay(20000);
}