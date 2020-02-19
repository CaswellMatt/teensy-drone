#ifndef RECEIVER_ALIGNER
#define RECEIVER_ALIGNER
#include "ReceiverPulseTimer.h"
#include "arm_math.h"

#include "Constants.h"

class ReceiverAligner {

public:
  ReceiverAligner(ReceiverPulseTimer* receiverTimer, float32_t minValue, float32_t maxValue, float32_t midValue) : 
    m_timer(receiverTimer),
    min(minValue),
    max(maxValue),
    mid(midValue) {};

  float32_t getPulseLength(const float32_t& startMapped, 
                           const float32_t& midMapped, 
                           const float32_t& endMapped) {
    float32_t pulse = m_timer->getPulseLengthMicros() < mid ? 
      Math::mapfloat(m_timer->getPulseLengthMicros(), min, mid, startMapped, midMapped) : 
      Math::mapfloat(m_timer->getPulseLengthMicros(), mid, max, midMapped, endMapped); 

    return pulse;
  }

  float32_t getPulseLengthAligned() {
    float32_t startMapped = THROTTLE_MAP_START;
    float32_t endMapped   = THROTTLE_MAP_END;
    float32_t midMapped   = THROTTLE_MAP_MIDDLE;
    float32_t pulse = m_timer->getPulseLengthMicros() < mid ? 
      Math::mapfloat(m_timer->getPulseLengthMicros(), min, mid, startMapped, midMapped) : 
      Math::mapfloat(m_timer->getPulseLengthMicros(), mid, max, midMapped, endMapped); 

    return pulse;
  }

  float32_t printTimer() {
    Serial.println(m_timer->getPulseLengthMicros());
  }

private:
  ReceiverPulseTimer* m_timer;
  float32_t min;
  float32_t max;
  float32_t mid;
    
};

#endif