#ifndef RECEIVER_ALIGNER
#define RECEIVER_ALIGNER
#include "ReceiverPulseTimer.h"
#include "arm_math.h"

#include "Constants.h"

class ReceiverAligner {

public:
  ReceiverAligner(ReceiverPulseTimer* receiverTimer, float32_t minValue, float32_t maxValue, float32_t midValue) : 
    timer(receiverTimer),
    min(minValue),
    max(maxValue),
    mid(midValue) {};

  ReceiverPulseTimer* timer;
  float32_t getPulseLength(const float32_t& startMapped, 
                           const float32_t& midMapped, 
                           const float32_t& endMapped) {
    float32_t pulse = timer->getPulseLength() < mid ? 
      Math::mapfloat(timer->getPulseLength(), min, mid, startMapped, midMapped) : 
      Math::mapfloat(timer->getPulseLength(), mid, max, midMapped, endMapped); 

    return pulse;
  }

  float32_t getPulseLength() {
    float32_t startMapped = throttleMapStart;
    float32_t endMapped   = throttleMapEnd;
    float32_t midMapped   = throttleMapMiddle;
    float32_t pulse = timer->getPulseLength() < mid ? 
      Math::mapfloat(timer->getPulseLength(), min, mid, startMapped, midMapped) : 
      Math::mapfloat(timer->getPulseLength(), mid, max, midMapped, endMapped); 

    return pulse;
  }

private:
  float32_t min;
  float32_t max;
  float32_t mid;
    
};

#endif