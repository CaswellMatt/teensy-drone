#ifndef RECEIVER_ALIGNER
#define RECEIVER_ALIGNER
#include "ReceiverPulseTimer.h"

#include "arm_math.h"

#include "Constants.h"

class ReceiverAligner {
private:
  ReceiverPulseTimer* m_timer;
  float32_t min;
  float32_t max;
  float32_t mid;

public:
  ReceiverAligner();

  void initialise(
    ReceiverPulseTimer* receiverTimer,
    float32_t minValue,
    float32_t maxValue,
    float32_t midValue);

  float32_t getPulseLength(
    const float32_t& startMapped,
    const float32_t& midMapped,
    const float32_t& endMapped);
  float32_t getPulseLengthAligned();
  void printTimer();
    
};

#endif