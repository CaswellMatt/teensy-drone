#ifndef RECEIVER_ALIGNER
#define RECEIVER_ALIGNER
#include <Arduino.h>
#include "arm_math.h"

#include "Constants.h"
#include "IReceiverChannel.h"

class ChannelAligner {
private:
  IReceiverChannel* m_receiverChannel;
  float32_t m_min;
  float32_t m_max;
  float32_t m_mid;

public:
  ChannelAligner();

  void setup(
    IReceiverChannel* receiverChannel,
    int startAddress);

  float32_t getAlignedData(
    const float32_t startMapped,
    const float32_t midMapped,
    const float32_t endMapped);


private:
  void validateExtents();
  void updateMinAndMax(int startAddress);
    
};

#endif