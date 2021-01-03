#ifndef RECEIVER_MANAGER
#define RECEIVER_MANAGER
#include <Arduino.h>
#include "MemoryLocations.h"
#include "Math.h"
#include "IBus.h"
#include "IReceiverChannel.h"
#include "ChannelAligner.h"
#include "IBusChannel.h"

#ifndef CHANNEL_COUNT
#error Require channel count, Should be defined by receiver protocol implementation
#endif

#define ROLL_CHANNEL_INDEX           0
#define PITCH_CHANNEL_INDEX          1
#define THROTTLE_CHANNEL_INDEX       2
#define YAW_CHANNEL_INDEX            3
#define TOP_LEFT_SWITCH_INDEX        4
#define TOP_RIGHT_SWITCH_INDEX       5

class ReceiverManager {

private:
  IBus m_iBus;
  IReceiverChannel* m_channels[CHANNEL_COUNT];
  ChannelAligner* m_aligners[CHANNEL_COUNT];
  IBusChannel m_rollChannel;
  IBusChannel m_pitchChannel;
  IBusChannel m_throttleChannel;
  IBusChannel m_yawChannel;
  IBusChannel m_topLeftSwitchChannel;
  IBusChannel m_topRightSwitchChannel;
  ChannelAligner m_rollAligner;
  ChannelAligner m_pitchAligner;
  ChannelAligner m_throttleAligner;
  ChannelAligner m_yawAligner;

public:
  ReceiverManager();
  void setup();
  void setupAligners();
  bool read();
  uint16_t getRoll();
  uint16_t getPitch();
  uint16_t getThrottle();
  uint16_t getYaw();
  uint16_t getTopLeftSwitch();
  uint16_t getTopRightSwitch();
  float32_t getRollAligned(
    const float32_t startMapped,
    const float32_t midMapped,
    const float32_t endMapped
  );
  float32_t getPitchAligned(
    const float32_t startMapped,
    const float32_t midMapped,
    const float32_t endMapped
  );
  float32_t getThrottleAligned(
    const float32_t startMapped,
    const float32_t midMapped,
    const float32_t endMapped
  );
  float32_t getYawAligned( 
    const float32_t startMapped,
    const float32_t midMapped,
    const float32_t endMapped
  );
  bool isReceiving();

  ChannelAligner* getAligner(uint8_t alignerIndex);
  IReceiverChannel* getChannel(uint8_t channelIndex);

  void printAllChannels();

};

#endif
