#ifndef RECEIVER_MANAGER
#define RECEIVER_MANAGER
#include <Arduino.h>
#include "ReceiverPulseTimer.h"
#include "MemoryLocations.h"
#include "Math.h"
#include "ReceiverAligner.h"
#include "IBus.h"
#include "IReceiverChannel.h"
#include "IBusChannel.h"

class ReceiverManager {

#ifndef CHANNEL_COUNT
#error require channel count
#endif

#define ROLL_CHANNEL_INDEX           1
#define PITCH_CHANNEL_INDEX          2
#define THROTTLE_CHANNEL_INDEX       3
#define YAW_CHANNEL_INDEX            4
#define TOP_LEFT_SWITCH_INDEX        5
#define TOP_RIGHT_SWITCH_INDEX       6

private:
  IBus m_iBus;
  IReceiverChannel* m_channels[CHANNEL_COUNT];
  IBusChannel m_rollChannel;
  IBusChannel m_pitchChannel;
  IBusChannel m_throttleChannel;
  IBusChannel m_yawChannel;
  IBusChannel m_topLeftSwitchChannel;
  IBusChannel m_topRightSwitchChannel;

public:
  ReceiverManager();
  void setup();
  uint16_t getRoll();
  uint16_t getPitch();
  uint16_t getThrottle();
  uint16_t getYaw();
  uint16_t getTopLeftSwitch();
  uint16_t getTopRightSwitch();
  bool isReceiving();

};

#endif
