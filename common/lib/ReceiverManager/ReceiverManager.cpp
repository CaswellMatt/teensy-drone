#include "ReceiverManager.h"
#include "EEPROM.h"

ReceiverManager::ReceiverManager() :
  m_channels(),
  m_aligners(),
  m_rollChannel(&m_iBus, ROLL_CHANNEL_INDEX),
  m_pitchChannel(&m_iBus, PITCH_CHANNEL_INDEX),
  m_throttleChannel(&m_iBus, THROTTLE_CHANNEL_INDEX),
  m_yawChannel(&m_iBus, YAW_CHANNEL_INDEX),
  m_topLeftSwitchChannel(&m_iBus, TOP_LEFT_SWITCH_INDEX),
  m_topRightSwitchChannel(&m_iBus, TOP_RIGHT_SWITCH_INDEX),
  m_rollAligner(),
  m_pitchAligner(),
  m_throttleAligner(),
  m_yawAligner() {

}

void ReceiverManager::setupAligners() {

  m_aligners[ROLL_CHANNEL_INDEX]     = &m_rollAligner;
  m_aligners[PITCH_CHANNEL_INDEX]    = &m_pitchAligner;
  m_aligners[THROTTLE_CHANNEL_INDEX] = &m_throttleAligner;
  m_aligners[YAW_CHANNEL_INDEX]      = &m_yawAligner;

  m_rollAligner.setup(&m_rollChannel, ROLL_START);	
  m_pitchAligner.setup(&m_pitchChannel, PITCH_START);
  m_throttleAligner.setup(&m_throttleChannel, THROTTLE_START);
  m_yawAligner.setup(&m_yawChannel, YAW_START);

}

void ReceiverManager::setup() {
  m_iBus.setup();

  m_channels[ROLL_CHANNEL_INDEX]     = &m_rollChannel;
  m_channels[PITCH_CHANNEL_INDEX]    = &m_pitchChannel;
  m_channels[THROTTLE_CHANNEL_INDEX] = &m_throttleChannel;
  m_channels[YAW_CHANNEL_INDEX]      = &m_yawChannel;
  m_channels[TOP_LEFT_SWITCH_INDEX]  = &m_topLeftSwitchChannel;
  m_channels[TOP_RIGHT_SWITCH_INDEX] = &m_topRightSwitchChannel;

}

bool ReceiverManager::read() {
  return m_iBus.read();
}

uint16_t ReceiverManager::getRoll() {
  return m_channels[ROLL_CHANNEL_INDEX]->getData();
}

uint16_t ReceiverManager::getPitch() {
  return m_channels[PITCH_CHANNEL_INDEX]->getData();
}

uint16_t ReceiverManager::getThrottle() {
  return m_channels[THROTTLE_CHANNEL_INDEX]->getData();
}

uint16_t ReceiverManager::getYaw() {
  return m_channels[YAW_CHANNEL_INDEX]->getData();
}

uint16_t ReceiverManager::getTopLeftSwitch() {
  return m_channels[TOP_LEFT_SWITCH_INDEX]->getData();
}

uint16_t ReceiverManager::getTopRightSwitch() {
  return m_channels[TOP_RIGHT_SWITCH_INDEX]->getData();
}

float32_t ReceiverManager::getRollAligned(    
  const float32_t startMapped,
  const float32_t midMapped,
  const float32_t endMapped) {
  return m_rollAligner.getAlignedData(startMapped, midMapped, endMapped);
}

float32_t ReceiverManager::getPitchAligned(
  const float32_t startMapped,
  const float32_t midMapped,
  const float32_t endMapped) {
  return m_pitchAligner.getAlignedData(startMapped, midMapped, endMapped);
}

float32_t ReceiverManager::getThrottleAligned(
  const float32_t startMapped,
  const float32_t midMapped,
  const float32_t endMapped) {
  return m_throttleAligner.getAlignedData(startMapped, midMapped, endMapped);
}

float32_t ReceiverManager::getYawAligned(
  const float32_t startMapped,
  const float32_t midMapped,
  const float32_t endMapped) {
  return m_yawAligner.getAlignedData(startMapped, midMapped, endMapped);
}

bool ReceiverManager::isReceiving() {
  return m_iBus.isReceiving();
}

ChannelAligner* ReceiverManager::getAligner(uint8_t alignerIndex) {
  return m_aligners[alignerIndex];
}

IReceiverChannel* ReceiverManager::getChannel(uint8_t channelIndex) {
  return m_channels[channelIndex];
}

void ReceiverManager::printAllChannels() {
  m_iBus.printAllChannels();
}