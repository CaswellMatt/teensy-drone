#include "ReceiverManager.h"

ReceiverManager::ReceiverManager() :
  m_rollChannel(&m_iBus, ROLL_CHANNEL_INDEX),
  m_pitchChannel(&m_iBus, PITCH_CHANNEL_INDEX),
  m_throttleChannel(&m_iBus, THROTTLE_CHANNEL_INDEX),
  m_yawChannel(&m_iBus, YAW_CHANNEL_INDEX),
  m_topLeftSwitchChannel(&m_iBus, TOP_LEFT_SWITCH_INDEX),
  m_topRightSwitchChannel(&m_iBus, TOP_RIGHT_SWITCH_INDEX)
  {

}

void ReceiverManager::setup() {
  m_channels[ROLL_CHANNEL_INDEX]     = &m_rollChannel;
  m_channels[PITCH_CHANNEL_INDEX]    = &m_pitchChannel;
  m_channels[THROTTLE_CHANNEL_INDEX] = &m_throttleChannel;
  m_channels[YAW_CHANNEL_INDEX]      = &m_yawChannel;
  m_channels[TOP_LEFT_SWITCH_INDEX]  = &m_topLeftSwitchChannel;
  m_channels[TOP_RIGHT_SWITCH_INDEX] = &m_topRightSwitchChannel;
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

bool ReceiverManager::isReceiving() {
  return m_iBus.isReceiving();
}
