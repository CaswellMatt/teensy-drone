#include "IBusChannel.h"

IBusChannel::IBusChannel(IBus* ibus, uint8_t channelId)
  : m_iBus(ibus),
    m_channelId(channelId) {

}

IBusChannel::IBusChannel() {

}

uint16_t IBusChannel::getData() {
  return m_iBus->getChannelData(m_channelId);
}

uint8_t IBusChannel::getId() {
  return m_channelId;
}
