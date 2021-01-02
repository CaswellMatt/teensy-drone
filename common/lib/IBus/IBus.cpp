#include <Arduino.h>
#include "IBus.h"

#define IBUS_BAUD_RATE 115200

#define PROTOCOL_LENGTH 0x20 // 32 bytes
#define COMMAND_HEADER  0x40

#define SIZE_OF_IBUS_HEADER_BYTES 2
#define SIZE_OF_IBUS_CRC_BYTES 2
#define SIZE_OF_IBUS_CHECKSUM_BYTES 2
#define IBUS_PACKET_SIZE SIZE_OF_IBUS_PACKET_BYTES - SIZE_OF_IBUS_HEADER_BYTES

#define CHECKSUM_TOTAL 0xFFFF

bool IBus::bufferHasEnoughDataForPacket() {
  /* a whole packet will definitely exist if the buffer has double the amount of data required */
  return Serial2.available() >= IBUS_BUFFER_SIZE;
}

bool IBus::canFindPacketInBuffer(void) {
  bool foundPacket = false;
  /* search for oldest header in fifo as it should be the most complete */
  for (uint8_t i = 0; i < IBUS_BUFFER_SIZE - 1; ++i) {
    if (m_rxBuffer[i] == PROTOCOL_LENGTH) {
      if (m_rxBuffer[i + 1] == COMMAND_HEADER) {
        foundPacket = verifyChecksum(i + 2);
        if (foundPacket) {
          m_currentIndex = i + 2;
          checkIndex(m_currentIndex);
          break;
        }
      }
    }
  }

  return foundPacket;
}

bool IBus::verifyChecksum(uint16_t startIndex) {
  
  uint16_t indexOfFirstChecksumByte = startIndex + IBUS_PACKET_SIZE - 2;
  uint16_t indexOfSecondChecksumByte = startIndex + IBUS_PACKET_SIZE - 1;

  indexOfFirstChecksumByte = checkIndex(indexOfFirstChecksumByte);
  indexOfSecondChecksumByte = checkIndex(indexOfSecondChecksumByte);

  uint32_t checksum = m_rxBuffer[indexOfFirstChecksumByte] | (m_rxBuffer[indexOfSecondChecksumByte] << 8);
  uint32_t sum = 0;

  sum += COMMAND_HEADER;
  sum += PROTOCOL_LENGTH;
  
  for (uint16_t i = 0; i < IBUS_PACKET_SIZE - SIZE_OF_IBUS_CHECKSUM_BYTES; ++i) {
    uint16_t currentIndex = startIndex + i;

    currentIndex = checkIndex(currentIndex);

    sum += m_rxBuffer[currentIndex];
  }

  sum += checksum;

  return sum == CHECKSUM_TOTAL;
}

uint16_t IBus::checkIndex(uint16_t index) {
  if (index >= IBUS_BUFFER_SIZE) {
    return index - IBUS_BUFFER_SIZE;
  } else {
    return index;
  }
}

void IBus::setup(void) {
  Serial2.begin(IBUS_BAUD_RATE);
}

bool IBus::read(void) {
  bool channelsHaveBeenUpdated = false;
  if (bufferHasEnoughDataForPacket()) {
    Serial2.readBytes(m_rxBuffer, IBUS_BUFFER_SIZE);
    if (canFindPacketInBuffer()) {
      
      const uint8_t BYTES_PER_CHANNEL = 2;
      for (uint16_t i = 0; i < CHANNEL_COUNT; i++) {
        uint16_t currentBufferIndex = (m_currentIndex + (i * BYTES_PER_CHANNEL));
        currentBufferIndex = checkIndex(currentBufferIndex);
        m_channelData[i] = m_rxBuffer[currentBufferIndex] | (m_rxBuffer[currentBufferIndex + 1] << 8);
      }

      channelsHaveBeenUpdated = true;

    }
  }

  return channelsHaveBeenUpdated;
}

uint16_t IBus::getChannelData(uint8_t channelId) {
  return m_channelData[channelId];
}

bool IBus::isReceiving() {
  return Serial2.available();
}
