#ifndef IBUS
#define IBUS

#include <Arduino.h>

#define IBUS_BAUD_RATE 115200

#define SIZE_OF_IBUS_PACKET_BYTES 32
#define IBUS_BUFFER_SIZE 2 * SIZE_OF_IBUS_PACKET_BYTES
#define CHANNEL_COUNT 14


class IBus {
private:
  uint8_t m_rxBuffer[IBUS_BUFFER_SIZE];
  uint16_t m_channelData[CHANNEL_COUNT];
  uint16_t m_currentIndex;

public:
  void setup(void);
  bool read(void);
  uint16_t getChannelData(uint8_t channelId);

private:
  bool bufferHasEnoughDataForPacket(void);
  bool canFindPacketInBuffer(void);
  bool verifyChecksum(uint16_t startIndex);
  uint16_t checkIndex(uint16_t index);

};

#endif