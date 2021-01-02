#include "ChannelAligner.h"
#include "Math.h"
#include "EEPROM.h"
ChannelAligner::ChannelAligner() {}

void ChannelAligner::setup(
  IReceiverChannel* receiverChannel, int startAddress) {
    m_receiverChannel = receiverChannel;
    updateMinAndMax(startAddress);
}

void ChannelAligner::validateExtents() {
  constexpr int lowerLimitForCalibrationLogError = 800;
  constexpr int upperLimitForCalibrationLogError = 2200;

  if (m_min < lowerLimitForCalibrationLogError || m_max < lowerLimitForCalibrationLogError || m_mid < lowerLimitForCalibrationLogError ||
      m_min > upperLimitForCalibrationLogError || m_min > upperLimitForCalibrationLogError || m_min > upperLimitForCalibrationLogError) {	
    while (1) {
      Serial.println("recalibrate receiver endpoints not setup");
      delay(10000);
    }
  }

}

void ChannelAligner::updateMinAndMax(int startAddress) {	

  int eeAddress = startAddress;
  EEPROM.get(eeAddress, m_max);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, m_min);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, m_mid);

  validateExtents();
}	

float32_t ChannelAligner::getAlignedData(
  const float32_t startMapped,
  const float32_t midMapped,
  const float32_t endMapped) {

  uint16_t channelData = m_receiverChannel->getData();
  float32_t data = channelData < m_mid ?
    Math::mapfloat(channelData, m_min, m_mid, startMapped, midMapped) :
    Math::mapfloat(channelData, m_mid, m_max, midMapped, endMapped);

  return data;
}