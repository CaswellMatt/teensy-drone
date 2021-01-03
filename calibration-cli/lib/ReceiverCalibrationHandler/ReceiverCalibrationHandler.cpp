#include "ReceiverCalibrationHandler.h"
#include <Arduino.h>
#include "ReceiverManager.h"

ReceiverManager receiverManager;

#include <arm_math.h>

const String rollOptionText = "Calibrate Roll";
const String pitchOptionText = "Calibrate Pitch";
const String yawOptionText = "Calibrate Yaw";
const String throttleOptionText = "Calibrate Throttle";
const String printOptionText = "Print Receiver Values";
const String printSavedValuesText = "Print Saved Receiver Endpoints";

ReceiverCalibrationHandler::ReceiverCalibrationHandler() {
  m_message = String("Calibrate Receiver");
}

void ReceiverCalibrationHandler::printTitle() {
  DEBUG_SERIAL.println("Receiver Calibration Menu");
}

void ReceiverCalibrationHandler::setup() {

  receiverManager.setup();

  while (!receiverManager.isReceiving());

  addExit(this);

  addOption(this, &ReceiverCalibrationHandler::rollCalibrator    , rollOptionText);
  addOption(this, &ReceiverCalibrationHandler::pitchCalibrator   , pitchOptionText);
  addOption(this, &ReceiverCalibrationHandler::yawCalibrator     , yawOptionText);
  addOption(this, &ReceiverCalibrationHandler::throttleCalibrator, throttleOptionText);
  addOption(this, &ReceiverCalibrationHandler::printAll          , printOptionText);
  addOption(this, &ReceiverCalibrationHandler::printSaved        , printSavedValuesText);

}

void ReceiverCalibrationHandler::printReceiver(String receiverName, int startAddress) {

  float32_t max;
  float32_t min;
  float32_t mid;

  int eeAddress = startAddress;
  EEPROM.get(eeAddress, max);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, min);

  eeAddress += sizeof(float32_t);
  EEPROM.get(eeAddress, mid);

  DEBUG_SERIAL.print(receiverName); DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print("Min "); DEBUG_SERIAL.print(min); DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print("Max "); DEBUG_SERIAL.print(max); DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print("Mid "); DEBUG_SERIAL.print(mid); DEBUG_SERIAL.println();
};


const int RECEIVER_PULSE_COUNT_TO_AVERAGE = 100;

void ReceiverCalibrationHandler::calibrateReceiver(IReceiverChannel* channel, int eepromStartAddress) {
  DEBUG_SERIAL.println("Lets collect the max for this receiver channel");
  DEBUG_SERIAL.println("Move the stick to the max position");
  delay(5000);

  auto getAveragePulseLengthForThisTimer = [](const IReceiverChannel& channel, int size) {
    long sumOf = 0;

    for (int i = 0; i < size; ++i) {
      while (!receiverManager.read());
      sumOf += channel.getData();
      delay(2);
    }

    float32_t averagePulseLengthMax = static_cast<float32_t>(sumOf / size);

    return averagePulseLengthMax;
  };

  int eeAddress = eepromStartAddress;

  DEBUG_SERIAL.println(eeAddress);
  float32_t max = getAveragePulseLengthForThisTimer(*channel, RECEIVER_PULSE_COUNT_TO_AVERAGE);
  EEPROM.put(eeAddress, max);
  DEBUG_SERIAL.println("done");
  DEBUG_SERIAL.print("Max = ");DEBUG_SERIAL.println(max);
  DEBUG_SERIAL.println();
  delay(500);

  DEBUG_SERIAL.println("Lets collect the min for this receiver channel");
  DEBUG_SERIAL.println("Move the stick to the min position");
  delay(5000);

  float32_t min = getAveragePulseLengthForThisTimer(*channel, RECEIVER_PULSE_COUNT_TO_AVERAGE);
  eeAddress += sizeof(float32_t);
  DEBUG_SERIAL.println(eeAddress);
  EEPROM.put(eeAddress, min);
  DEBUG_SERIAL.println("done");
  DEBUG_SERIAL.print("Min = ");DEBUG_SERIAL.println(min);
  DEBUG_SERIAL.println();
  delay(500);

  DEBUG_SERIAL.println("Lets collect the centre for this receiver channel");
  DEBUG_SERIAL.println("Move the stick to the centre position");
  delay(5000);

  float32_t mid = getAveragePulseLengthForThisTimer(*channel, RECEIVER_PULSE_COUNT_TO_AVERAGE);
  eeAddress += sizeof(float32_t);
  DEBUG_SERIAL.println(eeAddress);
  EEPROM.put(eeAddress, mid);
  DEBUG_SERIAL.println("done");
  DEBUG_SERIAL.print("Mid = ");DEBUG_SERIAL.println(mid);
  DEBUG_SERIAL.println();
  delay(500);

};

void ReceiverCalibrationHandler::rollCalibrator() { calibrateReceiver(receiverManager.getChannel(ROLL_CHANNEL_INDEX), ROLL_START); };
void ReceiverCalibrationHandler::pitchCalibrator() { calibrateReceiver(receiverManager.getChannel(PITCH_CHANNEL_INDEX), PITCH_START); };
void ReceiverCalibrationHandler::yawCalibrator() { calibrateReceiver(receiverManager.getChannel(YAW_CHANNEL_INDEX), YAW_START); };
void ReceiverCalibrationHandler::throttleCalibrator() { calibrateReceiver(receiverManager.getChannel(THROTTLE_CHANNEL_INDEX), THROTTLE_START); };

void ReceiverCalibrationHandler::printAll() { 
  for (int i = 0; i < 1000; ++i) {
    receiverManager.read();
    receiverManager.printAllChannels();
    DEBUG_SERIAL.println();
  }
};

void ReceiverCalibrationHandler::printSaved() { 

  printReceiver("roll", ROLL_START);
  printReceiver("pitch", PITCH_START);
  printReceiver("throttle", THROTTLE_START);
  printReceiver("yaw", YAW_START);

};