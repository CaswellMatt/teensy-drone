#include "ReceiverCalibrationHandler.h"
#include <Arduino.h>
#include "ReceiverManager.h"

#include <arm_math.h>

#define ROLL_INDEX           1
#define PITCH_INDEX          2
#define THROTTLE_INDEX       3
#define YAW_INDEX            4
#define PRINT_INDEX          5
#define PRINT_SAVED_INDEX    6

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

  addExit(this);

  addOption(this, &ReceiverCalibrationHandler::rollCalibrator    , rollOptionText);
  addOption(this, &ReceiverCalibrationHandler::pitchCalibrator   , pitchOptionText);
  addOption(this, &ReceiverCalibrationHandler::yawCalibrator     , yawOptionText);
  addOption(this, &ReceiverCalibrationHandler::throttleCalibrator, throttleOptionText);
  addOption(this, &ReceiverCalibrationHandler::printAll          , printOptionText);
  addOption(this, &ReceiverCalibrationHandler::printSaved        , printSavedValuesText);

  ReceiverManager::setupReceivers();
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

void ReceiverCalibrationHandler::calibrateReceiver(ReceiverPulseTimer* timer, int eepromStartAddress) {
  DEBUG_SERIAL.println("Lets collect the max for this receiver channel");
  DEBUG_SERIAL.println("Move the stick to the max position");
  delay(5000);

  auto getAveragePulseLengthForThisTimer = [](const ReceiverPulseTimer& timer, int size) {
    long sumOfPulse = 0;

    for (int i = 0; i < size; ++i) {
      sumOfPulse += timer.getPulseLengthMicros();
      delay(2);
    }

    float32_t averagePulseLengthMax = static_cast<float32_t>(sumOfPulse / size);

    return averagePulseLengthMax;
  };

  int eeAddress = eepromStartAddress;

  DEBUG_SERIAL.println(eeAddress);
  float32_t max = getAveragePulseLengthForThisTimer(*timer, RECEIVER_PULSE_COUNT_TO_AVERAGE);
  EEPROM.put(eeAddress, max);
  DEBUG_SERIAL.println("done");
  DEBUG_SERIAL.print("Max = ");DEBUG_SERIAL.println(max);
  DEBUG_SERIAL.println();
  delay(500);

  DEBUG_SERIAL.println("Lets collect the min for this receiver channel");
  DEBUG_SERIAL.println("Move the stick to the min position");
  delay(5000);

  float32_t min = getAveragePulseLengthForThisTimer(*timer, RECEIVER_PULSE_COUNT_TO_AVERAGE);
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

  float32_t mid = getAveragePulseLengthForThisTimer(*timer, RECEIVER_PULSE_COUNT_TO_AVERAGE);
  eeAddress += sizeof(float32_t);
  DEBUG_SERIAL.println(eeAddress);
  EEPROM.put(eeAddress, mid);
  DEBUG_SERIAL.println("done");
  DEBUG_SERIAL.print("Mid = ");DEBUG_SERIAL.println(mid);
  DEBUG_SERIAL.println();
  delay(500);

};

void ReceiverCalibrationHandler::rollCalibrator() { calibrateReceiver(&ReceiverManager::rollInput, ROLL_START); };
void ReceiverCalibrationHandler::pitchCalibrator() { calibrateReceiver(&ReceiverManager::pitchInput, PITCH_START); };
void ReceiverCalibrationHandler::yawCalibrator() { calibrateReceiver(&ReceiverManager::yawInput, YAW_START); };
void ReceiverCalibrationHandler::throttleCalibrator() { calibrateReceiver(&ReceiverManager::throttleInput, THROTTLE_START); };

void ReceiverCalibrationHandler::printAll() { 
  for (int i = 0; i < 1000; ++i) {
    ReceiverManager::printAllPulseLengths();
    DEBUG_SERIAL.println();
  }
};

void ReceiverCalibrationHandler::printSaved() { 

  printReceiver("roll", ROLL_START);
  printReceiver("pitch", PITCH_START);
  printReceiver("throttle", THROTTLE_START);
  printReceiver("yaw", YAW_START);

};