#include "ReceiverCalibrationHandler.h"
#include <Arduino.h>
#include "ReceiverManager.h"

#include <arm_math.h>

#define ROLL_INDEX        1
#define PITCH_INDEX       2
#define THROTTLE_INDEX    3
#define YAW_INDEX         4
#define PRINT_INDEX       5
#define PRINT_SAVED_INDEX 6

const String rollOptionText = "Calibrate Roll";
const String pitchOptionText = "Calibrate Pitch";
const String yawOptionText = "Calibrate Yaw";
const String throttleOptionText = "Calibrate Throttle";
const String printOptionText = "Print Receiver Values";
const String printSavedValuesText = "Print Saved Receiver Endpoints";

ReceiverCalibrationHandler::ReceiverCalibrationHandler() {
  message = String("Calibrate Receiver");
}

void ReceiverCalibrationHandler::printTitle() {
  Serial.println("Receiver Calibration Menu");
}

void ReceiverCalibrationHandler::setup() {

  auto rollCalibrator = [this]() { calibrateReceiver(&ReceiverManager::rollInput, ROLL_START); };

  optionsMap[ROLL_INDEX] = new MenuOption(
    rollCalibrator, 
    rollOptionText
  );

  auto pitchCalibrator = [this]() { calibrateReceiver(&ReceiverManager::pitchInput, PITCH_START); };

  optionsMap[PITCH_INDEX] = new MenuOption(
    pitchCalibrator, 
    pitchOptionText
  );

  auto yawCalibrator = [this]() { calibrateReceiver(&ReceiverManager::yawInput, YAW_START); };

  optionsMap[YAW_INDEX] = new MenuOption(
    yawCalibrator, 
    yawOptionText
  );

  auto throttleCalibrator = [this]() { calibrateReceiver(&ReceiverManager::throttleInput, THROTTLE_START); };

  optionsMap[THROTTLE_INDEX] = new MenuOption(
    throttleCalibrator, 
    throttleOptionText
  );

  auto printAll = [this]() { 
    for (int i = 0; i < 1000; ++i) {
      ReceiverManager::printAllPulseLengths();
      Serial.println();
    }
  };

  optionsMap[PRINT_INDEX] = new MenuOption(
    printAll, 
    printOptionText
  );

  auto printSaved = [this]() { 
    
   auto printReceiver = [](String receiverName, int startAddress) {

      float32_t min;
      float32_t max;
      float32_t mid;

      int eeAddress = startAddress;
      Serial.println(eeAddress);
      EEPROM.get(eeAddress, min);

      eeAddress += sizeof(float32_t);
      Serial.println(eeAddress);
      EEPROM.get(eeAddress, max);

      eeAddress += sizeof(float32_t);
      Serial.println(eeAddress);
      EEPROM.get(eeAddress, mid);

      Serial.print(receiverName); Serial.print(" ");
      Serial.print("Max "); Serial.print(max); Serial.print(" ");
      Serial.print("Min "); Serial.print(min); Serial.print(" ");
      Serial.print("Mid "); Serial.print(mid); Serial.println();
    };


    printReceiver("roll", ROLL_START);
    printReceiver("pitch", PITCH_START);
    printReceiver("throttle", THROTTLE_START);
    printReceiver("yaw", YAW_START);
    
  };

  optionsMap[PRINT_SAVED_INDEX] = new MenuOption(
    printSaved, 
    printSavedValuesText
  );

  ReceiverManager::setupReceivers();

}

const int RECEIVER_PULSE_COUNT_TO_AVERAGE = 100;

void ReceiverCalibrationHandler::calibrateReceiver(ReceiverPulseTimer* timer, int eepromStartAddress) {
  Serial.println("Lets collect the max for this receiver channel");
  Serial.println("Move the stick to the max position");
  delay(5000);

  auto getAveragePulseLengthForThisTimer = [](ReceiverPulseTimer* timer, int size) {
    long sumOfPulse = 0;

    for (int i = 0; i < size; ++i) {
      sumOfPulse += timer->getPulseLength();
      delay(2);
    }

    float32_t averagePulseLengthMax = static_cast<float32_t>(sumOfPulse / size);

    return averagePulseLengthMax;
  };

  int eeAddress = eepromStartAddress;

  Serial.println(eeAddress);
  float32_t max = getAveragePulseLengthForThisTimer(timer, RECEIVER_PULSE_COUNT_TO_AVERAGE);
  EEPROM.put(eeAddress, max);
  Serial.println("done");
  Serial.print("Max = ");Serial.println(max);
  Serial.println();
  delay(500);

  Serial.println("Lets collect the min for this receiver channel");
  Serial.println("Move the stick to the min position");
  delay(5000);

  float32_t min = getAveragePulseLengthForThisTimer(timer, RECEIVER_PULSE_COUNT_TO_AVERAGE);
  eeAddress += sizeof(float32_t);
  Serial.println(eeAddress);
  EEPROM.put(eeAddress, min);
  Serial.println("done");
  Serial.print("Min = ");Serial.println(min);
  Serial.println();
  delay(500);

  Serial.println("Lets collect the centre for this receiver channel");
  Serial.println("Move the stick to the centre position");
  delay(5000);

  float32_t mid = getAveragePulseLengthForThisTimer(timer, RECEIVER_PULSE_COUNT_TO_AVERAGE);
  eeAddress += sizeof(float32_t);
  Serial.println(eeAddress);
  EEPROM.put(eeAddress, mid);
  Serial.println("done");
  Serial.print("Mid = ");Serial.println(mid);
  Serial.println();
  delay(500);

};
