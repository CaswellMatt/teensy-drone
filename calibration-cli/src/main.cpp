#include <Arduino.h>
#include "MARGCalibrationHandler.h"
#include "ReceiverCalibrationHandler.h"

#include <arm_math.h>

#include "MenuOptionHandler.h"
#include "MainMenuHandler.h"

MARGCalibrationHandler margCalibrationHandler;
ReceiverCalibrationHandler receiverHandler;

MainMenuHandler mainMenu;

void setup() {
  mainMenu.addOptionHandler(&margCalibrationHandler);
  mainMenu.addOptionHandler(&receiverHandler);

  mainMenu.setup();
}

void loop() {
  delay(1000);
  mainMenu.start();
}