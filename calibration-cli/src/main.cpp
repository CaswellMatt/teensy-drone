#include <Arduino.h>
#include "MARGCalibrationHandler.h"
#include "ReceiverCalibrationHandler.h"
#include "ESCCalibrationHandler.h"
#include <arm_math.h>

#include "MenuOptionHandler.h"
#include "MainMenuHandler.h"

MARGCalibrationHandler margCalibrationHandler;
ReceiverCalibrationHandler receiverHandler;
ESCCalibrationHandler escHandler;

#define MARG_HANDLER_INDEX 1
#define RECEIVER_HANDLER_INDEX 2
#define ESC_HANDLER_INDEX 3

MainMenuHandler mainMenu;

void setup() {

  mainMenu.addOptionHandler(MARG_HANDLER_INDEX, &margCalibrationHandler);
  mainMenu.addOptionHandler(RECEIVER_HANDLER_INDEX, &receiverHandler);
  mainMenu.addOptionHandler(ESC_HANDLER_INDEX, &escHandler);

  mainMenu.setup();

}

void loop() {
  mainMenu.start();
}