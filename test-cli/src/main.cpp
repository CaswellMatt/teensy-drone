#include <Arduino.h>
#include "MainMenuHandler.h"

#include "MotorNoiseTest.h"
#include "FilterTest.h"

MainMenuHandler mainMenu;

#define MOTOR_NOISE_TEST_INDEX 1
#define FILTER_TEST_INDEX      2

MotorNoiseTest motorNoiseTest;
FilterTest filterTest;

void setup() {

  mainMenu.addOptionHandler(MOTOR_NOISE_TEST_INDEX, &motorNoiseTest);
  mainMenu.addOptionHandler(FILTER_TEST_INDEX, &filterTest);
  mainMenu.setup();

}

void loop() {

  mainMenu.start();

}
