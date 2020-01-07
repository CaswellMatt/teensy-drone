#include <Arduino.h>
#include "MainMenuHandler.h"

#include "MotorNoiseTest.h"
#include "FilterTest.h"
#include "MARGTest.h"

MainMenuHandler mainMenu;

#define MOTOR_NOISE_TEST_INDEX 1
#define FILTER_TEST_INDEX      2
#define MARG_TEST_INDEX        3

MotorNoiseTest motorNoiseTest;
FilterTest filterTest;
MARGTest margTest;

void setup() {

  mainMenu.addOptionHandler(MOTOR_NOISE_TEST_INDEX, &motorNoiseTest);
  mainMenu.addOptionHandler(FILTER_TEST_INDEX, &filterTest);
  mainMenu.addOptionHandler(MARG_TEST_INDEX, &margTest);
  mainMenu.setup();

}

void loop() {

  mainMenu.start();

}
