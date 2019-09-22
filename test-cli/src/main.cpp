#include <Arduino.h>
#include "MainMenuHandler.h"

#include "MotorNoiseTest.h"

MainMenuHandler mainMenu;

#define MOTOR_NOISE_TEST_INDEX 1
MotorNoiseTest motorNoiseTest;

void setup() {

  mainMenu.addOptionHandler(MOTOR_NOISE_TEST_INDEX, &motorNoiseTest);
  mainMenu.setup();

}

void loop() {

  mainMenu.start();

}
