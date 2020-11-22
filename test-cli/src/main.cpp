#include <Arduino.h>
#include "MainMenuHandler.h"

#include "MotorNoiseTest.h"
#include "FilterTest.h"
#include "MARGTest.h"

MainMenuHandler mainMenu;

MotorNoiseTest motorNoiseTest;
FilterTest filterTest;
MARGTest margTest;

void setup() {

  mainMenu.addOptionHandler(&motorNoiseTest);
  mainMenu.addOptionHandler(&filterTest);
  // mainMenu.addOptionHandler(&margTest);
  mainMenu.setup();

}

void loop() {

  mainMenu.start();

}
