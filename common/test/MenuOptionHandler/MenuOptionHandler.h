#ifndef MENU_OPTION_HANDLER
#define MENU_OPTION_HANDLER
#include <Arduino.h>
#include <map>
#include "MenuOption.h"
#include "EEPROM.h"
#include "MemoryLocations.h"

class MenuOptionHandler {
private:
  std::map<int, MenuOption*> optionsMap;
public:

  void printMessage() {
    Serial.println(message);
  }

  virtual void printTitle() = 0;
  virtual void setup() = 0;

  void addOption(int optionIndex, std::function<void()> optionFunction, const String optionMenuMessageText);

  void start();
protected:
  String message;

};

#endif