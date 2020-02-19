#ifndef MENU_OPTION_HANDLER
#define MENU_OPTION_HANDLER
#include <Arduino.h>
#include <map>
#include "MenuOption.h"
#include "EEPROM.h"
#include "MemoryLocations.h"
#include "DynamicArray.h"

class MenuOptionHandler {
private:
  DynamicArray<IMenuOption*> m_options;
  bool m_shouldContinue;
public:

  void printMessage() {
    DEBUG_SERIAL.println(m_message);
  }

  virtual void printTitle() = 0;
  virtual void setup() = 0;

  void exit() {
    m_shouldContinue = false;
  }

  template<typename Type>
  void addExit(Type* instance) {
    IMenuOption* menuOption = new MenuOption<Type>(
      instance,
      &Type::exit,
      "exit"
    );

    m_options.append(menuOption);
  }

  template<typename Type>
  void addOption(Type* instance, void (Type::*optionFunction)(), const String optionMenuMessageText) {
    IMenuOption* menuOption = new MenuOption<Type>(
      instance,
      optionFunction, 
      optionMenuMessageText
    );
    
    m_options.append(menuOption);
  }

  void start();
protected:
  String m_message;

};

#endif