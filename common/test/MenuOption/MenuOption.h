#ifndef MENU_OPTION
#define MENU_OPTION
#include <Arduino.h>
#include <functional>
#include "Constants.h"

class IMenuOption {
public:
  virtual void callback() = 0;
  virtual void printMessage()  = 0;
};

template<typename Type>
class MenuOption : public IMenuOption {
public:
  MenuOption(
    Type* instance,
    void (Type::*memberPtr)(), 
    const String optionMessage) : 
      m_instance(instance),
      m_memberPtr(memberPtr),
      m_message(optionMessage) {
     
  }

  virtual void callback() override {
    (m_instance->*m_memberPtr)();
  }
  
  Type* m_instance;
  void (Type::*m_memberPtr)();
  const String m_message;
  
  void printMessage() override {
    DEBUG_SERIAL.println(m_message);
  }
};

#endif