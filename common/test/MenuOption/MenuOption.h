#ifndef MENU_OPTION
#define MENU_OPTION
#include <Arduino.h>
#include <functional>
class MenuOption {
public:
  MenuOption(
    std::function<void()> optionCallback, 
    const String optionMessage) : 
      callback(optionCallback),
      message(optionMessage)
  {
     
  }
  
  std::function<void()> callback;
  const String message;
  
  void printMessage() {
    Serial.println(message);
  }
};

#endif