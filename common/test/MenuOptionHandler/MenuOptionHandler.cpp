#include "MenuOptionHandler.h"

void MenuOptionHandler::start() {
  printTitle();
  int exitIndex = 0;

  bool shouldContinue = true;
  optionsMap[exitIndex] = new MenuOption(
    [&shouldContinue](){ shouldContinue = false; },
    "exit"
  );

  while (shouldContinue) {

    DEBUG_SERIAL.println();

    std::map<int, MenuOption*>::iterator itr; 
    for (itr = std::next(optionsMap.begin()); itr != optionsMap.end(); ++itr) { 
      DEBUG_SERIAL.print(itr->first);DEBUG_SERIAL.print(". ");
      itr->second->printMessage(); 
    }

    DEBUG_SERIAL.print(0);DEBUG_SERIAL.print(". ");
    optionsMap[0]->printMessage(); 

    DEBUG_SERIAL.println();

    while(!DEBUG_SERIAL.available()) {};
    
    int asciiToNumberSelectionInput = DEBUG_SERIAL.read() - '0';
    if (optionsMap.find( asciiToNumberSelectionInput ) != optionsMap.end()) {
      optionsMap[asciiToNumberSelectionInput]->callback();
    } else {
      DEBUG_SERIAL.println("no option by that value available");
    }
  };
}

  void MenuOptionHandler::addOption(int optionIndex, std::function<void()> optionFunction, const String optionMenuMessageText) {
    optionsMap[optionIndex] = new MenuOption(
      optionFunction, 
      optionMenuMessageText
    );
  }