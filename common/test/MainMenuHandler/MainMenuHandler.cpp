#include "MainMenuHandler.h"

void MainMenuHandler::setup() {
  DEBUG_SERIAL.begin(1);
  DEBUG_SERIAL.flush();
  
  std::map<int,MenuOptionHandler*>::iterator itr; 
  for (itr = handlerMap.begin(); itr != handlerMap.end(); ++itr) { 
    DEBUG_SERIAL.println(itr->first);
    itr->second->setup();
  }
}

void MainMenuHandler::start() {
 delay(2000);
  bool shouldKeepMenuUp = true;
  while (shouldKeepMenuUp) {
    
    DEBUG_SERIAL.println("Main Menu");
    DEBUG_SERIAL.println();

    std::map<int,MenuOptionHandler*>::iterator itr; 
    for (itr = handlerMap.begin(); itr != handlerMap.end(); ++itr) { 
      DEBUG_SERIAL.print(itr->first);DEBUG_SERIAL.print(". ");
      itr->second->printMessage(); 
    }

    DEBUG_SERIAL.println("0. exit");
    DEBUG_SERIAL.println();

    
    while (!DEBUG_SERIAL.available()) {};
    int asciiToNumberSelectionInput = DEBUG_SERIAL.read() - '0';

    if (handlerMap.find( asciiToNumberSelectionInput ) != handlerMap.end()) {
      handlerMap[asciiToNumberSelectionInput]->start();
    } else if (asciiToNumberSelectionInput == EXIT_INDEX) {
      shouldKeepMenuUp = false;
    } else {
      DEBUG_SERIAL.println("no option selected");
    }
  };
  
  DEBUG_SERIAL.println("Finished");
  while(1) {};
}

void MainMenuHandler::addOptionHandler(int menuIndex, MenuOptionHandler* menuOptionHandler) {
  handlerMap[menuIndex] = menuOptionHandler;
}