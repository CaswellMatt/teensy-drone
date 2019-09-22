#include "MainMenuHandler.h"

void MainMenuHandler::setup() {
  while(!Serial);

  Serial.begin(1);
  Serial.flush();
  Serial.clear();

  std::map<int,MenuOptionHandler*>::iterator itr; 
  for (itr = handlerMap.begin(); itr != handlerMap.end(); ++itr) { 
    Serial.println(itr->first);
    itr->second->setup();
  }
}

void MainMenuHandler::start() {
 delay(2000);
  bool shouldKeepMenuUp = true;
  while (shouldKeepMenuUp) {
    
    Serial.println("Main Menu");
    Serial.println();

    std::map<int,MenuOptionHandler*>::iterator itr; 
    for (itr = handlerMap.begin(); itr != handlerMap.end(); ++itr) { 
      Serial.print(itr->first);Serial.print(". ");
      itr->second->printMessage(); 
    }

    Serial.println("0. exit");
    Serial.println();

    
    while (!Serial.available()) {};
    int asciiToNumberSelectionInput = Serial.read() - '0';

    if (handlerMap.find( asciiToNumberSelectionInput ) != handlerMap.end()) {
      handlerMap[asciiToNumberSelectionInput]->start();
    } else if (asciiToNumberSelectionInput == EXIT_INDEX) {
      shouldKeepMenuUp = false;
    } else {
      Serial.println("no option selected");
    }
  };
  
  Serial.println("Finished");
  while(1) {};
}

void MainMenuHandler::addOption(int menuIndex, MenuOptionHandler* menuOptionHandler) {
  handlerMap[menuIndex] = menuOptionHandler;
}