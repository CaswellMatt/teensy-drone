#include "MainMenuHandler.h"

void MainMenuHandler::setup() {
  DEBUG_SERIAL.begin(9600);
  DEBUG_SERIAL.flush();

  for (size_t i = 0; i < m_handlers.size(); ++i) {
    DEBUG_SERIAL.println(i);
    m_handlers.at(i)->setup();
  }
}

void MainMenuHandler::start() {
  delay(2000);
  bool shouldKeepMenuUp = true;
  while (shouldKeepMenuUp) {
    
    DEBUG_SERIAL.println("Main Menu");
    DEBUG_SERIAL.println();

    for (size_t i = 0; i < m_handlers.size(); ++i) {
      int position = i + 1;
      DEBUG_SERIAL.print(position);DEBUG_SERIAL.print(". ");
      m_handlers.at(i)->printMessage();
    }

    DEBUG_SERIAL.println("0. exit");
    DEBUG_SERIAL.println();
    
    while (!DEBUG_SERIAL.available()) {};
    int asciiToNumberSelectionInput = DEBUG_SERIAL.read() - '0';
    if (asciiToNumberSelectionInput > 0 && (size_t)asciiToNumberSelectionInput <= m_handlers.size()) {
      int arrayIndex = asciiToNumberSelectionInput - 1;
      m_handlers.at(arrayIndex)->start();
    } else if (asciiToNumberSelectionInput == EXIT_INDEX) {
      shouldKeepMenuUp = false;
    } else {
      DEBUG_SERIAL.println("no option selected");
    }
  };
  
  DEBUG_SERIAL.println("Finished");
  while(1) {};
}

void MainMenuHandler::addOptionHandler(MenuOptionHandler* menuOptionHandler) {
  m_handlers.append(menuOptionHandler);
}