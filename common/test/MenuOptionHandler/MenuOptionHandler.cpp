#include "MenuOptionHandler.h"

void MenuOptionHandler::start() {
  printTitle();
  int exitIndex = 0;

  m_shouldContinue = true;

  while (m_shouldContinue) {

    DEBUG_SERIAL.println();

    for (int i = 1; i < m_options.size(); ++i) {
      DEBUG_SERIAL.print(i);DEBUG_SERIAL.print(". ");
      m_options.at(i)->printMessage(); 
    }

    DEBUG_SERIAL.print(0);DEBUG_SERIAL.print(". ");
    m_options.at(0)->printMessage(); 

    DEBUG_SERIAL.println();

    while(!DEBUG_SERIAL.available()) {};
    
    int asciiToNumberSelectionInput = DEBUG_SERIAL.read() - '0';
    if (asciiToNumberSelectionInput >= 0 && asciiToNumberSelectionInput < m_options.size()) {
      m_options.at(asciiToNumberSelectionInput)->callback();
    } else {
      DEBUG_SERIAL.println("no option by that value available");
    }
  };
}