#include "CalibrationHandler.h"

void CalibrationHandler::start() {
  printTitle();
  int exitIndex = 0;

  bool shouldContinue = true;
  optionsMap[exitIndex] = new CalibrationOption(
    [&shouldContinue](){ shouldContinue = false; },
    "exit"
  );

  while (shouldContinue) {

    Serial.println();

    std::map<int, CalibrationOption*>::iterator itr; 
    for (itr = std::next(optionsMap.begin()); itr != optionsMap.end(); ++itr) { 
      Serial.print(itr->first);Serial.print(". ");
      itr->second->printMessage(); 
    }

    Serial.print(0);Serial.print(". ");
    optionsMap[0]->printMessage(); 

    Serial.println();

    while(!Serial.available()) {};
    
    int asciiToNumberSelectionInput = Serial.read() - '0';
    if (optionsMap.find( asciiToNumberSelectionInput ) != optionsMap.end()) {
      optionsMap[asciiToNumberSelectionInput]->callback();
    } else {
      Serial.println("no option by that value available");
    }
  };
}
