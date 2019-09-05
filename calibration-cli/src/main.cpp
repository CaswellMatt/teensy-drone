#include <Arduino.h>
#include "MARGCalibrationHandler.h"
#include "ReceiverCalibrationHandler.h"
#include "ESCCalibrationHandler.h"
#include <arm_math.h>
#include "CalibrationHandler.h"
#include <map>

#define WAITFORINPUT(){          \
  while(!Serial.available()){};  \
  while(Serial.available()){     \
    Serial.read();               \
  };                             \
}                                \

MARGCalibrationHandler margCalibrationHandler;
ReceiverCalibrationHandler receiverHandler;
ESCCalibrationHandler escHandler;

#define EXIT_INDEX 0
#define MARG_HANDLER_INDEX 1
#define RECEIVER_HANDLER_INDEX 2
#define ESC_HANDLER_INDEX 3

std::map<int, CalibrationHandler*> handlerMap;

void setup() {
  while(!Serial);
  
  Serial.begin(1);
  Serial.flush();
  Serial.clear();

  handlerMap[MARG_HANDLER_INDEX] = &margCalibrationHandler;
  handlerMap[RECEIVER_HANDLER_INDEX] = &receiverHandler;
  handlerMap[ESC_HANDLER_INDEX] = &escHandler;

  std::map<int, CalibrationHandler*>::iterator itr; 
  for (itr = handlerMap.begin(); itr != handlerMap.end(); ++itr) { 
    Serial.println(itr->first);
    itr->second->setup();
  }

}

void loop() {

  delay(2000);
  bool shouldKeepMenuUp = true;
  while (shouldKeepMenuUp) {
    
    Serial.println("Main Menu");
    Serial.println();

    std::map<int, CalibrationHandler*>::iterator itr; 
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