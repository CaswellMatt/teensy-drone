#include <Arduino.h>

#include "Quaternion.h"
#include "OrientationFilter.h"
#include "ReceiverManager.h"
#include "Arduino.h"
#include "MotorControlManager.h"
 
OrientationFilter orientationFilter;

long timer;
float32_t loopTime;

void setup() {
  while(!Serial);
  ReceiverManager::setupReceivers();
  ReceiverManager::setupAligners();
  MotorControlManager::setup();
  timer = micros();
}

float pulseTime = 125;
int direction = 1;

void loop() {
  while(micros() - timer < 500);
  loopTime = (float32_t)(micros() - timer) / 1000000;
  
  timer = micros();

  // MotorControlManager::frontLeft.trigger(pulseTime);
  // MotorControlManager::frontRight.trigger(pulseTime);
  // MotorControlManager::backLeft.trigger(pulseTime);
  // MotorControlManager::backRight.trigger(pulseTime);

  orientationFilter.update(loopTime);

  if (pulseTime > 140 || pulseTime < 125) {
    direction *= -1;
  }

  noInterrupts();
    pulseTime += direction * 0.01;
    // Serial.println(pulseTime);
  interrupts(); 

  // Serial.println();
}





