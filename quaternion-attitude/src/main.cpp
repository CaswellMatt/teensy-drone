#include <Arduino.h>

#include "Quaternion.h"
#include "OrientationFilter.h"

OrientationFilter orientationFilter;


long timer;
float32_t loopTime;

void setup() {
  timer = micros();
}



void loop() {
  
  while(micros() - timer < 500);
  loopTime = (float32_t)(micros() - timer) / 1000000;
  timer = micros();
  orientationFilter.update(loopTime);

}
