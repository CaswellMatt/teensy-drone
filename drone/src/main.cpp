#include <Arduino.h>
#include "Drone.h"
Drone drone;

void setup() {
  drone.setup();
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  drone.start();
}
