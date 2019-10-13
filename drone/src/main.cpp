#include <Arduino.h>
#include "Drone.h"
Drone drone;

void setup() {
  drone.setup();
}

void loop() {
  drone.start();
}
