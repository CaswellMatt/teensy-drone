#ifndef DRONE
#define DRONE

#include <Arduino.h>

#include "Quaternion.h"
#include "OrientationFilter.h"
#include "MotorControlManager.h"
 
class Drone {
public:
  Drone();
  ~Drone();

  void setup();
  void start();

private:
  MARG m_marg;
  OrientationFilter m_orientationFilter;
  long m_timer;

  bool motorsAreActive();
};


#endif