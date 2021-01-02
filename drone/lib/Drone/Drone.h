#ifndef DRONE
#define DRONE

#include <Arduino.h>

#include "Quaternion.h"
#include "OrientationFilter.h"
// #include "MotorControlManager.h"
#include "ChannelAligner.h"
#include "ReceiverManager.h"

class Drone {
public:
  Drone();
  ~Drone();

  void setup();
  void start();

private:
  MARG m_marg;
  OrientationFilter m_orientationFilter;
  ReceiverManager m_receiverManager;
  long m_timer;

  bool motorsAreActive();
  void debugPrint();
  float32_t pulseToRadiansPerSecondControlInput(ChannelAligner* aligner);
};


#endif
