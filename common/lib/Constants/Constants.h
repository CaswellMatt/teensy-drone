#ifndef CONSTANTS
#define CONSTANTS

#include "arm_math.h"

const long LOOPTIME_US = 1000;
const float32_t LOOPTIME_S = (float32_t)LOOPTIME_US / 1000000;

const float32_t throttleMapStart  = 125;
const float32_t throttleMapEnd    = 240;
const float32_t throttleMapMiddle = (throttleMapStart + throttleMapEnd) / 2;

#endif