#ifndef CONSTANTS
#define CONSTANTS

// #define BLUETOOTH
#ifdef BLUETOOTH
#define DEBUG_SERIAL Serial1
#else
#define DEBUG_SERIAL Serial
#endif

#include "arm_math.h"

constexpr long LOOPTIME_US = 200;
constexpr float32_t LOOPTIME_S = (float32_t)LOOPTIME_US / 1000000;

constexpr float32_t THROTTLE_MAP_START  = 0;
constexpr float32_t THROTTLE_MAP_END    = 1796;
constexpr float32_t THROTTLE_MAP_MIDDLE = (THROTTLE_MAP_START + THROTTLE_MAP_END) / 2;

#endif