#include "DShotMotorController.h"

#define SIGNAL_CREATION_DELAY_ALIGNMENT 18 //ns
#define DSHOT_BT_DURATION         ((1670 * 2) - SIGNAL_CREATION_DELAY_ALIGNMENT) //1652 //1650 //1670          // Duration of 1 DSHOT packet in ns
#define DSHOT_LP_DURATION         ((1250 * 2) - SIGNAL_CREATION_DELAY_ALIGNMENT) //1232 //1225 //1250          // Duration of a DSHOT long pulse in ns
#define DSHOT_SP_DURATION         ((625 * 2) - SIGNAL_CREATION_DELAY_ALIGNMENT)  //611  //600  //625           // Duration of a DSHOT short pulse in ns

namespace 
{
  const uint32_t CPU_FREQUENCY = F_CPU_ACTUAL;
  const uint32_t DSHOT_SHORT_PULSE_CLOCK_TICKS   = ((CPU_FREQUENCY>>16) * DSHOT_SP_DURATION) / (1000000000UL>>16);     // DSHOT short pulse duration (nb of F_BUS periods)
  const uint32_t DSHOT_LONG_PULSE_CLOCK_TICKS    = ((CPU_FREQUENCY>>16) * DSHOT_LP_DURATION) / (1000000000UL>>16);     // DSHOT long pulse duration (nb of F_BUS periods)
  const uint32_t DSHOT_TOTAL_PULSE_CLOCK_TICKS   = ((CPU_FREQUENCY>>16) * DSHOT_BT_DURATION) / (1000000000UL>>16);     // DSHOT bit duration (nb of F_BUS periods)
}

uint16_t DShotMotorController::prepareDShotPacket(const uint16_t value, uint16_t telemBit = 0)
{
  uint16_t packet = (value << 1) | telemBit;
  int fourBitCRC = 0, crcData = packet;

  for (uint8_t i = 0; i < 3; i++) {
    fourBitCRC ^= crcData; // xor data by nibbles
    crcData >>= 4;
  }

  return (packet << 4) | (fourBitCRC & 0xf); // append checksum
}


void DShotMotorController::sendPacket(uint16_t frontLeftPacket, uint16_t frontRightPacket, uint16_t backLeftPacket, uint16_t backRightPacket)
{
  constexpr uint32_t BIT_MASK_FOR_ALL_ON = 
    CORE_PIN20_BITMASK | CORE_PIN21_BITMASK | CORE_PIN22_BITMASK | CORE_PIN23_BITMASK;

  constexpr uint32_t BIT_MASK_FOR_ALL_OFF = 
    CORE_PIN20_BITMASK | CORE_PIN21_BITMASK | CORE_PIN22_BITMASK | CORE_PIN23_BITMASK;

  // __disable_irq();
  for (int i = 15; i >= 0; --i)
  {
    uint32_t cyclesStart = ARM_DWT_CYCCNT;
    GPIO6_DR_SET = BIT_MASK_FOR_ALL_ON;

    bool frontLeftBitIsSet = (frontLeftPacket & (1 << i));
    bool frontRightBitIsSet = (frontRightPacket & (1 << i));
    bool backLeftBitIsSet = (backLeftPacket & (1 << i));
    bool backRightBitIsSet = (backRightPacket & (1 << i));

    uint32_t BIT_MASK_FOR_THIS_SIGNAL = 0;
    if (!frontLeftBitIsSet)  { BIT_MASK_FOR_THIS_SIGNAL |= CORE_PIN20_BITMASK; }
    if (!frontRightBitIsSet) { BIT_MASK_FOR_THIS_SIGNAL |= CORE_PIN21_BITMASK; }
    if (!backLeftBitIsSet)   { BIT_MASK_FOR_THIS_SIGNAL |= CORE_PIN22_BITMASK; }
    if (!backRightBitIsSet)  { BIT_MASK_FOR_THIS_SIGNAL |= CORE_PIN23_BITMASK; }
    while (ARM_DWT_CYCCNT - cyclesStart < DSHOT_SHORT_PULSE_CLOCK_TICKS);
    GPIO6_DR_CLEAR = BIT_MASK_FOR_THIS_SIGNAL;
    
    while (ARM_DWT_CYCCNT - cyclesStart < DSHOT_LONG_PULSE_CLOCK_TICKS);
    CORE_PIN22_PORTCLEAR = BIT_MASK_FOR_ALL_OFF;

    while (ARM_DWT_CYCCNT - cyclesStart < DSHOT_TOTAL_PULSE_CLOCK_TICKS);

  }
  // __enable_irq();
}

void DShotMotorController::setup() {
  pinMode(FRONT_LEFT_MOTOR_PIN , OUTPUT);
  pinMode(FRONT_RIGHT_MOTOR_PIN, OUTPUT);
  pinMode(BACK_LEFT_MOTOR_PIN  , OUTPUT);
  pinMode(BACK_RIGHT_MOTOR_PIN , OUTPUT);
}

void DShotMotorController::setSpeed(
  const uint16_t frontLeftSpeed, 
  const uint16_t frontRightSpeed, 
  const uint16_t backLeftSpeed, 
  const uint16_t backRightSpeed)
{
  constexpr uint16_t MIN_THROTTLE = 48;
  uint16_t frontLeftPacket  = prepareDShotPacket(frontLeftSpeed  + MIN_THROTTLE);
  uint16_t frontRightPacket = prepareDShotPacket(frontRightSpeed + MIN_THROTTLE);
  uint16_t backLeftPacket   = prepareDShotPacket(backLeftSpeed   + MIN_THROTTLE);
  uint16_t backRightPacket  = prepareDShotPacket(backRightSpeed  + MIN_THROTTLE);
  sendPacket(frontLeftPacket, frontRightPacket, backLeftPacket, backRightPacket);
}