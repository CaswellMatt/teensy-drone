#include "MARGTest.h"
#include <arm_math.h>

#define TEST_MARG_OUTPUT 1

MARGTest::MARGTest() {
  m_message = String("Test Marg");
}

const String runFilterMessage = "Run filter on data";
const String rotationRatesMessage = "Run gyro";
const String accelerationMessage = "Run accel";
const String magneticsMessage = "Run magnetometer";

void MARGTest::setup() {
  addExit(this);
  addOption(this, &MARGTest::showFilteredVsUnfiltered, runFilterMessage);
  addOption(this, &MARGTest::printRawRotationalRates, rotationRatesMessage);
  addOption(this, &MARGTest::printRawAccelerometer, accelerationMessage);
  addOption(this, &MARGTest::printRawMagnetics, magneticsMessage);
  const bool FILTERS_ON = false;
  m_marg = new MARG(FILTERS_ON, FILTERS_ON);
} 

void MARGTest::printTitle() {
  DEBUG_SERIAL.println("MARG Test");
}

void MARGTest::showFilteredVsUnfiltered() {
  static long timer = micros();
  for (int i = 0; i < 20000; ++i) {
    m_marg->read();
    Vector rotationalRates    = m_marg->getRotationalRates();
    Vector rotationalRatesRaw = m_marg->getRotationalRatesRaw();

    DEBUG_SERIAL.print(rotationalRatesRaw.v0, 5);DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.println(rotationalRates.v0, 5);
    DEBUG_SERIAL.print(rotationalRatesRaw.v0, 5);DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.println(rotationalRates.v0, 5);
    while(micros() - timer < LOOPTIME_US);
    timer = micros();
  }
  
}

void MARGTest::printRawRotationalRates()
{
  printSensor(&MARG::getRotationalRatesRaw);
}

void MARGTest::printRawAccelerometer()
{
  printSensor(&MARG::getAccelerationRaw);
}

void MARGTest::printRawMagnetics()
{
  printSensor(&MARG::getMagneticsRaw);
}


void MARGTest::printSensor(Vector (MARG::*dataGetter)())
{
  static long timer = micros();
  for (int i = 0; i < 20000; ++i) {
    m_marg->read();
    Vector data = (m_marg->*(dataGetter))();

    DEBUG_SERIAL.print(data.v0, 5);DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print(data.v1, 5);DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print(data.v2, 5);DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.println();
    
    while(micros() - timer < LOOPTIME_US);
    timer = micros();
  }
}