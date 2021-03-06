#include "MARGTest.h"
#include <arm_math.h>

#define TEST_MARG_OUTPUT 1

MARGTest::MARGTest() {
  m_message = String("Test Marg");
}

const String runFilterMessage = "Run filter on data";

void MARGTest::setup() {
  addExit(this);
  addOption(this, &MARGTest::showFilteredVsUnfiltered, runFilterMessage);
  const bool FILTERS_ON = true;
  m_marg = new MARG(FILTERS_ON, FILTERS_ON);
} 

void MARGTest::printTitle() {
  DEBUG_SERIAL.println("MARG Test");
}

void MARGTest::showFilteredVsUnfiltered() {
  static long timer = micros();
  for (int i = 0; i < 5000; ++i) {
    m_marg->read();
    Vector rotationalRates    = m_marg->getRotationalRates();
    Vector rotationalRatesRaw = m_marg->getRotationalRatesRaw();

    DEBUG_SERIAL.print(rotationalRatesRaw.v0, 5);DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.println(rotationalRates.v0, 5);
    while(micros() - timer < LOOPTIME_US);
    timer = micros();
  }
  
}