#ifndef MARG_TEST_HANDLER
#define MARG_TEST_HANDLER

#include "MenuOptionHandler.h"
#include "MARG.h"

class MARGTest : public MenuOptionHandler {
private:
  MARG* m_marg;

public:

  MARGTest();
  ~MARGTest() {
    delete m_marg;
  }

  virtual void setup() override;
  virtual void printTitle() override;

  void showFilteredVsUnfiltered();
  void printSensor(Vector (MARG::*dataGetter)());
  void printRawRotationalRates();
  void printRawAccelerometer();
  void printRawMagnetics();


};

#endif 