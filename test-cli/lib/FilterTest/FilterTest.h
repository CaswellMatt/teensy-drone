#ifndef FILTER_TEST_HANDLER
#define FILTER_TEST_HANDLER

#include "MenuOptionHandler.h"


class FilterTest : public MenuOptionHandler {
public:

  FilterTest();

  virtual void setup() override;
  virtual void printTitle() override;

  void runFilter();

};

#endif 