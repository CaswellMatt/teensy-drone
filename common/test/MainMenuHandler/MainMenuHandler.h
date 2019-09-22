
#ifndef MAIN_MENU_HANDLER
#define MAIN_MENU_HANDLER

#include "MenuOptionHandler.h"

#define EXIT_INDEX 0

class MainMenuHandler {
public:

  void setup();
  void start();

  void addOptionHandler(int menuIndex, MenuOptionHandler* menuOptionHandler);

private:
  std::map<int, MenuOptionHandler*> handlerMap;

};

#endif