
#ifndef MAIN_MENU_HANDLER
#define MAIN_MENU_HANDLER

#include "MenuOptionHandler.h"
#include "DynamicArray.h"
#define EXIT_INDEX 0
class MainMenuHandler {
public:

  void setup();
  void start();

  void addOptionHandler(MenuOptionHandler* menuOptionHandler);

private:
  DynamicArray<MenuOptionHandler*> m_handlers;

};

#endif