//
// Created by wenchun on 3/26/21.
//

#ifndef XIAOTIANHYBRID_USERINTERFACE_H
#define XIAOTIANHYBRID_USERINTERFACE_H

#include "DataSets.h"
#include "joystick.h"
#include <memory>

using namespace std;

class UserInterface {
public:
  UserInterface(UserParameterHandler *param, UserCmd *userCmd);

  void update_keyboard();
  void update_joystick();

private:
  UserParameterHandler *param;
  UserCmd *userCmd;
  GamepadCommand gameCmd;
  JoystickEvent event;
  shared_ptr<Joystick> joystick;
  int key;
  int iter = 0;
};

void keyBoardDetector(int *key)
{
    system("stty -icanon"); //reference: https://blog.csdn.net/yinshitaoyuan/article/details/51412738
    system("stty -echo");
    while (true) //0: false ; non-zero: true
    {
        *key = getchar();
        usleep(1000);
    }
}

void pressKeytoContinue(int *key, int ASCII)
{
    while (*key != ASCII)
    {
        usleep(1000);
    }
}

#endif //XIAOTIANHYBRID_USERINTERFACE_H
