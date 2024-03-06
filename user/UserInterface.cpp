//
// Created by wenchun on 3/26/21.
//

#include "UserInterface.h"
#include <iostream>
#include <thread>
#include <string>
#define KEYBOARD_CONTROL

UserInterface::UserInterface(UserParameterHandler *param,
                             UserCmd *userCmd) :
        param(param),
        userCmd(userCmd)
{
    #ifdef KEYBOARD_CONTROL
    std::thread detect_keyboard(keyBoardDetector, &key);
    detect_keyboard.detach();

    std::cout << "Press c to continue..." << std::endl;
    pressKeytoContinue(&key, 99);
    #endif

    #ifdef JOYSTICK_CONTROL
    joystick = std::make_shared<Joystick>("/dev/input/js0");
    if (!joystick->isFound())
        printf("joystick not found!\n");
    #endif
}

void UserInterface::update_keyboard()
{
    std::cout << "\033[2J\033[1;1H";
    std::cout << "Foot Control Commands (need to enter Control Mode 6 first):\n"
    << "q >> hold   |   w >> calibration   |   e >> extension   |   r >> compression\n"
    << "----------------------------------------------------------------------------\n"
    << "Control Mode Commands:\n" 
    << "1 >> legCtrl(TODO)     |   2 >> hold           |   3 >> WheelStanceTest\n"
    << "4 >> WheelMotionTest   |   5 >> swingTest_v2   |   6 >> swingTest_v3\n"
    << "7 >> PDSitDown(TODO)   |   8 >> PDSwitch       |   9 >> PDStance\n"
    << "----------------------------------------------------------------------------\n"
    << "current key pressed: " << (char)key << "\n"
    << "current ctrl mode: " << param->ctrl_num << std::endl;

    //---------------- Control Mode Commands ----------------
    if ((char)key == '0')
    {
        param->ctrl_num = 0;
        userCmd->gaitNum = 2;
    }
    else if ((char)key == '1')//legControl.run()
    {
        param->ctrl_num = 2;
        userCmd->gaitNum = 1;
        // double filter = 0.1, filter_dh = 0.01, vel_test = 0.5;
        // userCmd->vx_des = filter * 2 * vel_test + (1 - filter) * userCmd->vx_des;
    }
    else if ((char)key == '2')//legControl.run_foot()
    {
        param->ctrl_num = 2;
        userCmd->gaitNum = 2;
        userCmd->vx_des = 0.1;
    }
    else if ((char)key == '3')// WheelStanceTest
    {
        param->ctrl_num = 3;
        userCmd->gaitNum = 1;

        double filter = 0.1, filter_dh = 0.01, vel_test = 0.0;
        userCmd->vx_des = filter * 2 * vel_test + (1 - filter) * userCmd->vx_des;
        // userCmd->dh = filter_dh * 0.3 * gameCmd.rightStickAnalog[0] + (1 - filter_dh) * userCmd->dh;        
    }
    else if ((char)key == '4')// WheelMotionTest
    {
        param->ctrl_num = 4;
        userCmd->gaitNum = 1;

        for (int i(0); i < 4; i++)
            param->test[i] = 0.5;//防止多次进入mode4出现目标速度过快情况

        double filter = 0.1, filter_dh = 0.01, vel_test = 0.5;
        userCmd->vx_des = filter * 2 * vel_test + (1 - filter) * userCmd->vx_des;
        // userCmd->dh = filter_dh * 0.3 * gameCmd.rightStickAnalog[0] + (1 - filter_dh) * userCmd->dh;        
    }
    else if ((char)key == '5')//swingTest_v2
    {
        param->ctrl_num = 5;
    }
    else if ((char)key == '6')//swingTest_v3
    {
        param->ctrl_num = 6;
    }
    else if ((char)key == '7')//PDSitDown(TODO)
    {
        param->ctrl_num = 7;
    }
    else if ((char)key == '8')//PDSwitch
    {
        param->ctrl_num = 8;
    }
    else if ((char)key == '9')//PDStance
    {
        param->ctrl_num = 9;
    }
    //---------------- Control Mode Commands ----------------

    //---------------- Foot Control Commands ----------------
    else if ((char)key == 'q') //q-0-hold  |  w-1-calibration  |  e-2-extension  |  r-3-compression
    {
        for (int i(0); i < 4; i++) {
            param->des_foot_state[i] = 0;
        }
    }
    else if ((char)key == 'w')
    {
        for (int i(0); i < 4; i++) {
            param->des_foot_state[i] = 1;
        }
    }
    else if ((char)key == 'e')
    {
        for (int i(0); i < 4; i++) {
            param->des_foot_state[i] = 2;
        }
    }
    else if ((char)key == 'r')
    {
        for (int i(0); i < 4; i++) {
            param->des_foot_state[i] = 3;
        }
    }
    else if ((char)key == 'A')
    {
        for (int i(0); i < 4; i++) {
            if (param->test[i] <= 2)
                param->test[i] += 0.005;
        }
    }
    else if ((char)key == 'B')
    {
        for (int i(0); i < 4; i++) {
            if (param->test[i] >= -2)
                param->test[i] -= 0.005;
        }
    }
    else if ((char)key == 'C')
    {
        param->test[0] = 0.5;
        param->test[2] = 0.5;
        param->test[1] = -0.5;
        param->test[3] = -0.5;
    }
    else if ((char)key == 'D')
    {
        param->test[0] = -0.5;
        param->test[2] = -0.5;
        param->test[1] = 0.5;
        param->test[3] = 0.5;
    }
    //---------------- Foot Control Commands ----------------
}

void UserInterface::update_joystick()
{
    // update joystick state at 60Hz
    if (iter % 1 / (60 * param->dt) == 0)
        joystick->updateCommand(&event, gameCmd);

    // control mode
    if (param->ctrl_num == 10 && gameCmd.BACK)
        param->ctrl_num = 9; //PD stance

    else if (param->ctrl_num == 9 && gameCmd.START)
    {
        param->ctrl_num = 1; // from PD stance to force control
        userCmd->vx_des = 0.;
        userCmd->vy_des = 0.;
    }
    else if ((param->ctrl_num == 1 && userCmd->gaitNum == 1) && gameCmd.BACK)
        param->ctrl_num = 7; // from force stance to lie down
    else if(param->ctrl_num == 7 && gameCmd.B)
        param->ctrl_num = 9; // PD stance
    else if (param->ctrl_num == 10 && gameCmd.Y) {
        param->ctrl_num = 11;
    }
    else if (param->ctrl_num == 11 && gameCmd.RB && gameCmd.A)
        param->ctrl_num = 10;
    else if (param->ctrl_num == 11 && gameCmd.START) { // from wheel control to mpc locomotion
        param->ctrl_num = 1;
        userCmd->gaitNum = 2;
        userCmd->vx_des = 0.;
        userCmd->vy_des = 0.;
        userCmd->yawd_des = 0.;
    }
    else if (param->ctrl_num == 1 && (userCmd->gaitNum == 1 || userCmd->gaitNum == 2) && gameCmd.Y) { // from mpc to wheel control
        param->ctrl_num = 11;
    }


    if (param->ctrl_num == 11) { // enable wheel test
        userCmd->gaitNum = 1;
        double filter = 0.1, filter_dh = 0.01;
        // userCmd->vx_wheel = filter * 2*gameCmd.leftStickAnalog[0] + (1 - filter) * userCmd->vx_wheel;
        userCmd->vx_des = filter * 2*gameCmd.leftStickAnalog[0] + (1 - filter) * userCmd->vx_des;
        // userCmd->yawd_des = filter * 8*gameCmd.rightStickAnalog[1] + (1 - filter) * userCmd->yawd_des;
        userCmd->dh = filter_dh * 0.3*gameCmd.rightStickAnalog[0] + (1 - filter_dh) * userCmd->dh;
    }

    // gait
    if (param->ctrl_num == 1 && gameCmd.LB && gameCmd.X)
        userCmd->gaitNum = 2; // trotting
    else if (param->ctrl_num == 1 && gameCmd.LB && gameCmd.Y)
        userCmd->gaitNum = 5; // bounding
    else if (param->ctrl_num == 1 && gameCmd.LB && gameCmd.A)
        userCmd->gaitNum = 1; // stance

    // command
    if (param->ctrl_num == 1)
    {
        double filter = 0.1;
        if (userCmd->gaitNum != 1) {
            // userCmd->vx_des = filter * gameCmd.leftStickAnalog[0] + (1 - filter) * userCmd->vx_des;
            userCmd->vy_des = filter * gameCmd.leftStickAnalog[1] + (1 - filter) * userCmd->vy_des;
        }
        userCmd->vx_des = filter * gameCmd.leftStickAnalog[0] + (1 - filter) * userCmd->vx_des;
        userCmd->yawd_des = filter * gameCmd.rightStickAnalog[1] + (1 - filter) * userCmd->yawd_des;
        // cout << "yawd_des = " << userCmd->yawd_des << endl;
    }

    iter++;
}
