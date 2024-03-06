/**
 * @file    main.cpp
 * @brief   The entering gate of the whole project
 *
 * The main method works as the following order. First,
 *
 * @author  Junde
 * @date    2024-02-26
 * @version 1.0
 */

#include <iostream>
#include <thread>
#include <fstream>
#include <csignal>
#include "Controller.h"
#include "Estimator.h"
#include "GaitScheduler.h"
#include "Planner.h"
#include "Configuration.h"
#include "Timer.h"
#include "UserInterface.h"
#include "utility/limxsdk/datatypes.h"
#include "ctime"

bool exitrequest = false;

static void default_handler(int sig)
{
    (void)sig;
    exitrequest = true;
}

int main()
{
    signal(SIGINT, default_handler);  // ctrl c
    signal(SIGTSTP, default_handler); // ctrl z
    signal(SIGQUIT, default_handler); // ctrl '\'

    DataSets dataSets;                                                // contrain data used for controlling the robot
    UserParameterHandler param(std::string(THIS_COM) + "param.yaml"); // TODO: check the yaml file
    pin::urdf::buildModel(param.urdf, pin::JointModelFreeFlyer(),
                          dataSets.robotModelData.model);
    dataSets.robotModelData.data = pin::Data(dataSets.robotModelData.model);
    dataSets.robotModelData.flwID = dataSets.robotModelData.model.getBodyId(param.LF_wheel);
    dataSets.robotModelData.frwID = dataSets.robotModelData.model.getBodyId(param.RF_wheel);
    dataSets.robotModelData.hlwID = dataSets.robotModelData.model.getBodyId(param.LH_wheel);
    dataSets.robotModelData.hrwID = dataSets.robotModelData.model.getBodyId(param.RH_wheel);
    dataSets.robotModelData.flhID = dataSets.robotModelData.model.getJointId(param.LF_HFE);
    dataSets.robotModelData.frhID = dataSets.robotModelData.model.getJointId(param.RF_HFE);
    dataSets.robotModelData.hlhID = dataSets.robotModelData.model.getJointId(param.LH_HFE);
    dataSets.robotModelData.hrhID = dataSets.robotModelData.model.getJointId(param.RH_HFE);
    dataSets.robotModelData.flfootID = dataSets.robotModelData.model.getBodyId(param.LF_foot);
    dataSets.robotModelData.frfootID = dataSets.robotModelData.model.getBodyId(param.RF_foot);
    dataSets.robotModelData.hlfootID = dataSets.robotModelData.model.getBodyId(param.LH_foot);
    dataSets.robotModelData.hrfootID = dataSets.robotModelData.model.getBodyId(param.RH_foot);
    dataSets.robotModelData.baseID = dataSets.robotModelData.model.getBodyId(param.BASE);

    // TODO: comment?
    Estimator estimator(&dataSets.measuredState,
                        &dataSets.robotModelData,
                        &dataSets.estimatedState,
                        &dataSets.switchState,
                        &param);

    GaitScheduler gait(&dataSets.estimatedState,
                       &dataSets.userCmd,
                       &dataSets.gaitData,
                       &param);

    Planner planner(&dataSets.estimatedState,
                    &dataSets.robotModelData,
                    &dataSets.userCmd,
                    &dataSets.gaitData,
                    &dataSets.tasksData,
                    &dataSets.switchState,
                    &param);

    Controller controller(&dataSets.estimatedState,
                          &dataSets.robotModelData,
                          &dataSets.tasksData,
                          &dataSets.gaitData,
                          &dataSets.userCmd,
                          &dataSets.jointsCmd,
                          &dataSets.switchState,
                          &param);

    UserInterface userInterface(&param, &dataSets.userCmd);

Timer t;
    double time_init = t.getMs();
    size_t iter = 0;

    while (!exitrequest) {
            cout << std::setprecision(9) << "time: " << t.getMs() << endl;
            double time_control_start;
            time_control_start = t.getMs();
            dataSets.jointsCmd.tau_ff.setZero(); // set all initial motor input to zero
            userInterface.update_keyboard(); //receive user input from the joystick/keyboard
            estimator.estimate(); // estimate states and model

            gait.step();
            planner.plan();
            controller.run();
            double time_control_end;
            time_control_end = t.getMs();
            std::cout << "t: " << (time_control_end - time_control_start) << std::endl;
    }
}