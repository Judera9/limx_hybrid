//
// Created by wenchun on 3/17/21.
//

#ifndef XIAOTIANHYBRID_CONTROLLER_H
#define XIAOTIANHYBRID_CONTROLLER_H

#include "DataSets.h"
#include "WholeBodyController.h"
#include "LegController.h"
#include "iomanip"

#define PI 3.14159

class Controller {
public:
    Controller(const EstimatedState *estimatedState,
               const RobotModelData *robotModelData,
               const TasksData *tasksData,
               const GaitData *gaitData,
               const UserCmd *userCmd,
               JointsCmd *jointsCmd,
               SwitchState *switchState,
               const UserParameterHandler *param);

    virtual void run();

private:
    const EstimatedState *estimatedState;
    const RobotModelData *robotModelData;
    const TasksData *tasksData;
    const GaitData *gaitData;
    const UserCmd *userCmd;
    JointsCmd *jointsCmd;
    SwitchState *switchState;
    const UserParameterHandler *param;

    WholeBodyController wbc;
    LegController legController;

    DVec qpos_init;
    DVec qpos_mid_des;
    DVec qpos_footextend_des;

    double hipOffset_f = -22. / 180 * PI;
    double kneeOffset_f = 65. / 180 * PI;// front leg
    double hipOffset_r = hipOffset_f - 0.03;
    double kneeOffset_r = kneeOffset_f - 0.1362;// rear leg
    double kneeOffset_wheel2foot = -36. / 180 * PI;

    bool enter_down;
    bool enter_stand;
    bool enter_switch;
    bool enter_swing;

    int ctrl_num_pre = 0;
    int _iter = 0;
    int _iter_buf = 0;

    double wheel_int[4] = {0};

    void LeggedPdStance();
    void printJointStates();
};

#endif //XIAOTIANHYBRID_CONTROLLER_H
