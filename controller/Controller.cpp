//
// Created by wenchun on 3/17/21.
//

#include "Controller.h"

Controller::Controller(const EstimatedState *estimatedState,
                       const RobotModelData *robotModelData,
                       const TasksData *tasksData,
                       const GaitData *gaitData,
                       const UserCmd *userCmd,
                       JointsCmd *jointsCmd,
                       SwitchState *switchState,
                       const UserParameterHandler *param) : estimatedState(estimatedState),
                                                            robotModelData(robotModelData),
                                                            tasksData(tasksData),
                                                            gaitData(gaitData),
                                                            userCmd(userCmd),
                                                            jointsCmd(jointsCmd),
                                                            switchState(switchState),
                                                            param(param),
                                                            wbc(estimatedState, robotModelData, tasksData, gaitData,
                                                                jointsCmd, param),
                                                            legController(estimatedState, robotModelData, tasksData,
                                                                          gaitData, jointsCmd,
                                                                          userCmd, switchState, param) {
    qpos_init.resize(20);
    qpos_init.setZero();
    qpos_mid_des.resize(20);
    qpos_mid_des.setZero();
    qpos_footextend_des.resize(20);
    qpos_footextend_des.setZero();
}

void Controller::run() {
//    std::cout << "[ctrl_num_pre]: " << ctrl_num_pre << std::endl;
//    std::cout << "[param->ctrl_num]: " << param->ctrl_num << std::endl;

    // static int ctrl_num_pre=0;
    switch (param->ctrl_num) {
        case 0:
            break;
        case 1:
            legController.run();
            break;
        case 2:
            legController.run_foot();
            break;
        case 3:
            LeggedPdStance();
            break;
        case 10: // default in here
            break;
        default:
            throw std::runtime_error("selected controller is not implemented ...");
    }
    ctrl_num_pre = param->ctrl_num;
    _iter++;
}

void Controller::LeggedPdStance() {
    static int iter;
    static Vec16 qposDes;
    if (enter_stand) {
        iter = 0;
        enter_stand = false;
        qpos_init = estimatedState->jointsState.qpos;
//        qpos_mid_des = estimatedState->jointsState.qpos;

        // std::cout << estimatedState->jointsState.qpos << std::endl;

        for (int i(0); i < 4; i++)
            for (int j(0); j < 4; j++)
                qposDes(i * 4 + j) = param->qLegDes[j] * param->jointsSigns[i * 4 + j];
    }

    for (int i = 0; i < 4; i++) {
        jointsCmd->Kp.segment(4 * i, 4) << param->KP[0], param->KP[1], param->KP[2], param->KP[3];
        jointsCmd->Kd.segment(4 * i, 4) << param->KD[0], param->KD[1], param->KD[2], param->KD[3];
    }

    DVec qdes;
    double r;
    if (iter <= 1000) {
        r = double(iter) / 1000;
        if (r >= 1) r = 1;
        qdes = (1 - r) * qpos_init + r * qposDes;
//    } else if (iter > 500 && iter < 2500) {
//        r = double(iter - 500) / 2000;
//        if (r >= 1) r = 1;
//        qdes = (1 - r) * qpos_mid_des + r * qposDes;
    } else {
        qdes = qposDes;
    }

    jointsCmd->tau_ff = robotModelData->generalizedGravity.tail(16);

    for (int i(0); i < 16; i++) {
        double tau = robotModelData->generalizedGravity[6 + i] +
                     jointsCmd->Kp[i] * (qdes(i) - estimatedState->jointsState.qpos[i]) +
                     jointsCmd->Kd[i] * (0 - estimatedState->jointsState.qvel[i]);
        if (abs(tau) > 35 && (i % 5 != 3)) { // TODO: check the max tau for joint motor and wheel motor
            jointsCmd->tau_ff[i] = 35 * abs(tau) / tau;
        } else if (abs(tau) > 200 && (i % 5 == 3)) {
            jointsCmd->tau_ff[i] = 200 * abs(tau) / tau;
        } else {
            jointsCmd->tau_ff[i] = tau;
        }
    }
    iter++;
}

void Controller::printJointStates() {
    std::cout.precision(3); //  set output precision
    std::cout << "\033[2J\033[1;1H";
    std::cout << "Joint States: (" << _iter << ")" << std::endl;
    for (int i = 0; i < 4; i++)//打印各角度
    {
        printf("\tabad \thip\tknee\tfoot\twheel\n");
        printf("pos:\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\n",
               estimatedState->jointsState.qpos[i * 5 + 0],
               estimatedState->jointsState.qpos[i * 5 + 1],
               estimatedState->jointsState.qpos[i * 5 + 2],
               estimatedState->jointsState.qpos[i * 5 + 3],
               estimatedState->jointsState.qpos[i * 5 + 4]);
        printf("des:\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\n",
               jointsCmd->qpos_des[i * 5 + 0] * param->jointsSigns[i * 5 + 0],
               jointsCmd->qpos_des[i * 5 + 1] * param->jointsSigns[i * 5 + 1],
               jointsCmd->qpos_des[i * 5 + 2] * param->jointsSigns[i * 5 + 2],
               jointsCmd->qpos_des[i * 5 + 3] * param->jointsSigns[i * 5 + 3],
               jointsCmd->qpos_des[i * 5 + 4] * param->jointsSigns[i * 5 + 4]);
        printf("vel:\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\n",
               estimatedState->jointsState.qvel[i * 5 + 0],
               estimatedState->jointsState.qvel[i * 5 + 1],
               estimatedState->jointsState.qvel[i * 5 + 2],
               estimatedState->jointsState.qvel[i * 5 + 3],
               estimatedState->jointsState.qvel[i * 5 + 4]);
        printf("des:\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\n",
               jointsCmd->qvel_des[i * 5 + 0] * param->jointsSigns[i * 5 + 0],
               jointsCmd->qvel_des[i * 5 + 1] * param->jointsSigns[i * 5 + 1],
               jointsCmd->qvel_des[i * 5 + 2] * param->jointsSigns[i * 5 + 2],
               jointsCmd->qvel_des[i * 5 + 3] * param->jointsSigns[i * 5 + 3],
               jointsCmd->qvel_des[i * 5 + 4] * param->jointsSigns[i * 5 + 4]);
        printf("tau:\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\n",
               estimatedState->jointsState.tau[i * 5 + 0],
               estimatedState->jointsState.tau[i * 5 + 1],
               estimatedState->jointsState.tau[i * 5 + 2],
               estimatedState->jointsState.tau[i * 5 + 3],
               estimatedState->jointsState.tau[i * 5 + 4]);
        printf("tau_ff:\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\n",
               jointsCmd->tau_ff[i * 5 + 0],
               jointsCmd->tau_ff[i * 5 + 1],
               jointsCmd->tau_ff[i * 5 + 2],
               jointsCmd->tau_ff[i * 5 + 3],
               jointsCmd->tau_ff[i * 5 + 4]);
        printf("KP :\t%0.1f\t%0.1f\t%0.1f\t%0.1f\t%0.1f\n",
               jointsCmd->Kp[i * 5 + 0],
               jointsCmd->Kp[i * 5 + 1],
               jointsCmd->Kp[i * 5 + 2],
               jointsCmd->Kp[i * 5 + 3],
               jointsCmd->Kp[i * 5 + 4]);
        printf("KD :\t%0.1f\t%0.1f\t%0.1f\t%0.1f\t%0.1f\n",
               jointsCmd->Kd[i * 5 + 0],
               jointsCmd->Kd[i * 5 + 1],
               jointsCmd->Kd[i * 5 + 2],
               jointsCmd->Kd[i * 5 + 3],
               jointsCmd->Kd[i * 5 + 4]);
    }
}
