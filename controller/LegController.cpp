//
// Created by nimapng on 7/23/21.
//

#include "LegController.h"
#include "Param.h"

#define FIX_WHEEL

LegController::LegController(const EstimatedState *estimatedState, const RobotModelData *robotModelData,
                             const TasksData *tasksData, const GaitData *gaitData, JointsCmd *jointsCmd,
                             const UserCmd *userCmd,
                             SwitchState *switchState, const UserParameterHandler *param) : estimatedState(
        estimatedState),
                                                                                            robotModelData(
                                                                                                    robotModelData),
                                                                                            tasksData(tasksData),
                                                                                            gaitData(gaitData),
                                                                                            jointsCmd(jointsCmd),
                                                                                            userCmd(userCmd),
                                                                                            switchState(switchState),
                                                                                            param(param) {
}

void LegController::run() {
//    for (int i = 0; i < 4; i++) {
//        switchState->foot_mode[i] = 1;
//    }
    Matrix<double, 12, 26> Jc_all;
    for (int i = 0; i < 4; i++) {
        Jc_all.block(3 * i, 0, 3, 26) = robotModelData->Je[i]; // use this jacobian is better
//         Jc_all.block(3 * i, 0, 3, 22) = robotModelData->Jc[i];
    }
    jointsCmd->tau_ff = -Jc_all.block(0, 6, 12, 20).transpose() * tasksData->forcesTaskData.forces_ref +
                        robotModelData->generalizedGravity.tail(20);
    // jointsCmd->tau_ff = robotModelData->generalizedGravity.tail(20);

    // Vec22 tau_ff;
    // tau_ff.setZero();
    Vec4Int footID;
    footID << robotModelData->flwID, robotModelData->frwID, robotModelData->hlwID, robotModelData->hrwID;

//    std::cout << "footID: \n" << footID << std::endl;
    // Vec4Int footID;
    // footID << robotModelData->flfootID, robotModelData->frfootID, robotModelData->hlfootID, robotModelData->hrfootID;
    /************************* swing ****************************/
    for (int i = 0; i < 4; i++) {
        if (gaitData->swingTimeRemain[i] > 0.) // swing
        {
            firstStance[i] = true;

            jointsCmd->Kp.segment(5 * i, 5) << param->KP[0], param->KP[1], param->KP[2], param->KP[3], param->KP[4];
            jointsCmd->Kd.segment(5 * i, 5) << param->KD[0], param->KD[1], param->KD[2], param->KD[3], param->KD[4];
            // jointsCmd->Kp.segment(5 * i, 5) << 1000, 300, 300, 10000, 0;
            // jointsCmd->Kd.segment(5 * i, 5) << 40., 10.0, 10.0, 100, 20.0;

            DMat J_inv = robotModelData->Je[i].middleCols(5 * i + 6, 3).inverse();
            Vec3 delta_q = J_inv * (tasksData->footTaskData[i].pos - robotModelData->data.oMf[footID(i)].translation());
            // std::cout << "des_foot" << i << "_pos: " << tasksData->footTaskData[i].pos.transpose() << std::endl;
            // std::cout << "current_foot" << i << "_pos: "
            //           << robotModelData->data.oMf[footID(i)].translation().transpose() << std::endl;
            // std::cout << "foot" << i << "_delta_q: " << delta_q.transpose() << std::endl;
            Vec3 qd_des = J_inv * (tasksData->footTaskData[i].vWorld - estimatedState->floatingBaseState.vWorld);
            jointsCmd->qpos_des.segment(5 * i, 3) = estimatedState->jointsState.qpos.segment(5 * i, 3) + delta_q;
            jointsCmd->qpos_des(5 * i + 3) = 0.0;
            jointsCmd->qpos_des(5 * i + 4) = estimatedState->jointsState.qpos(
                    5 * i + 4); // execute damping control for wheel
            jointsCmd->qvel_des.segment(5 * i, 3) = qd_des;
        }
    }

    /************************* stance ****************************/
    for (int i = 0; i < 4; i++) {
        if (gaitData->swingTimeRemain[i] <= 0.)// stance
        {
#ifdef FIX_WHEEL
            jointsCmd->Kp.segment(5 * i, 5) << 0, 0, 0, 100000., 100.;
            jointsCmd->Kd.segment(5 * i, 5) << 0.2, 0.2, 0.2, 100., 2.;
            // jointsCmd->Kp.segment(5 * i, 5) << 0, 0, 0, 10000., 1000.;
            // jointsCmd->Kd.segment(5 * i, 5) << 2., 2., 2., 100., 20.;

#else
            jointsCmd->Kp.segment(4 * i, 4).setZero();
            jointsCmd->Kd.segment(4 * i, 4).setZero();
#endif
            // fixed wheel when stance
            /* if (firstStance[i]) {
                jointsCmd->qpos_des(4 * i + 3) = estimatedState->jointsState.qpos(4 * i + 3);
                jointsCmd->qvel_des(4 * i + 3) = 0;
                firstStance[i] = false;
            } */

            auto vBodyDes = estimatedState->floatingBaseState.R_wb.transpose() * tasksData->comTaskData.vWorld;
            auto vBodyEs = estimatedState->floatingBaseState.vBody;
            jointsCmd->qvel_des(5 * i + 4) = (0.4 * vBodyEs[0] + 0.6 * vBodyDes[0]) / param->rw;
            // jointsCmd->qvel_des(4 * i + 3) = tasksData->forcesTaskData.vWheel_ref[i]/param->rw; // use unified planning
            // jointsCmd->qvel_des(4 * i + 3) = userCmd->vx_des/param->rw;
            jointsCmd->qpos_des(5 * i + 3) = 0.0;
            jointsCmd->qpos_des(5 * i + 4) =
                    estimatedState->jointsState.qpos(5 * i + 4) + jointsCmd->qvel_des(5 * i + 4) * param->dt;
        }
    }
    toRobot();
}

//void LegController::step_switch() {
//    static int switch_iter;
//    if (enter_step_switch) {
//        switch_iter = 0;
//        enter_step_switch = false;
//    }
//
//    Matrix<double, 12, 26> Jc_all;
//    for (int i = 0; i < 4; i++) {
//        Jc_all.block(3 * i, 0, 3, 26) = robotModelData->Je[i]; // use this jacobian is better
//        // Jc_all.block(3 * i, 0, 3, 22) = robotModelData->Jc[i];
//    }
//    jointsCmd->tau_ff = -Jc_all.block(0, 6, 12, 20).transpose() * tasksData->forcesTaskData.forces_ref +
//                        robotModelData->generalizedGravity.tail(20);
//    // jointsCmd->tau_ff = robotModelData->generalizedGravity.tail(20);
//
//
//    // Vec22 tau_ff;
//    // tau_ff.setZero();
//    Vec4Int wheelID;
//    wheelID << robotModelData->flwID, robotModelData->frwID, robotModelData->hlwID, robotModelData->hrwID;
//
//    Vec4Int footID;
//    footID << robotModelData->flfootID, robotModelData->frfootID, robotModelData->hlfootID, robotModelData->hrfootID;
//    /************************* swing ****************************/
//    for (int i = 0; i < 4; i++) {
//        if (gaitData->swingTimeRemain[i] > 0.) // swing
//        {
//            firstStance[i] = true;
//
//            jointsCmd->Kp.segment(5 * i, 5) << 100, 30, 30, 100000, 0;
//            jointsCmd->Kd.segment(5 * i, 5) << 4., 1.0, 1.0, 100, 2.0;
//            // jointsCmd->Kp.segment(5 * i, 5) << 1000, 300, 300, 10000, 0;
//            // jointsCmd->Kd.segment(5 * i, 5) << 40., 10.0, 10.0, 100, 20.0;
//
//            DMat J_inv = robotModelData->Je[i].middleCols(5 * i + 6, 3).inverse();
//
//            Vec3 delta_q;
//            if (switchState->foot_mode[i] == 0) {
//                delta_q = J_inv * (tasksData->footTaskData[i].pos - robotModelData->data.oMf[wheelID(i)].translation());
//            } else if (switchState->foot_mode[i] == 1) {
//                delta_q = J_inv * (tasksData->footTaskData[i].pos - robotModelData->data.oMf[footID(i)].translation());
//            }
//
////            std::cout << "des_foot" << i << "_pos: " << tasksData->footTaskData[i].pos.transpose() << std::endl;
////            std::cout << "current_foot" << i << "_pos: "
////                      << robotModelData->data.oMf[footID(i)].translation().transpose() << std::endl;
////            std::cout << "foot" << i << "_delta_q: " << delta_q.transpose() << std::endl;
//            Vec3 qd_des = J_inv * (tasksData->footTaskData[i].vWorld - estimatedState->floatingBaseState.vWorld);
//            jointsCmd->qpos_des.segment(5 * i, 3) = estimatedState->jointsState.qpos.segment(5 * i, 3) + delta_q;
//            jointsCmd->qpos_des(5 * i + 4) = estimatedState->jointsState.qpos(
//                    5 * i + 4); // execute damping control for wheel
//            jointsCmd->qvel_des.segment(5 * i, 3) = qd_des;
//        }
//    }
//
//    /************************* stance ****************************/
//    for (int i = 0; i < 4; i++) {
//        if (gaitData->swingTimeRemain[i] <= 0.)// stance
//        {
//#ifdef FIX_WHEEL
//            jointsCmd->Kp.segment(5 * i, 5) << 0, 0, 0, 100000., 100.;
//            jointsCmd->Kd.segment(5 * i, 5) << 0.2, 0.2, 0.2, 100., 2.;
//            // jointsCmd->Kp.segment(5 * i, 5) << 0, 0, 0, 10000., 1000.;
//            // jointsCmd->Kd.segment(5 * i, 5) << 2., 2., 2., 100., 20.;
//
//#else
//            jointsCmd->Kp.segment(4 * i, 4).setZero();
//            jointsCmd->Kd.segment(4 * i, 4).setZero();
//#endif
//            // fixed wheel when stance
//            /* if (firstStance[i]) {
//                jointsCmd->qpos_des(4 * i + 3) = estimatedState->jointsState.qpos(4 * i + 3);
//                jointsCmd->qvel_des(4 * i + 3) = 0;
//                firstStance[i] = false;
//            } */
//
//            auto vBodyDes = estimatedState->floatingBaseState.R_wb.transpose() * tasksData->comTaskData.vWorld;
//            auto vBodyEs = estimatedState->floatingBaseState.vBody;
//            jointsCmd->qvel_des(5 * i + 4) = (0.4 * vBodyEs[0] + 0.6 * vBodyDes[0]) / param->rw;
//            // jointsCmd->qvel_des(4 * i + 3) = tasksData->forcesTaskData.vWheel_ref[i]/param->rw; // use unified planning
//            // jointsCmd->qvel_des(4 * i + 3) = userCmd->vx_des/param->rw;
//            jointsCmd->qpos_des(5 * i + 4) =
//                    estimatedState->jointsState.qpos(5 * i + 4) + jointsCmd->qvel_des(5 * i + 4) * param->dt;
//        }
//    }
//
//    /************************** foot *****************************/
//    for (int i = 0; i < 4; i++) {
//        if (switch_iter <= 1500) {
//            double extend_ratio = (double) switch_iter / 1500.;
//            jointsCmd->qpos_des(5 * i + 3) = -0.005 + 0.085 * extend_ratio;
//        } else if (switch_iter > 2000) {
//            //currently designed for wheel-foot 2 point-foot
//            if (switchState->foot_mode[i] != 1) // 0 - wheel-foot mode   |   1 - point-foot mode
//            {
//                if (switchState->switch_handled[i] == 2) {
//                    switchState->switch_request[i] = 1; // 0 - wheel-foot switch request  |  1 - point-foot switch request  |  2 - no request
//                } else if (switchState->switch_handled[i] == 1) {
//                    switchState->foot_mode[i] = 1;
//                    switchState->switch_request[i] = 2;
//                }
//            }
//
//            jointsCmd->qpos_des(5 * i + 3) = 0.080;
//        }
//    }
//    std::cout << "foot_mode: " << switchState->foot_mode.transpose() << std::endl;
//    std::cout << "switch_request: " << switchState->switch_request.transpose() << std::endl;
//    toRobot();
//    switch_iter++;
//}

void LegController::run_foot() { // might be point-foot-mode
    Matrix<double, 12, 26> Jc_all;
    for (int i = 0; i < 4; i++) {
        Jc_all.block(3 * i, 0, 3, 26) = robotModelData->Je[i]; // use this jacobian is better
        // Jc_all.block(3 * i, 0, 3, 22) = robotModelData->Jc[i]; 
    }
    jointsCmd->tau_ff = -Jc_all.block(0, 6, 12, 20).transpose() * tasksData->forcesTaskData.forces_ref +
                        robotModelData->generalizedGravity.tail(20);
    // jointsCmd->tau_ff = robotModelData->generalizedGravity.tail(20);

    Vec4Int footID;
    footID << robotModelData->flfootID, robotModelData->frfootID, robotModelData->hlfootID, robotModelData->hrfootID;

    std::cout << "footID: \n" << footID << std::endl;
    /************************* swing ****************************/
    for (int i = 0; i < 4; i++) {
        if (gaitData->swingTimeRemain[i] > 0.) // swing
        {
            firstStance[i] = true;

            jointsCmd->Kp.segment(5 * i, 5) << param->KP[0], param->KP[1], param->KP[2], param->KP[3], param->KP[4];
            jointsCmd->Kd.segment(5 * i, 5) << param->KD[0], param->KD[1], param->KD[2], param->KD[3], param->KD[4];

            // jointsCmd->Kp.segment(5 * i, 5) << 100, 30, 30, 100000, 0;
            // jointsCmd->Kd.segment(5 * i, 5) << 4., 1.0, 1.0, 100, 2.0;
            // jointsCmd->Kp.segment(5 * i, 5) << 10000, 10000, 10000, 100000, 0;
            // jointsCmd->Kd.segment(5 * i, 5) << 40., 10.0, 10.0, 100, 20.0;

            DMat J_inv = robotModelData->Je[i].middleCols(5 * i + 6, 3).inverse();
            Vec3 delta_q = J_inv * (tasksData->footTaskData[i].pos - robotModelData->data.oMf[footID(i)].translation());
            std::cout << "des_foot" << i << "_pos: " << tasksData->footTaskData[i].pos.transpose() << std::endl;
            std::cout << "current_foot" << i << "_pos: "
                      << robotModelData->data.oMf[footID(i)].translation().transpose() << std::endl;
            std::cout << "foot" << i << "_delta_q: " << delta_q.transpose() << std::endl;
            Vec3 qd_des = J_inv * (tasksData->footTaskData[i].vWorld - estimatedState->floatingBaseState.vWorld);
            jointsCmd->qpos_des.segment(5 * i, 3) = estimatedState->jointsState.qpos.segment(5 * i, 3) + delta_q;
            jointsCmd->qpos_des(5 * i + 3) = -0.005;
            jointsCmd->qpos_des(5 * i + 4) = estimatedState->jointsState.qpos(
                    5 * i + 4); // execute damping control for wheel
            jointsCmd->qvel_des.segment(5 * i, 3) = qd_des;
        }
    }

    /************************* stance ****************************/
    for (int i = 0; i < 4; i++) {
        if (gaitData->swingTimeRemain[i] <= 0.)// stance
        {
#ifdef FIX_WHEEL
            jointsCmd->Kp.segment(5 * i, 5) << 0, 0, 0, 100000., 100.;
            jointsCmd->Kd.segment(5 * i, 5) << 0.2, 0.2, 0.2, 100., 2.;
            // jointsCmd->Kp.segment(5 * i, 5) << 0, 0, 0, 100000., 1000.;
            // jointsCmd->Kd.segment(5 * i, 5) << 2., 2., 2., 100., 20.;

#else
            jointsCmd->Kp.segment(4 * i, 4).setZero();
            jointsCmd->Kd.segment(4 * i, 4).setZero();
#endif
            // fixed wheel when stance
            /* if (firstStance[i]) {
                jointsCmd->qpos_des(4 * i + 3) = estimatedState->jointsState.qpos(4 * i + 3);
                jointsCmd->qvel_des(4 * i + 3) = 0;
                firstStance[i] = false;
            } */

            auto vBodyDes = estimatedState->floatingBaseState.R_wb.transpose() * tasksData->comTaskData.vWorld;
            auto vBodyEs = estimatedState->floatingBaseState.vBody;
            jointsCmd->qvel_des(5 * i + 4) = (0.4 * vBodyEs[0] + 0.6 * vBodyDes[0]) / param->rw;
            // jointsCmd->qvel_des(4 * i + 3) = tasksData->forcesTaskData.vWheel_ref[i]/param->rw; // use unified planning
            // jointsCmd->qvel_des(4 * i + 3) = userCmd->vx_des/param->rw;
            jointsCmd->qpos_des(5 * i + 3) = 0.0;
            jointsCmd->qpos_des(5 * i + 4) =
                    estimatedState->jointsState.qpos(5 * i + 4) + jointsCmd->qvel_des(5 * i + 4) * param->dt;
        }
    }
    toRobot();
}

// void LegController::run_original()
// {
//     Matrix<double, 12, 22> Jc_all;
//     for (int i = 0; i < 4; i++)
//     {
//         Jc_all.block(3 * i, 0, 3, 22) = robotModelData->Je[i]; // use this jacobian is better
//         // Jc_all.block(3 * i, 0, 3, 22) = robotModelData->Jc[i]; 
//     }
//     jointsCmd->tau_ff = -Jc_all.block(0, 6, 12, 16).transpose() * tasksData->forcesTaskData.forces_ref + robotModelData->generalizedGravity.tail(16);
//
//     Vec22 tau_ff;
//     tau_ff.setZero();
//     Vec4Int footID;
//     footID << robotModelData->flwID, robotModelData->frwID, robotModelData->hlwID, robotModelData->hrwID;
//     /************************* swing ****************************/
//     for (int i = 0; i < 4; i++)
//     {
//         if (gaitData->swingTimeRemain[i] > 0.) // swing
//         {
//             firstStance[i] = true;
//
//             jointsCmd->Kp.segment(4 * i, 4) << 100, 30, 30, 0;
//             jointsCmd->Kd.segment(4 * i, 4) << 4., 1.0, 1.0, 2.0;
//             DMat J_inv = robotModelData->Je[i].middleCols(i * 4 + 6, 3).inverse();
//             Vec3 delta_q = J_inv * (tasksData->footTaskData[i].pos - robotModelData->data.oMf[footID(i)].translation());
//             Vec3 qd_des = J_inv * (tasksData->footTaskData[i].vWorld - estimatedState->floatingBaseState.vWorld);
//             jointsCmd->qpos_des.segment(4 * i, 3) = estimatedState->jointsState.qpos.segment(4 * i, 3) + delta_q;
//             jointsCmd->qpos_des(4*i + 3) = estimatedState->jointsState.qpos(4*i + 3); // execute damping control for wheel
//             jointsCmd->qvel_des.segment(4 * i, 3) = qd_des;
//         }
//     }
//
//     /************************* stance ****************************/
//     for (int i = 0; i < 4; i++)
//     {
//         if (gaitData->swingTimeRemain[i] <= 0.)// stance
//         {
// #ifdef FIX_WHEEL
//             jointsCmd->Kp.segment(4 * i, 4) << 0, 0, 0, 100;
//             jointsCmd->Kd.segment(4 * i, 4) << 0.2, 0.2, 0.2, 2;
// #else
//             jointsCmd->Kp.segment(4 * i, 4).setZero();
//             jointsCmd->Kd.segment(4 * i, 4).setZero();
// #endif
//             // fixed wheel when stance
//             /* if (firstStance[i]) {
//                 jointsCmd->qpos_des(4 * i + 3) = estimatedState->jointsState.qpos(4 * i + 3);
//                 jointsCmd->qvel_des(4 * i + 3) = 0;
//                 firstStance[i] = false;
//             } */
//
//             auto vBodyDes = estimatedState->floatingBaseState.R_wb.transpose()*tasksData->comTaskData.vWorld;
//             auto vBodyEs = estimatedState->floatingBaseState.vBody;
//             jointsCmd->qvel_des(4 * i + 3) = (0.4*vBodyEs[0] + 0.6*vBodyDes[0])/param->rw;
//             // jointsCmd->qvel_des(4 * i + 3) = tasksData->forcesTaskData.vWheel_ref[i]/param->rw; // use unified planning
//             // jointsCmd->qvel_des(4 * i + 3) = userCmd->vx_des/param->rw;
//             jointsCmd->qpos_des(4 * i + 3) = estimatedState->jointsState.qpos(4 * i + 3) + jointsCmd->qvel_des(4 * i + 3) * param->dt;
//         }
//     }
//     toRobot();
// }

void LegController::toRobot() {
    for (int i(0); i < 20; i++) {
//        std::cout << "tau_ff = " << jointsCmd->tau_ff << std::endl;
        double tau = jointsCmd->tau_ff[i] +
                     jointsCmd->Kp[i] * (jointsCmd->qpos_des[i] - estimatedState->jointsState.qpos[i]) +
                     jointsCmd->Kd[i] *
                     (jointsCmd->qvel_des[i] - estimatedState->jointsState.qvel[i]);
        //    double tau = jointsCmd->tau_ff[i];
        if (abs(tau) > 40 & (i % 5 != 3)) {
            jointsCmd->tau_ff[i] = 40 * abs(tau) / tau;
        } else if (abs(tau) > 200 & (i % 5 == 3)) {
            jointsCmd->tau_ff[i] = 200 * abs(tau) / tau;
        } else if (i % 5 == 4) {  //TODO no control for wheel
            jointsCmd->tau_ff[i] = 0;
        } else {
            jointsCmd->tau_ff[i] = tau;
        }
    }
}

/* ---------- swing test -------------*/
// void LegController::swingTest(size_t leg)
// {
//     Vec4Int footID;
//     footID << robotModelData->flwID, robotModelData->frwID, robotModelData->hlwID, robotModelData->hrwID;
//     jointsCmd->tau_ff.setZero();
//     if (gaitData->swingTimeRemain[leg] > 0.)
//     {
//         if (first)
//         {
//             first = false;
//             footSwingTrajectory.setInitialPosition(
//                     estimatedState->footState[leg].pos - estimatedState->floatingBaseState.pos);
//             footSwingTrajectory.setFinalPosition(
//                     estimatedState->footState[leg].pos - estimatedState->floatingBaseState.pos);
//             footSwingTrajectory.setHeight(0.05);
//             std::cout << "s: "
//                       << (estimatedState->footState[leg].pos - estimatedState->floatingBaseState.pos).transpose()
//                       << std::endl;
//         }

//         double swingPhase = 1 - gaitData->swingTimeRemain[leg] / gaitData->swingTime[leg];
//         footSwingTrajectory.computeSwingTrajectoryBezier(swingPhase, gaitData->swingTime[leg]);

//         jointsCmd->Kp.segment(4 * leg, 4) << 30, 30, 30, 500;
//         jointsCmd->Kd.segment(4 * leg, 4) << 1.0, 1.0, 1.0, 5;
//         DMat J_inv = robotModelData->Je[leg].middleCols(leg * 4 + 6, 3).inverse();
//         Vec3 delta_q = J_inv * (footSwingTrajectory.getPosition() -
//                                 (robotModelData->data.oMf[footID(leg)].translation() -
//                                  estimatedState->floatingBaseState.pos));
//         Vec3 qd_des = J_inv * footSwingTrajectory.getVelocity();
//         jointsCmd->qpos_des.segment(4 * leg, 3) = estimatedState->jointsState.qpos.segment(4 * leg, 3) + delta_q;
//         jointsCmd->qvel_des.segment(4 * leg, 3) = qd_des;
//         for (int i = leg * 4; i < leg * 4 + 4; i++)
//         {
//             double tau = robotModelData->generalizedGravity[i] +
//                          jointsCmd->Kp[i] * (jointsCmd->qpos_des[i] - estimatedState->jointsState.qpos[i]) +
//                          jointsCmd->Kd[i] *
//                          (jointsCmd->qvel_des[i] - estimatedState->jointsState.qvel[i]);
//             if (abs(tau) > 40)
//             {
//                 jointsCmd->tau_ff[i] = 40 * abs(tau) / tau;
//             }
//             else
//             {
//                 jointsCmd->tau_ff[i] = tau;
//             }
//         }
//     }
//     else
//     {
//         jointsCmd->tau_ff.setZero();
//     }
// }

void LegController::swingTest() {
    int leg = param->leg_index;

    // std::cout << "nu: " << robotModelData->model.nu << std::endl;
    // for(int i(0); i < 4; i++)
    // {
    //     jointsCmd->tau_ff[5*leg+0] = robotModelData->generalizedGravity[6 + 5*leg];
    //     jointsCmd->tau_ff[5*leg+1] = robotModelData->generalizedGravity[6 + 5*leg + 1];
    //     jointsCmd->tau_ff[5*leg+2] = robotModelData->generalizedGravity[6 + 5*leg + 2];
    //     // jointsCmd->tau_ff[5*leg+3] = -5;
    //     // jointsCmd->tau_ff[5*leg+4] = param->des_tau[4];
    // }

    jointsCmd->tau_ff[5 * leg + 0] = robotModelData->generalizedGravity[6 + 5 * leg];
    jointsCmd->tau_ff[5 * leg + 1] = robotModelData->generalizedGravity[6 + 5 * leg + 1];
    jointsCmd->tau_ff[5 * leg + 2] = robotModelData->generalizedGravity[6 + 5 * leg + 2];

    // jointsCmd->tau_ff[5*leg+0] = param->des_tau[0];
    // jointsCmd->tau_ff[5*leg+1] = param->des_tau[1];
    // jointsCmd->tau_ff[5*leg+2] = param->des_tau[2];
    // jointsCmd->tau_ff[5*leg+3] = param->des_tau[3];
    // jointsCmd->tau_ff[5*leg+4] = param->des_tau[4];


    for (int i(0); i < 4; i++) {
        std::cout << "estimatedState" << i << ": " << estimatedState->jointsState.qpos.segment(5 * i, 3).transpose()
                  << std::endl;
    }
}

void LegController::swingTest_v2(double swingTime, bool enter_swing)//控制单个腿运动函数 参数swingtime是控制腿到达目标点位时间
{
    int leg = param->leg_index;//index对应控制腿的编号
    Vec4Int footID;
    footID << robotModelData->flwID, robotModelData->frwID, robotModelData->hlwID, robotModelData->hrwID;
    jointsCmd->tau_ff.setZero();

    if (enter_swing) {
        timeStamp_init = getTimeStamp();
        // joint_num_related_value
        _p0 = robotModelData->data.oMf[footID(leg)].translation() - estimatedState->floatingBaseState.pos;
        // _p0 = robotModelData->data.oMf[footID(leg)].translation() - p_torso;
        Vec3 des_p;
        des_p[0] = param->des_pos[0];
        des_p[1] = param->des_pos[1];
        des_p[2] = param->des_pos[2];
        _pf = des_p;
    }

    Vec3 des_p;
    des_p[0] = param->des_pos[0];
    des_p[1] = param->des_pos[1];
    des_p[2] = param->des_pos[2];
    // des_v[0] = param->des_vel[0];
    // des_v[1] = param->des_vel[1];
    // des_v[2] = param->des_vel[2];
    double phase = (getTimeStamp() - timeStamp_init) / swingTime;//phase相当于进度，0-1

    linearInterpolation(_p0, _pf, phase, swingTime);
    //(checked)
    // std::cout << "s: " << (estimatedState->footState[leg].pos - estimatedState->floatingBaseState.pos).transpose() << std::endl;

    jointsCmd->Kp.segment(5 * leg, 5)
            << param->KP[0], param->KP[1], param->KP[2], param->KP[3], param->KP[4];//a, h, k, f, w 调整各个关节的Kp及Kd 最后一个是wheel
    jointsCmd->Kd.segment(5 * leg, 5) << param->KD[0], param->KD[1], param->KD[2], param->KD[3], param->KD[4];
    DMat J_inv = robotModelData->Je[leg].middleCols(leg * 5 + 6, 3).inverse();

    Vec3 delta_q = J_inv * (g_pos -
                            (robotModelData->data.oMf[footID(leg)].translation() -
                             estimatedState->floatingBaseState.pos));// joint_num_related_value
    Vec3 qd_des = J_inv * g_vel;
    jointsCmd->qpos_des.segment(5 * leg, 3) = estimatedState->jointsState.qpos.segment(5 * leg, 3) + delta_q;
    jointsCmd->qvel_des.segment(5 * leg, 3) = qd_des;

    // iter+=1;
    std::cout << "iter: " << iter << " phase: " << phase << "\ng_pos: " << g_pos.transpose() << " g_vel: "
              << g_vel.transpose() << std::endl;

    std::cout << "pos: " << (robotModelData->data.oMf[footID(leg)].translation() -
                             estimatedState->floatingBaseState.pos).transpose() << std::endl;
    std::cout << "estimatedState: " << estimatedState->jointsState.qpos.segment(5 * leg, 3).transpose() << std::endl;
    std::cout << "delta_q: " << delta_q.transpose() << std::endl;
    std::cout << "generalizedGravity: " << robotModelData->generalizedGravity.segment(6 + leg * 5, 5).transpose()
              << std::endl;

    for (int i = leg * 5; i < leg * 5 + 4; i++) {
        double tau =
                robotModelData->generalizedGravity[6 + i] +
                jointsCmd->Kp[i] * (jointsCmd->qpos_des[i] - estimatedState->jointsState.qpos[i]) +
                jointsCmd->Kd[i] *
                (jointsCmd->qvel_des[i] - estimatedState->jointsState.qvel[i]);// joint_num_related_value
        if (abs(tau) > 40) {
            jointsCmd->tau_ff[i] = 40 * abs(tau) / tau;
        } else {
            jointsCmd->tau_ff[i] = tau;
        }
    }

    jointsCmd->tau_ff[leg * 5 + 3] = -5.0;

    std::cout << "tau[0]: " << jointsCmd->tau_ff[leg * 5] << "| tau[1]: " << jointsCmd->tau_ff[leg * 5 + 1]
              << "| tau[2]: " << jointsCmd->tau_ff[leg * 5 + 2] << std::endl;
    // std::cout << "generalizedGravity: " << robotModelData->generalizedGravity.segment(6+leg*5, 5).transpose() << std::endl;


    // jointsCmd->tau_ff[leg*4] = param->des_tau[0];
    // jointsCmd->tau_ff[leg*4+1] = param->des_tau[1];
    // jointsCmd->tau_ff[leg*4+2] = param->des_tau[2];
    // jointsCmd->tau_ff[leg*4+3] = param->des_tau[3];
    // jointsCmd->tau_ff.setZero();
}

void LegController::swingTest_v3(double swingTime) {
    int leg = param->leg_index;

    // std::cout << "nu: " << robotModelData->model.nu << std::endl;

    // jointsCmd->tau_ff[5*leg+0] = robotModelData->generalizedGravity[6 + 5*leg];
    // jointsCmd->tau_ff[5*leg+1] = robotModelData->generalizedGravity[6 + 5*leg + 1];
    // jointsCmd->tau_ff[5*leg+2] = robotModelData->generalizedGravity[6 + 5*leg + 2];
    // jointsCmd->tau_ff[5*leg+3] = -5;
    // jointsCmd->tau_ff[5*leg+4] = param->des_tau[4];

    // jointsCmd->tau_ff[5*leg+0] = param->des_tau[0];
    // jointsCmd->tau_ff[5*leg+1] = param->des_tau[1];
    // jointsCmd->tau_ff[5*leg+2] = param->des_tau[2];
    // jointsCmd->tau_ff[5*leg+3] = param->des_tau[3];
    // jointsCmd->tau_ff[5*leg+4] = param->des_tau[4];

    for (int i(0); i < 4; i++) {
        std::cout << "estimatedState" << i << ": " << estimatedState->jointsState.qpos.segment(5 * i, 3).transpose()
                  << std::endl;
    }
    // std::cout << "footExtendLen_fl: " << estimatedState->jointsState.qpos[3]
    // << "\nfootExtendLen_fr: " << estimatedState->jointsState.qpos[8]
    // << "\nfootExtendLen_rl: " << estimatedState->jointsState.qpos[13]
    // << "\nfootExtendLen_rr: " << estimatedState->jointsState.qpos[18]
    std::cout << "\ngeneralizedGravity: " << robotModelData->generalizedGravity.segment(6 + leg * 5, 5).transpose()
              << std::endl;
}

void LegController::linearInterpolation(Vec3 p0, Vec3 pf, double phase, double swingTime) {
    if (phase > double(1.0)) {
        g_vel.setZero();
        g_pos = pf;
        return;
    }
    double dis = (pf - p0).norm();
    double m_pos, m_vel, m_acc;
    m_acc = dis / pow(swingTime / 2, 2);
    if (phase < double(0.5)) {
        m_vel = m_acc * phase * swingTime;
        m_pos = 0.5 * m_acc * pow(phase * swingTime, 2);
    } else {
        m_vel = m_acc * ((1 - phase) * swingTime);
        m_pos = m_acc * pow(0.5 * swingTime, 2) - 0.5 * m_acc * pow((1 - phase) * swingTime, 2);
    }
    g_vel = (pf - p0).normalized() * m_vel;
    double ratio_dis = m_pos / dis;
    g_pos = (1 - ratio_dis) * p0 + ratio_dis * pf;
}

