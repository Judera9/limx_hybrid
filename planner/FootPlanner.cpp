//
// Created by wenchun on 3/22/21.
//

#include "FootPlanner.h"
#include "orientation_tools.h"

using namespace ori;

FootPlanner::FootPlanner(const EstimatedState *estimatedState,
                         const RobotModelData *robotModelData,
                         const UserCmd *userCmd,
                         const GaitData *gaitData, //Question:GaitData里的swingTime和stance
                         FootTaskData *footTaskData,
                         SwitchState *switchState,
                         const UserParameterHandler *param):
        estimatedState(estimatedState),    // 赋值操作
        robotModelData(robotModelData),
        userCmd(userCmd),
        gaitData(gaitData),
        footTaskData(footTaskData),
        switchState(switchState),
        param(param) {
    for (int i(0); i < 4; i++) {
        firstSwing[i] = true;
        // i < 2 means front legs, (i == 0 || i == 2) means left legs
        pHipBody[i] = Vec3((i < 2) ? 0.2836 : -0.2836, (i == 0 || i == 2) ? 0.1768 : -0.1768, 0.);
    }

    vBodyDes.setZero();
}

void FootPlanner::plan() { // TODO: check what is pHipBody
    // 0 - wheel-foot mode   |   1 - point-foot mode
    for (int i(0); i < 4; i++) {
        if (switchState->foot_mode[i] == 0){
            pHipBody[i] = Vec3((i < 2) ? 0.2836 : -0.2836, (i == 0 || i == 2) ? 0.2128 : -0.2128, 0.);
        }
        else if (switchState->foot_mode[i] == 1){
            pHipBody[i] = Vec3((i < 2) ? 0.2836 : -0.2836, (i == 0 || i == 2) ? 0.1768 : -0.1768, 0.);
        }
//        std::cout << switchState->foot_mode[i] << std::endl;
    }

    //FootSwingTrajectory: p0 pf p v a
    if (firstRun) { // the initial state equals the final state
        for (int i = 0; i < 4; i++) {
            footSwingTrajectory[i].setHeight(0.05); // TODO: check why 0.05
            footSwingTrajectory[i].setInitialPosition(estimatedState->footState[i].pos);
            footSwingTrajectory[i].setFinalPosition(estimatedState->footState[i].pos);
        }
        firstRun = false;
    }

    vBodyDes = 0.8 * vBodyDes + 0.2 * Vec3(userCmd->vx_des, userCmd->vy_des, 0); // userCmd: vx = vy = 0
    Vec3 vWorldDes = estimatedState->floatingBaseState.R_wb * vBodyDes;

    // compute foot placement
    for (int i(0); i < 4; i++) {
        footSwingTrajectory[i].setHeight(0.05);

        double stanceTime = gaitData->nextStanceTime[i];
        Vec3 pYawCorrected =
                coordinateRotation(CoordinateAxis::Z, -userCmd->yawd_des * gaitData->swingTimeRemain[i]) * pHipBody[i];// TODO: what?

        Vec3 Pf = estimatedState->floatingBaseState.pos + estimatedState->floatingBaseState.R_wb * (pYawCorrected
                                                                                                    + estimatedState->floatingBaseState.vBody *
                                                                                                      gaitData->swingTimeRemain[i]); // TODO: body frame & world frame

        double p_rel_max = 0.3f;

        // Using the estimated velocity is correct
        /*double pfx_rel = estimatedState->floatingBaseState.vWorld[0] * 0.5 * gaitData->nextStanceTime[i] +
                         .03f * (estimatedState->floatingBaseState.vWorld[0] - vWorldDes[0]) +
                         (0.5f * estimatedState->floatingBaseState.pos[2] / 9.81f) *
                         (estimatedState->floatingBaseState.vWorld[1] * userCmd->yawd_des);*/

        double pfx_rel = vWorldDes[0] * 0.3 * gaitData->nextStanceTime[i] +
                         .1f * (estimatedState->floatingBaseState.vWorld[0] - vWorldDes[0]) +
                         sqrt(estimatedState->floatingBaseState.pos[2] / 9.81f) *
                         (estimatedState->floatingBaseState.vWorld[1] * userCmd->yawd_des);

//        std::cout << "pfx_rel: " << pfx_rel << ", " << userCmd->vx_des * 0.5 * gaitData->nextStanceTime[i] << std::endl;

        double pfy_rel = vWorldDes[1] * 0.3 * gaitData->nextStanceTime[i] +
                         .1f * (estimatedState->floatingBaseState.vWorld[1] - vWorldDes[1]) +
                         (0.5f * estimatedState->floatingBaseState.pos[2] / 9.81f) *
                         (-estimatedState->floatingBaseState.vWorld[0] * userCmd->yawd_des);

        pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
        pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
        Pf[0] += pfx_rel;
        Pf[1] += pfy_rel;
//        std::cout << "footmode: " << switchState->foot_mode[0] << std::endl;
        if ( switchState->foot_mode[i] == 1 )// 0 - wheel-foot mode   |   1 - point-foot mode
        {
            Pf[2] = 0.025; // TODO: why Pz = 0.04
        }
        else
        {
            Pf[2] = param->rw;
        }

        footSwingTrajectory[i].setFinalPosition(Pf);
//        std::cout << "Foot" << i << " final pos: " << Pf.transpose() << std::endl;
    }

    // generating foot trajectory
    for (int i = 0; i < 4; i++) {
        if (gaitData->swingTimeRemain[i] > 0.) // swing
        {
            if (firstSwing[i]) {
                firstSwing[i] = false;
                footSwingTrajectory[i].setInitialPosition(estimatedState->footState[i].pos);
//                std::cout << "Foot" << i << " initial pos: " << estimatedState->footState[i].pos.transpose() << std::endl;
            }

            double swingPhase = 1 - gaitData->swingTimeRemain[i] / gaitData->swingTime[i];
//            std::cout << "swingPhase: " << swingPhase << std::endl;

            if (switchState->switch_request[i] == 1 & swingPhase == 0.)
            {
                switchState->switch_handled[i] = 1;
                firstSwing[i] = true;
            }
            else if (switchState->switch_request[i] == 2)
            {
                switchState->switch_handled[i] = 2;
            }

            footSwingTrajectory[i].computeSwingTrajectoryBezier(swingPhase, gaitData->swingTime[i]);

            footTaskData[i].pos = footSwingTrajectory[i].getPosition();
            footTaskData[i].vWorld = footSwingTrajectory[i].getVelocity();
            footTaskData[i].linAccWorld = footSwingTrajectory[i].getAcceleration();
        } else // stance
        {
            firstSwing[i] = true;
            footTaskData[i].pos = footSwingTrajectory[i].getPosition();
            footTaskData[i].vWorld = footSwingTrajectory[i].getVelocity();

            /*footTaskData[i].pos = estimatedState->footState[i].pos;
            footTaskData[i].vWorld = footSwingTrajectory[i].getVelocity();*/
            footTaskData[i].linAccWorld = Vec3::Zero();
        }
    }
}
