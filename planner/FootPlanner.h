//
// Created by wenchun on 3/22/21.
//

#ifndef XIAOTIANHYBRID_FOOTPLANNER_H
#define XIAOTIANHYBRID_FOOTPLANNER_H

#include "DataSets.h"
#include "FootSwingTrajectory.h"

class FootPlanner {
public:
    FootPlanner(const EstimatedState *estimatedState, const RobotModelData *robotModelData,
                const UserCmd *userCmd, const GaitData *gaitData, FootTaskData *footTaskData,
                SwitchState *switchState, const UserParameterHandler *param);

    void plan();

private:
    const EstimatedState *estimatedState;
    const RobotModelData *robotModelData;
    const UserCmd *userCmd;
    const GaitData *gaitData;
    FootTaskData *footTaskData;
    SwitchState *switchState;
    const UserParameterHandler *param;

    FootSwingTrajectory footSwingTrajectory[4];
    Vec3 pHipBody[4];
    bool firstRun = true;
    bool firstSwing[4];
    Vec3 vBodyDes;
};


#endif //XIAOTIANHYBRID_FOOTPLANNER_H
