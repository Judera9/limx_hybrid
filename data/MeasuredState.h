//
// Created by wenchun on 3/17/21.
//

#ifndef XIAOTIANHYBRID_MEASUREDSTATE_H
#define XIAOTIANHYBRID_MEASUREDSTATE_H

#include "cppTypes.h"

struct CheatData{
    // joint_num_related_value-1
    Vec23 qpos;
    Vec22 qvel;
    double timeStep;
};

struct IMUData {
    Quat quat;
    Vec3 gyro;
    Vec3 acc;
};

struct JointsState {
    // joint_num_related_value-1
    Vec16 qpos;
    Vec16 qvel;
    Vec16 tau;
};

struct MeasuredState {
    IMUData imuData; // [w x y z]
    JointsState jointsState; //会传递给estimatedState->jointsState.qpos
    CheatData cheatData;
};


#endif //XIAOTIANHYBRID_MEASUREDSTATE_H
