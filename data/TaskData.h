//
// Created by wenchun on 3/17/21.
//

#ifndef XIAOTIANHYBRID_TASKDATA_H
#define XIAOTIANHYBRID_TASKDATA_H

#include "cppTypes.h"

struct CoMTaskData {
    Vec3 pos, rpy, vWorld, omegaWorld, linAccWorld, angAccWorld;
};

struct FootTaskData {
    Vec3 pos, rpy, vWorld, omegaWorld, linAccWorld, angAccWorld;
    ContactState contactState;
};

struct ForcesTaskData {
    Vec12 forces_ref;
    Vec4 vWheel_ref;
};

struct JointsTaskData {
    Vec16 qpos, qvel, tau;
};

struct TasksData {
    FootTaskData footTaskData[4];
    CoMTaskData comTaskData;
    ForcesTaskData forcesTaskData;
    JointsTaskData jointsTaskData;
};
#endif //XIAOTIANHYBRID_TASKDATA_H
