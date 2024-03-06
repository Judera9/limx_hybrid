//
// Created by wenchun on 3/17/21.
//

#ifndef XIAOTIANHYBRID_JOINTSCMD_H
#define XIAOTIANHYBRID_JOINTSCMD_H

#include "cppTypes.h"

struct JointsCmd {
    // joint_num_related_value-1
    Vec16 Kp = Vec16::Zero();
    Vec16 Kd = Vec16::Zero();
    Vec16 qpos_des = Vec16::Zero();
    Vec16 qvel_des = Vec16::Zero();
    Vec16 tau_ff = Vec16::Zero();

};

#endif //XIAOTIANHYBRID_JOINTSCMD_H
