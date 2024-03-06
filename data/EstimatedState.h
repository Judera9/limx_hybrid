/**
 * @file    EstimatedState.h
 * @brief   FloatingBaseState decides 
 *
 * The main method works as the following order. First,
 *
 * @author  Junde
 * @date    2024-02-26
 * @version 1.0
 */

#ifndef XIAOTIANHYBRID_ESTIMATESTATE_H
#define XIAOTIANHYBRID_ESTIMATESTATE_H

#include "MeasuredState.h"


struct FloatingBaseState {
    Vec3 pos, rpy, vWorld, vBody, omegaWorld, omegaBody, aBody, aWorld;
    Mat3 R_wb;
    Quat quat;
};

struct FootState {
    Vec3 pos, rpy, vWorld, omegaWorld;
    Mat3 R_wf, R_wh;
    ContactState contactState;
};

enum class Feet {
    FL, FR, HL, HR
};

struct EstimatedState {
    FloatingBaseState floatingBaseState;
    JointsState jointsState;  //change to Jointstate
    FootState footState[4];

    Vec4 contactPhaseDes = Vec4::Zero();
};
#endif //XIAOTIANHYBRID_ESTIMATESTATE_H
