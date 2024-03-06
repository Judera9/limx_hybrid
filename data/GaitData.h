//
// Created by wenchun on 3/17/21.
//

#ifndef XIAOTIANHYBRID_GAITDATA_H
#define XIAOTIANHYBRID_GAITDATA_H

#include "cppTypes.h"

struct GaitData {
    void zero() {
        swingTime.setZero();
        nextStanceTime.setZero();
        swingTimeRemain.setZero();
        stanceTimeRemain.setZero();
    }

    Vec4 swingTime; // keep constant for each leg
    Vec4 nextStanceTime; // may change
    Vec4 swingTimeRemain;
    Vec4 stanceTimeRemain;
    
    int gaitNum = 0;
};

#endif //XIAOTIANHYBRID_GAITDATA_H
