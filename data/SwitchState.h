//
// Created by wenchun on 3/17/21.
//

#ifndef XIAOTIANHYBRID_SWITCHSTATE_H
#define XIAOTIANHYBRID_SWITCHSTATE_H

#include "cppTypes.h"

struct SwitchState {
    Vec4Int foot_mode = Vec4Int::Ones(); // 0 - wheel-foot mode   |   1 - point-foot mode // TODO: change initial state to point-foot mode
    Vec4Int switch_request = {2, 2, 2, 2}; // 0 - wheel-foot switch request  |  1 - point-foot switch request  |  2 - no request
    Vec4Int switch_handled = {2, 2, 2, 2}; // 0 - wheel-foot switch handled  |  1 - point-foot switch handled  |  2 - no handled record
};

#endif
