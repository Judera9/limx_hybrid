//
// Created by wenchun on 3/17/21.
//

#ifndef XIAOTIANHYBRID_DATASETS_H
#define XIAOTIANHYBRID_DATASETS_H

#include "JointsCmd.h"
#include "EstimatedState.h"
#include "MeasuredState.h"
#include "SwitchState.h"
#include "datatypes.h"
#include "TaskData.h"
#include "UserCmd.h"
#include "RobotModelData.h"
#include "GaitData.h"
#include "SharedMemoryInterface.h"
#include "Param.h" // TODO: adjust UserParameterHandler

class DataSets {
public:
    DataSets() {
        // sharedMemory.attach(XIAOTIAN_SHARED_MEMORY_NAME);
        // sharedMemory().init();
        gaitData.zero();
    }

    // SharedMemoryObject<SharedMemoryInterface> sharedMemory; // do not use shared memory anymore
    
    EstimatedState estimatedState;
    SwitchState switchState;
    RobotModelData robotModelData;
    TasksData tasksData;
    UserCmd userCmd;
    GaitData gaitData;
};

#endif //XIAOTIANHYBRID_DATASETS_H
