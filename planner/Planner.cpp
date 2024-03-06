//
// Created by wenchun on 3/17/21.
//

#include "Planner.h"
#include "Timer.h"

// #define USE_WHEEL_BASE_PLANNER

Planner::Planner(const EstimatedState *estimatedState,
                 const RobotModelData *robotModelData,
                 const UserCmd *userCmd,
                 const GaitData *gaitData,
                 TasksData *tasksData,
                 SwitchState *switchState,
                 const UserParameterHandler *param) :
        estimatedState(estimatedState),
        robotModelData(robotModelData),
        userCmd(userCmd),
        gaitData(gaitData),
        tasksData(tasksData),
        param(param),
        footstepPlanner(estimatedState, robotModelData, userCmd, gaitData, tasksData->footTaskData, switchState, param),
        comPlanner(estimatedState, robotModelData, userCmd, gaitData, &tasksData->forcesTaskData,
                   &tasksData->comTaskData, param),
        wheelCoMPlanner(estimatedState, robotModelData, userCmd, gaitData, &tasksData->forcesTaskData,
                   &tasksData->comTaskData, param),
        iter(0)
{

}

void Planner::plan()
{

    footstepPlanner.plan();

#ifdef USE_WHEEL_BASE_PLANNER
    wheelCoMPlanner.generateHighLevelRef();
#else
    comPlanner.generateHighLevelRef();
#endif

    if (iter % 10 == 0) // TODO: iteration time is 0.01
    {
        Timer t;

#ifdef USE_WHEEL_BASE_PLANNER
        wheelCoMPlanner.plan();
#else
        comPlanner.plan();
#endif
        time_mpc_solved = t.getMs();
        // printf("mpc solved time %f ms\n", time_mpc_solved); // TODO
        // if(userCmd->gaitNum==2)
        //     getchar();
    }

#ifdef USE_WHEEL_BASE_PLANNER
    wheelCoMPlanner.updateTaskData();
#else
    comPlanner.updateTaskData();
#endif

    iter++;
}