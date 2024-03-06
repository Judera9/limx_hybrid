//
// Created by wenchun on 3/17/21.
//

#ifndef XIAOTIANHYBRID_PLANNER_H
#define XIAOTIANHYBRID_PLANNER_H

#include "DataSets.h"
#include "FootPlanner.h"
#include "CoMPlannerMPC.h"
#include "orientation_tools.h"
#include "WheelCoMPlanner.h"

class Planner {
public:
  Planner(const EstimatedState *estimatedState, const RobotModelData *robotModelData,
          const UserCmd *userCmd, const GaitData *gaitData, TasksData *tasksData, 
          SwitchState *switchState, const UserParameterHandler *param);

  void plan();
  double time_mpc_solved;
  
private:
  const EstimatedState *estimatedState;
  const RobotModelData *robotModelData;
  const UserCmd *userCmd;
  const GaitData *gaitData;
  TasksData *tasksData;
  const UserParameterHandler *param;

  FootPlanner footstepPlanner;
  CoMPlannerMPC comPlanner;

  WheelCoMPlanner wheelCoMPlanner;

  size_t iter;
  
};


#endif //XIAOTIANHYBRID_PLANNER_H
