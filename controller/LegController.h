//
// Created by nimapng on 7/23/21.
//

#ifndef XIAOTIANHYBRID_LEGCONTROLLER_H
#define XIAOTIANHYBRID_LEGCONTROLLER_H

#include "DataSets.h"
#include "FootSwingTrajectory.h"
#include "orientation_tools.h"
#include "qpOASES.hpp"
#include <chrono>

using namespace Eigen;

class LegController {
public:
    LegController(const EstimatedState *estimatedState,
                  const RobotModelData *robotModelData,
                  const TasksData *tasksData,
                  const GaitData *gaitData,
                  JointsCmd *jointsCmd,
                  const UserCmd *userCmd,
                  SwitchState *switchState,
                  const UserParameterHandler *param);

    void run();

//    void step_switch();
//    bool enter_step_switch;

    void run_foot();

    void toRobot();

    void swingTest();

    void swingTest_v2(double swingTime, bool enter_swing);

    void swingTest_v3(double swingTime);

    void linearInterpolation(Vec3 _p0, Vec3 _pf, double phase, double swingTime);

    std::time_t getTimeStamp()
    {
        std::chrono::time_point<std::chrono::system_clock,std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        auto tmp=std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
        std::time_t timestamp = tmp.count();
        //std::time_t timestamp = std::chrono::system_clock::to_time_t(tp);
        return timestamp;
    }

private:
    const EstimatedState *estimatedState;
    const RobotModelData *robotModelData;
    const TasksData *tasksData;
    const GaitData *gaitData;
    JointsCmd *jointsCmd;
    const UserCmd *userCmd;
    SwitchState *switchState;
    const UserParameterHandler *param;

    /* swing test */
    bool first = true;
    FootSwingTrajectory footSwingTrajectory;
    bool firstStance[4] = {true};
    Vec3 g_pos, g_vel;
    Vec3 _p0, _pf;
    time_t timeStamp_init = -1;
    int iter = 0;
};


#endif //XIAOTIANHYBRID_LEGCONTROLLER_H
