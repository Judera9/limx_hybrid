//
// Created by wenchun on 3/17/21.
//

#ifndef XIAOTIANHYBRID_ESTIMATOR_H
#define XIAOTIANHYBRID_ESTIMATOR_H

#include "DataSets.h"
#include "Timer.h"

class Estimator {
private:
    void orientationEstimator();

    void linearKFPositionVelocityEstimator();

    void computeModelData();

    void setup();

    void cheatMode();
    void legTestMode();

    const MeasuredState *measuredState;
    RobotModelData *robotModelData;
    EstimatedState *estimatedState;
    const SwitchState *switchState;
    const UserParameterHandler *param;

    // velocity and position estimation parameters
    Eigen::Matrix<double, 12, 1> _ps;
    Eigen::Matrix<double, 12, 1> _vs;
    Eigen::Matrix<double, 18, 18> _A;
    Eigen::Matrix<double, 18, 1> _xhat;
    Eigen::Matrix<double, 18, 18> _Q0;
    Eigen::Matrix<double, 18, 18> _P;
    Eigen::Matrix<double, 28, 28> _R0;
    Eigen::Matrix<double, 18, 3> _B;
    Eigen::Matrix<double, 28, 18> _C;

    Timer timer;

protected:
    bool _b_first_visit = true;
    Quat _ori_ini_inv;
    bool _useContactInvariant = true;

public:
    Estimator(const MeasuredState *measuredState, RobotModelData *robotModelData, EstimatedState *estimatedState,
              const SwitchState *switchState, const UserParameterHandler *param);

    void estimate();

    void setContactPhase(Vec4 &phase, bool useContactInvariant) {
        estimatedState->contactPhaseDes = phase;
        _useContactInvariant = useContactInvariant;
    }
};


#endif //XIAOTIANHYBRID_ESTIMATOR_H
