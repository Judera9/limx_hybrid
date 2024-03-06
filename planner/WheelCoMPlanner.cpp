//
// Created by avenger on 22-4-27.
//

#include "WheelCoMPlanner.h"
#include "orientation_tools.h"

#define BIG_NUMBER 5e10
#define PRINT_PROBLEM_DATA

WheelCoMPlanner::WheelCoMPlanner(const EstimatedState *estimatedState,
                             const RobotModelData *robotModelData,
                             const UserCmd *userCmd,
                             const GaitData *gaitData,
                             ForcesTaskData *forcesTaskData,
                             CoMTaskData *comTaskData,
                             const UserParameterHandler *param) : estimatedState(estimatedState),
                                                                  robotModelData(robotModelData),
                                                                  userCmd(userCmd),
                                                                  gaitData(gaitData),
                                                                  forcesTaskData(forcesTaskData),
                                                                  comTaskData(comTaskData),
                                                                  param(param),
                                                                  convexWSRBmpc(param->horizons, param->dtMPC),
                                                                  _iter(0)
{
    rpInt.setZero();
    rpCom.setZero();
    trajInit.setZero();

    x0.setZero();

    X_d.resize(25 * param->horizons, Eigen::NoChange);
    U_b.resize(20 * param->horizons, Eigen::NoChange);
    contactTable.resize(4 * param->horizons, Eigen::NoChange);
    fmat.resize(20 * param->horizons, 12 * param->horizons);

    X_d.setZero();
    U_b.setZero();
    contactTable.setZero();
    fmat.setZero();

    q_soln.resize(12 * param->horizons);
    q_soln.setZero();
    // TODO add inertia

    /* xiaotian-wheel */
    mass = 33.8703;
    Ibody <<  0.829087,  -0.00198788,   -0.0686223,
            -0.00198788,       2.4131, -8.53912e-05,
            -0.0686223, -8.53912e-05,      2.51353;


    Wheel_SRB_MPC::Vec Qx(12);
    Qx.segment(0, 3) << param->Q_rpy[0], param->Q_rpy[1], param->Q_rpy[2];
    Qx.segment(3, 3) << param->Q_pos[0], param->Q_pos[1], param->Q_pos[2];
    Qx.segment(6, 3) << param->Q_omega[0], param->Q_omega[1], param->Q_omega[2];
    Qx.segment(9, 3) << param->Q_vel[0], param->Q_vel[1], param->Q_vel[2];
    
    Wheel_SRB_MPC::Vec2 Wxy(100, 100);
    convexWSRBmpc.setWeight(Qx, 4e-5, Wxy);

    Wheel_SRB_MPC::Vec8 hipRef;
    hipRef << 0.2836, 0.1768, 0.2836, -0.1768, -0.2836, 0.1768, -0.2836, -0.1768;
    convexWSRBmpc.setHipPosition(hipRef);

    convexWSRBmpc.setMassAndInertia(mass, Ibody);
    convexWSRBmpc.setMaxForce(200);
    _ext_wrench.setZero();
    vBodyDes.setZero();

    bodyHeight = param->height;
}

void WheelCoMPlanner::setExternalWrench(Vec6 wrench)
{
    _ext_wrench = wrench;
}

void WheelCoMPlanner::plan()
{
    x0 << estimatedState->floatingBaseState.rpy, estimatedState->floatingBaseState.pos,
            estimatedState->floatingBaseState.omegaWorld, estimatedState->floatingBaseState.vWorld, -9.81, estimatedState->footState[0].pos, estimatedState->footState[1].pos,estimatedState->footState[2].pos, estimatedState->footState[3].pos;
    convexWSRBmpc.setCurrentState(x0);
    generateRefTraj();
    convexWSRBmpc.setDesiredDiscreteTrajectory(X_d);
    generateContactTable();
    Matrix<int, -1, -1> contactTable_4h(4, param->horizons);
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < param->horizons; j++)
        {
            contactTable_4h(i, j) = contactTable(4 * j + i);
        }
    }
    convexWSRBmpc.setContactTable(contactTable_4h);

    convexWSRBmpc.setExternalWrench(_ext_wrench);

    Vec3 r;
    vector<Vec3> cp;
    for (int leg(0); leg < 4; leg++)
    {
        r = estimatedState->footState[leg].pos;
        // std::cout << "r " << leg << ": " << r.transpose() << "\n";
        cp.push_back(r);
    }
    convexWSRBmpc.setContactPointPos(cp);
    convexWSRBmpc.solve(0.0);
    auto OptInputs = convexWSRBmpc.getCurrentDesiredControlInputs();
    std::cout << "MPC Input: \n"
              << OptInputs.transpose() << "\n";
    for (int i(0); i < 4; i++) {
        forcesTaskData->forces_ref.segment(3*i,3) = OptInputs.segment(4*i,3);
        forcesTaskData->vWheel_ref(i) = OptInputs(4*i + 3);
    }
    updateTaskData();
    _ext_wrench.setZero();
    _iter++;
}

void WheelCoMPlanner::generateHighLevelRef()
{
    Vec3 _vBodyDes = 0.8 * vBodyDes + 0.2 * Vec3(userCmd->vx_des, userCmd->vy_des, 0);
    Vec3 vWorldDes = estimatedState->floatingBaseState.R_wb * _vBodyDes;
    yawdDes = 0.8 * yawdDes + 0.2 * userCmd->yawd_des;
    hInt += 4*param->dt*(bodyHeight - estimatedState->floatingBaseState.pos[2]);

    if (firstRun)
    {
        xDes = estimatedState->floatingBaseState.pos[0];
        yDes = estimatedState->floatingBaseState.pos[1];
        yawDes = estimatedState->floatingBaseState.rpy[2];
        firstRun = false;
    }
    else
    {
        xDes += vWorldDes[0] * param->dt;
        yDes += vWorldDes[1] * param->dt;
//        yawDes = estimatedState->floatingBaseState.rpy[2] + yawdDes * param->dt;
        yawDes += userCmd->yawd_des * param->dt;
        // std::cout << "yaw des: " << yawDes << "\n";
        // std::cout << "yaw ori: " << estimatedState->floatingBaseState.rpy[2] << "\n";
        // std::cout << "yaw error: " << yawDes-estimatedState->floatingBaseState.rpy[2] << "\n\n";
    }
    rpInt[0] += param->dt * (rollDes - estimatedState->floatingBaseState.rpy[0]);
    rpInt[1] += param->dt * (pitchDes - estimatedState->floatingBaseState.rpy[1]);
    rpCom[0] = rollDes + rpInt[0];
    rpCom[1] = pitchDes + 2 * (pitchDes - estimatedState->floatingBaseState.rpy[1]) + rpInt[1];
}

void WheelCoMPlanner::generateContactTable()
{
    Vec4Int stTimeRemainInt = (gaitData->stanceTimeRemain / param->dtMPC).cast<int>();
    Vec4Int swTimeRemainInt = (gaitData->swingTimeRemain / param->dtMPC).cast<int>();
    Vec4Int swTimeInt = (gaitData->swingTime / param->dtMPC).cast<int>();
    Vec4Int stTimeInt = (gaitData->nextStanceTime / param->dtMPC).cast<int>();

    for (int i(0); i < param->horizons; i++)
    {
        for (int j(0); j < 4; j++)
        {
            if (stTimeRemainInt[j] > 0)
            { // leg is in stance
                if (i < stTimeRemainInt[j])
                    contactTable[i * 4 + j] = 1;
                else if (i >= stTimeRemainInt[j] &&
                         i < (swTimeInt[j] + stTimeRemainInt[j]))
                    contactTable[i * 4 + j] = 0;
                else if (i >= (swTimeInt[j] + stTimeRemainInt[j]) &&
                         i < (swTimeInt[j] + stTimeRemainInt[j] + stTimeInt[j]))
                    contactTable[i * 4 + j] = 1;
                else
                    printf("Predicted horizons exceed one gait cycle!\n");
            }
            else
            { // leg in swing
                if (i < swTimeRemainInt[j])
                    contactTable[i * 4 + j] = 0;
                else if (i >= swTimeRemainInt[j] &&
                         i < (swTimeRemainInt[j] + stTimeInt[j]))
                    contactTable[i * 4 + j] = 1;
                else if (i >= (swTimeRemainInt[j] + stTimeInt[j]) &&
                         i < (swTimeRemainInt[j] + stTimeInt[j] + swTimeInt[j]))
                    contactTable[i * 4 + j] = 0;
                else
                    printf("Predicted horizons exceed one gait cycle!\n");
            }
        }
    }
}

void WheelCoMPlanner::generateRefTraj()
{
    vBodyDes = 0.8 * vBodyDes + 0.2 * Vec3(userCmd->vx_des, userCmd->vy_des, 0);
    Vec3 vWorldDes = estimatedState->floatingBaseState.R_wb * vBodyDes;
    Vec3 pos = estimatedState->floatingBaseState.pos;
    Vec3 rpy = estimatedState->floatingBaseState.rpy;

    if (xDes - pos[0] > maxPosError)
        xDes = pos[0] + maxPosError;
    if (pos[0] - xDes > maxPosError)
        xDes = pos[0] - maxPosError;

    if (yDes - pos[1] > maxPosError)
        yDes = pos[1] + maxPosError;
    if (pos[1] - yDes > maxPosError)
        yDes = pos[1] - maxPosError;

    if (yawDes - rpy[2] > 0.1 )
        yawDes = rpy[2] + 0.1;
    else if (rpy[2] - yawDes > 0.1)
        yawDes = rpy[2] - 0.1;

#ifdef USE_MPC_STANCE
    double _bodyHeight_slow;
    if (_iter < 200)
    {
        _bodyHeight_slow = 0.05 + 0.30 / 200 * _iter;
    }
    else
    {
        _bodyHeight_slow = bodyHeight;
    }
    trajInit << rpCom[0], rpCom[1], yawDes,
        xDes, yDes, _bodyHeight_slow,
        0.0, 0.0, yawdDes,
        vWorldDes;
#else
    trajInit << rpCom[0], rpCom[1], yawDes,
            xDes, yDes, bodyHeight + hInt,
            0.0, 0.0, yawdDes,
            vWorldDes;
#endif
    X_d.head(12) = trajInit;
    for (int i = 1; i < param->horizons; i++)
    {
        for (int j = 0; j < 12; j++)
            X_d[25 * i + j] = trajInit[j];

        X_d[25 * i + 2] = trajInit[2] + i * yawdDes * param->dtMPC;
        X_d.segment(25 * i + 3, 3) = trajInit.segment(3, 3) +
                                     i * vWorldDes * param->dtMPC;
//                std::cout << "x_d " << i << ": " << X_d.segment(13 * i, 13).transpose() << std::endl;
    }
}

void WheelCoMPlanner::updateTaskData()
{
    // getchar();
    // Matrix<double,13,1> xnow;
    // xnow << estimatedState->floatingBaseState.rpy,
    //         estimatedState->floatingBaseState.pos,
    //         estimatedState->floatingBaseState.omegaWorld,
    //         estimatedState->floatingBaseState.vWorld, -9.81;
    auto xopt = convexWSRBmpc.getDiscreteOptimizedStates();

    /* std::cout << "optimal traj:\n";
    for (int i(0); i < param->horizons; i++) 
        std::cout << xopt.segment(25*i,25).transpose() << "\n";
    std::cout << "\n"; */

    auto xdd = convexWSRBmpc.getXDot();

    comTaskData->pos = xopt.segment(3, 3);
    comTaskData->vWorld = xopt.segment(9, 3);
    comTaskData->linAccWorld = xdd.segment(9, 3);
    comTaskData->angAccWorld = xdd.segment(6, 3);
    comTaskData->rpy << rollDes, pitchDes, yawDes;
    comTaskData->omegaWorld << 0., 0., yawdDes;
}

/* for test */
void WheelCoMPlanner::setInitialState(const Vec3 &rpy, const Vec3 &p, const Vec3 &omega, const Vec3 &v)
{
    x0 << rpy, p, omega, v, -9.8;
    // std::cout << "x0: " << x0.transpose() << "\n";
}

void WheelCoMPlanner::setMPCTable(const Matrix<int, Dynamic, 1> &mpcTable)
{
    contactTable = mpcTable;
}

void WheelCoMPlanner::setReferenceTraj(const DVec &Xref)
{
    for (int i(0); i < param->horizons; i++)
        for (int j(0); j < 12; j++)
            X_d(13 * i + j, 0) = Xref(12 * i + j, 0);
}
