//
// Created by wenchun on 3/22/21.
//

#include "CoMPlannerMPC.h"
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include "orientation_tools.h"

#define BIG_NUMBER 5e10
#define PRINT_PROBLEM_DATA
//#define USE_MPC_STANCE

CoMPlannerMPC::CoMPlannerMPC(const EstimatedState *estimatedState,
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
                                                                  srgbMpc(param->horizons, param->dtMPC),
                                                                  _iter(0) {
    rpInt.setZero();
    rpCom.setZero();
    trajInit.setZero();

    x0.setZero();

    X_d.resize(13 * param->horizons, Eigen::NoChange);
    U_b.resize(20 * param->horizons, Eigen::NoChange);
    contactTable.resize(4 * param->horizons, Eigen::NoChange);
    fmat.resize(20 * param->horizons, 12 * param->horizons);

    X_d.setZero();
    U_b.setZero();
    contactTable.setZero();
    fmat.setZero();

    q_soln.resize(12 * param->horizons);
    q_soln.setZero();

    /*Vec3 Id;
    Id << .07f, 0.26f, 0.242f; // miniCheetah*/
    /*Ibody << 0.353682, -0.000624863, -0.0313391,
            -0.000624863, 0.813405, -0.000940105,
            -0.0313391, -0.000940105, 0.863894;
    */
    /*Ibody.setZero();
    Ibody.diagonal() << 3 *  Id;*/
    /*
    mass = 15.8198;*/

    // Ibody << 246744762.68E-09, -271602.97E-09, -25854.78E-09, -271602.97E-09, 1354589401.49E-09, -99962.03E-09, -25854.78E-09, -99962.03E-09, 1554032296.64E-09;
    /*Ibody.setZero();
    Ibody.diagonal() << 3 *  Id;*/
    // mass = 30.69226;
//    mass = 30.32;
//     Ibody << 0.498815, -0.000325231,   -0.0771047,
// -0.000325231,      1.95893, -0.000144054,
//   -0.0771047, -0.000144054,      2.12069;
//    Ibody << 0.597502, 0.334135, -0.109927,
//             0.334135, 1.88116,   0.244158,
//            -0.109927, 0.0244158, 2.1456;
    /* xiaotian-wheel */
    mass = 14.4826;
    Ibody << 0.545662, 0.00316728, 0.00310174,
            0.00316728, 1.37052, 0.0032086,
            0.00310174, 0.0032086, 1.47737;


    SRGB_MPC::Vec Qx(12);
    Qx.segment(0, 3) << param->Q_rpy[0], param->Q_rpy[1], param->Q_rpy[2];
    Qx.segment(3, 3) << param->Q_pos[0], param->Q_pos[1], param->Q_pos[2];
    Qx.segment(6, 3) << param->Q_omega[0], param->Q_omega[1], param->Q_omega[2];
    Qx.segment(9, 3) << param->Q_vel[0], param->Q_vel[1], param->Q_vel[2];
    SRGB_MPC::Vec Qf(3);
    Qf.fill(4e-5);
    srgbMpc.setWeight(Qx, Qf);
    srgbMpc.setMassAndInertia(mass, Ibody);
    srgbMpc.setMaxForce(200);
    _ext_wrench.setZero();
    vBodyDes.setZero();

    bodyHeight = param->height;
}

void CoMPlannerMPC::setExternalWrench(Vec6 wrench) {
    _ext_wrench = wrench;
}

void CoMPlannerMPC::plan() {
    x0 << estimatedState->floatingBaseState.rpy, estimatedState->floatingBaseState.pos,
            estimatedState->floatingBaseState.omegaWorld, estimatedState->floatingBaseState.vWorld, -9.81;

    std::cout << "x0\n" << x0 << std::endl;

//    std::cout << fixed << setprecision(3) << "[floatingBaseState.rpy]: \n" << x0.segment(0, 3) << std::endl;
//    std::cout << fixed << setprecision(3) << "[floatingBaseState.pos]: \n" << x0.segment(3, 3) << std::endl;
//    std::cout << fixed << setprecision(3) << "[floatingBaseState.omegaWorld]: \n" << x0.segment(6, 3) << std::endl;
//    std::cout << fixed << setprecision(3) << "[floatingBaseState.vWorld]: \n" << x0.segment(9, 3) << std::endl;

    srgbMpc.setCurrentState(x0);
    generateRefTraj(); // update X_d
//     std::cout << "first horizon traj: \n" << trajInit.transpose() << "\n\n";
    srgbMpc.setDesiredDiscreteTrajectory(X_d);
    generateContactTable();
    Matrix<int, -1, -1> contactTable_4h(4, param->horizons);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < param->horizons; j++) {
            contactTable_4h(i, j) = contactTable(4 * j + i);
        }
    }
    std::cout << "[contactTable]:\n" << contactTable_4h << std::endl;
    srgbMpc.setContactTable(contactTable_4h);

    srgbMpc.setExternalWrench(_ext_wrench);

    Vec3 r;
    vector<Vec3> cp;
    for (int leg(0); leg < 4; leg++) {
        r = estimatedState->footState[leg].pos;  //TODO check footState
        std::cout << "footState[" << leg << "]: " << estimatedState->footState[leg].pos.transpose() << std::endl;
        cp.push_back(r);
    }
    
    srgbMpc.setContactPointPos(cp);
    srgbMpc.solve(0.0);
    forcesTaskData->forces_ref = srgbMpc.getCurrentDesiredContactForce();

    std::cout << "forces_ref: " << forcesTaskData->forces_ref.transpose() << std::endl;

    updateTaskData();
    _ext_wrench.setZero();
    _iter++;
}

void CoMPlannerMPC::generateHighLevelRef() {
    vBodyDes = 0.8 * vBodyDes + 0.2 * Vec3(userCmd->vx_des, userCmd->vy_des, 0);
    // Vec3 vWorldDes = estimatedState->floatingBaseState.R_wb * vBodyDes;
    double yc = cos(estimatedState->floatingBaseState.rpy(2));
    double ys = sin(estimatedState->floatingBaseState.rpy(2));
    Mat3 Rz;
    Rz << yc, -ys, 0.,
            ys, yc, 0.,
            0., 0., 1;
    Vec3 vWorldDes = Rz * vBodyDes;
    // yawdDes = 0.8 * yawdDes + 0.2 * userCmd->yawd_des;
    yawdDes = userCmd->yawd_des;
//    hInt += 4 * param->dt * (bodyHeight - estimatedState->floatingBaseState.pos[2]);

    if (firstRun) {
        xDes = estimatedState->floatingBaseState.pos[0];
        yDes = estimatedState->floatingBaseState.pos[1];
        yawDes = estimatedState->floatingBaseState.rpy[2];
        firstRun = false;
//        std::cout << "[userCmd->gaitNum]: " << userCmd->gaitNum << std::endl;
    } else {

//        std::cout << "[userCmd->gaitNum]: " << userCmd->gaitNum << std::endl;
        if (userCmd->gaitNum == 1) { // Gait: Stand
            Vec3 CoG(0, 0, 0);
            for (int i(0); i < 4; i++)
                CoG += estimatedState->footState[i].pos;
            xDes = CoG[0] / 4;
            yDes = CoG[1] / 4;
//            std::cout << "[CoG]: xDes: " << xDes
//                      << " yDes: " << yDes
//                      << " yawDes: " << yawDes << std::endl;
        } else {
            xDes += vWorldDes[0] * param->dtMPC;
            yDes += vWorldDes[1] * param->dtMPC;
        }

        if (fabs(yawdDes) >= 0.1)
            // yawDes = estimatedState->floatingBaseState.rpy[2] + yawdDes * param->dt;
            yawDes += userCmd->yawd_des * param->dtMPC;
        // std::cout << "yaw des: " << yawDes << "\n";
        // std::cout << "yaw ori: " << estimatedState->floatingBaseState.rpy[2] << "\n";
        // std::cout << "yaw error: " << yawDes-estimatedState->floatingBaseState.rpy[2] << "\n\n";
    }
    rpInt[0] += param->dtMPC * (rollDes - estimatedState->floatingBaseState.rpy[0]);
    rpInt[1] += param->dtMPC * (pitchDes - estimatedState->floatingBaseState.rpy[1]);
    rpCom[0] = rollDes + rpInt[0];
    rpCom[1] = pitchDes + 2 * (pitchDes - estimatedState->floatingBaseState.rpy[1]) + rpInt[1];
}

void CoMPlannerMPC::generateContactTable() {
    Vec4Int stTimeRemainInt = (gaitData->stanceTimeRemain / param->dtMPC).cast<int>();
    Vec4Int swTimeRemainInt = (gaitData->swingTimeRemain / param->dtMPC).cast<int>();
    Vec4Int swTimeInt = (gaitData->swingTime / param->dtMPC).cast<int>();
    Vec4Int stTimeInt = (gaitData->nextStanceTime / param->dtMPC).cast<int>();

    for (int i(0); i < param->horizons; i++) {
        for (int j(0); j < 4; j++) {
            if (stTimeRemainInt[j] > 0) { // leg is in stance
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
//                std::cout << "contactTable " << i << ": " << contactTable.segment(4 * i, 4).transpose() << std::endl;
            } else { // leg in swing
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
        std::cout << std::endl;
    }
}

void CoMPlannerMPC::generateRefTraj() {
    // vBodyDes = 0.8 * vBodyDes + 0.2 * Vec3(userCmd->vx_des, userCmd->vy_des, 0);
    // Vec3 vWorldDes = estimatedState->floatingBaseState.R_wb * vBodyDes;
    if (userCmd->gaitNum != 1) // not standing
        vBodyDes << userCmd->vx_des, userCmd->vy_des, 0.;
    else { // standing
        vBodyDes << userCmd->vx_des, 0., 0.;
//        std::cout << "IS STANDING!" << std::endl;
    }

    double yc = cos(estimatedState->floatingBaseState.rpy(2));
    double ys = sin(estimatedState->floatingBaseState.rpy(2));
    Mat3 Rz;
    Rz << yc, -ys, 0.,
            ys, yc, 0.,
            0., 0., 1;
    Vec3 vWorldDes = Rz * vBodyDes;

    Vec3 pos = estimatedState->floatingBaseState.pos;
    Vec3 rpy = estimatedState->floatingBaseState.rpy;

    if (xDes - pos[0] > maxPosError) // only add 0.1 at max everytime
        xDes = pos[0] + maxPosError;
    if (pos[0] - xDes > maxPosError)
        xDes = pos[0] - maxPosError;

    if (yDes - pos[1] > maxPosError)
        yDes = pos[1] + maxPosError;
    if (pos[1] - yDes > maxPosError)
        yDes = pos[1] - maxPosError;

    if (yawDes - rpy[2] > 0.1)
        yawDes = rpy[2] + 0.1;
    else if (rpy[2] - yawDes > 0.1)
        yawDes = rpy[2] - 0.1;

#ifdef USE_MPC_STANCE
    double _bodyHeight_slow;
    if (_iter < 200)
    {
        _bodyHeight_slow = bodyHeight / 200 * _iter;
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
            xDes, yDes, bodyHeight + userCmd->dh, // 0.35 + 0.2
            0.0, 0.0, yawdDes,
            vWorldDes;

//    std::cout << "[trajInit]: " << trajInit << std::endl;
#endif
    X_d.head(12) = trajInit;
    for (int i = 1; i < param->horizons; i++) {
        for (int j = 0; j < 12; j++)
            X_d[13 * i + j] = trajInit[j];

        X_d[13 * i + 2] = trajInit[2] + i * yawdDes * param->dtMPC;
        X_d.segment(13 * i + 3, 3) = trajInit.segment(3, 3) +
                                     i * vWorldDes * param->dtMPC; // TODO: need to assign vWorldDes?
//        std::cout << "x_d " << i << ": " << X_d.segment(13 * i, 13).transpose() << std::endl;
    }
    std::cout << "\n";
}

void CoMPlannerMPC::updateTaskData() {
    // getchar();
    // Matrix<double,13,1> xnow;
    // xnow << estimatedState->floatingBaseState.rpy,
    //         estimatedState->floatingBaseState.pos,
    //         estimatedState->floatingBaseState.omegaWorld,
    //         estimatedState->floatingBaseState.vWorld, -9.81;
    auto xopt = srgbMpc.getDiscreteOptimizedTrajectory();
    auto xdd = srgbMpc.getXDot();

    comTaskData->pos = xopt.segment(3, 3);
    comTaskData->vWorld = xopt.segment(9, 3);
    comTaskData->linAccWorld = xdd.segment(9, 3);
    comTaskData->angAccWorld = xdd.segment(6, 3);
    comTaskData->rpy << rollDes, pitchDes, yawDes;
    comTaskData->omegaWorld << 0., 0., yawdDes;
}

/* for test */
void CoMPlannerMPC::setInitialState(const Vec3 &rpy, const Vec3 &p, const Vec3 &omega, const Vec3 &v) {
    x0 << rpy, p, omega, v, -9.8;
    // std::cout << "x0: " << x0.transpose() << "\n";
}

void CoMPlannerMPC::setMPCTable(const Matrix<int, Dynamic, 1> &mpcTable) {
    contactTable = mpcTable;
}

void CoMPlannerMPC::setReferenceTraj(const DVec &Xref) {
    for (int i(0); i < param->horizons; i++)
        for (int j(0); j < 12; j++)
            X_d(13 * i + j, 0) = Xref(12 * i + j, 0);
}
