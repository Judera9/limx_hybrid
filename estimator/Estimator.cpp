//
// Created by wenchun on 3/17/21.
//

#include "Estimator.h"
#include "orientation_tools.h"
#include <iostream>

using namespace ori;

Estimator::Estimator(const MeasuredState *measuredState, RobotModelData *robotModelData, EstimatedState *estimatedState,
                     const SwitchState *switchState, const UserParameterHandler *param)
        : measuredState(measuredState), robotModelData(robotModelData),
          estimatedState(estimatedState),
          switchState(switchState), param(param) {
    setup();
}

void Estimator::setup() {
    // TODO: need to check the size of A B C P ... for using EKF
    _xhat.setZero();
    _ps.setZero();
    _vs.setZero();
    _A.setZero();
    _A.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    _A.block(0, 3, 3, 3) = param->dt * Eigen::Matrix<double, 3, 3>::Identity();
    _A.block(3, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    _A.block(6, 6, 12, 12) = Eigen::Matrix<double, 12, 12>::Identity();
    _B.setZero();
    _B.block(3, 0, 3, 3) = param->dt * Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
    C1 << Eigen::Matrix<double, 3, 3>::Identity(), Eigen::Matrix<double, 3, 3>::Zero();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
    C2 << Eigen::Matrix<double, 3, 3>::Zero(), Eigen::Matrix<double, 3, 3>::Identity();
    _C.setZero();
    _C.block(0, 0, 3, 6) = C1;
    _C.block(3, 0, 3, 6) = C1;
    _C.block(6, 0, 3, 6) = C1;
    _C.block(9, 0, 3, 6) = C1;
    _C.block(0, 6, 12, 12) = double(-1) * Eigen::Matrix<double, 12, 12>::Identity();
    _C.block(12, 0, 3, 6) = C2;
    _C.block(15, 0, 3, 6) = C2;
    _C.block(18, 0, 3, 6) = C2;
    _C.block(21, 0, 3, 6) = C2;
    _C(27, 17) = double(1);
    _C(26, 14) = double(1);
    _C(25, 11) = double(1);
    _C(24, 8) = double(1);
    _P.setIdentity();
    _P = double(100) * _P;
    _Q0.setIdentity();
    _Q0.block(0, 0, 3, 3) = (param->dt / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
    _Q0.block(3, 3, 3, 3) =
            (param->dt * 9.8f / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
    _Q0.block(6, 6, 12, 12) = param->dt * Eigen::Matrix<double, 12, 12>::Identity();
    _R0.setIdentity();
}

void Estimator::orientationEstimator() {
    estimatedState->floatingBaseState.quat =
            measuredState->imuData.quat;

    if (_b_first_visit) {
        Vec3 rpy_ini = quatToRPY(measuredState->imuData.quat);
        rpy_ini[0] = 0;
        rpy_ini[1] = 0;
        _ori_ini_inv = rpyToQuat(-rpy_ini); // extract Yaw only and convert it back to quat representation
        _b_first_visit = false;
    }
    estimatedState->floatingBaseState.quat =
            quatProduct(_ori_ini_inv, estimatedState->floatingBaseState.quat);

    estimatedState->floatingBaseState.rpy =
            quatToRPY(estimatedState->floatingBaseState.quat);
    // rpy --> tosor row pitch yaw
//     std::cout << "floatingBaseState.rpy:" << estimatedState->floatingBaseState.rpy.transpose() << std::endl;
    estimatedState->floatingBaseState.R_wb = quaternionToRotationMatrix(
            estimatedState->floatingBaseState.quat).transpose();

    estimatedState->floatingBaseState.omegaBody =
            measuredState->imuData.gyro;

    estimatedState->floatingBaseState.omegaWorld =
            estimatedState->floatingBaseState.R_wb *
            estimatedState->floatingBaseState.omegaBody;

    estimatedState->floatingBaseState.aBody =
            measuredState->imuData.acc;

    estimatedState->floatingBaseState.aWorld =
            estimatedState->floatingBaseState.R_wb *
            estimatedState->floatingBaseState.aBody;

    // aWorld seems to be the acc caused by external forces excluding gravity
//     std::cout << "floatingBaseState.aWorld:" << estimatedState->floatingBaseState.aWorld.transpose() << std::endl;
}

// void Estimator::contactEstimator() {
//   estimatedState->footState->contactState =
//       *this->_data.contactPhase;
// }

void Estimator::linearKFPositionVelocityEstimator() {
    double process_noise_pimu = double(0.02);
    double process_noise_vimu = double(0.02);
    double process_noise_pfoot = double(0.002);
    double sensor_noise_pimu_rel_foot = double(0.001);
    double sensor_noise_vimu_rel_foot = double(0.1);
    double sensor_noise_zfoot = double(0.001);

    estimatedState->jointsState.qpos = measuredState->jointsState.qpos;
    estimatedState->jointsState.qvel = measuredState->jointsState.qvel;
//    estimatedState->jointsState.qpos = measuredState->cheatData.qpos;
//    estimatedState->jointsState.qvel = measuredState->cheatData.qvel;

    // checked: measuredState -> jointsState pos & vel are corresponded to the demonstration in the <<hybrid.xml>>
    std::cout << "[jointsState.qpos]:\n" << measuredState->jointsState.qpos.segment(0, 4)
              << "[jointsState.qvel]:\n" << measuredState->jointsState.qvel.segment(0, 4) << std::endl;
    std::cout << "[cheatData.qpos]:\n" << measuredState->cheatData.qpos.segment(0, 4)
              << "[cheatData.qvel]:\n" << measuredState->cheatData.qvel.segment(0, 4) << std::endl;

    Eigen::Matrix<double, 18, 18> Q = Eigen::Matrix<double, 18, 18>::Identity();
    Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * process_noise_pimu;
    Q.block(3, 3, 3, 3) = _Q0.block(3, 3, 3, 3) * process_noise_vimu;
    Q.block(6, 6, 12, 12) = _Q0.block(6, 6, 12, 12) * process_noise_pfoot;

    Eigen::Matrix<double, 28, 28> R = Eigen::Matrix<double, 28, 28>::Identity();
    R.block(0, 0, 12, 12) = _R0.block(0, 0, 12, 12) * sensor_noise_pimu_rel_foot;
    R.block(12, 12, 12, 12) =
            _R0.block(12, 12, 12, 12) * sensor_noise_vimu_rel_foot;
    R.block(24, 24, 4, 4) = _R0.block(24, 24, 4, 4) * sensor_noise_zfoot;

    std::cout << std::setprecision(1) << std::fixed;
    std::cout << "[Q]:\n" << Q << "\n[R]:\n" << R << std::endl;

    int qindex = 0;
    int rindex1 = 0;
    int rindex2 = 0;
    int rindex3 = 0;

    Vec3 g(0, 0, double(-9.81));
    Mat3 Rbod = estimatedState->floatingBaseState.R_wb;
    Vec3 a = estimatedState->floatingBaseState.aWorld + g;
    Vec4 pzs = Vec4::Zero();
    Vec4 trusts = Vec4::Zero();
    Vec3 p0, v0;
    p0 << _xhat[0], _xhat[1], _xhat[2];
    v0 << _xhat[3], _xhat[4], _xhat[5];

//   Vec23 qpos <<
    Vec23 qpos_pin;
    qpos_pin << 0, 0, 0, estimatedState->floatingBaseState.quat.tail(3),
            estimatedState->floatingBaseState.quat(0), estimatedState->jointsState.qpos;
    Vec22 qvel_pin;
    qvel_pin << 0, 0, 0, estimatedState->floatingBaseState.omegaBody, estimatedState->jointsState.qvel;

    std::cout << "[qpos_pin]:\n" << qpos_pin.transpose()
              << "\n[qvel_pin]:\n" << qvel_pin.transpose() << std::endl;

    pin::normalize(robotModelData->model, qpos_pin);
    pin::forwardKinematics(robotModelData->model, robotModelData->data, qpos_pin, qvel_pin);

    size_t linkID[4] = {robotModelData->flwID, robotModelData->frwID, robotModelData->hlwID, robotModelData->hrwID};

    for (int i = 0; i < 4; i++) {
        pin::updateFramePlacement(robotModelData->model, robotModelData->data, linkID[i]);
        auto v = pin::getFrameVelocity(robotModelData->model, robotModelData->data, linkID[i],
                                       pin::LOCAL_WORLD_ALIGNED);
        int i1 = 3 * i;

        // center of wheel position relative to base expressed in world(checked)
        Vec3 p_f = robotModelData->data.oMf[linkID[i]].translation();
        // std::cout << "p_f[" << i << "].pos:" <<  p_f.transpose() << std::endl;
        // p_f(2) -= param->rw; // 
        // center of wheel velocity relative to base expressed in world
        Vec3 v_f = v.linear();

        // check wheel angular velocity direction, assume wheel is not slippy and ground is flat
        Vec3 vx_wheel(measuredState->jointsState.qvel[4 * i + 3] * param->rw, 0., 0.);
        double yc = cos(estimatedState->floatingBaseState.rpy(2));
        double ys = sin(estimatedState->floatingBaseState.rpy(2));
        Mat3 Rz;
        Rz << yc, -ys, 0.,
                ys, yc, 0.,
                0., 0., 1;
        Vec3 v_wheel = Rz * vx_wheel;

        qindex = 6 + i1;
        rindex1 = i1;
        rindex2 = 12 + i1;
        rindex3 = 24 + i;

        double trust = double(1);
        double phase = fmin(estimatedState->contactPhaseDes(i), double(1));
//    double trust_window = double(0.25);
        double trust_window = double(0.2);

        if (phase < trust_window) {
            trust = phase / trust_window;
        } else if (phase > (double(1) - trust_window)) {
            trust = (double(1) - phase) / trust_window;
        }
        double high_suspect_number(500);
        // double high_suspect_number(100);

        // printf("Trust %d: %.3f\n", i, trust);
        Q.block(qindex, qindex, 3, 3) =
                (double(1) + (double(1) - trust) * high_suspect_number) * Q.block(qindex, qindex, 3, 3);
        R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);
        R.block(rindex2, rindex2, 3, 3) =
                (double(1) + (double(1) - trust) * high_suspect_number) * R.block(rindex2, rindex2, 3, 3);
        R(rindex3, rindex3) =
                (double(1) + (double(1) - trust) * high_suspect_number) * R(rindex3, rindex3);

        trusts(i) = trust;

        _ps.segment(i1, 3) = -p_f;
        pzs(i) = (1.0f - trust) * (p0(2) + p_f(2));
        if (_useContactInvariant)
            _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-v_f); // contact invariant
        else
            _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (v_wheel - v_f); // contact rolling 
//        pzs(i) = trust * (p0(2) + p_f(2));
    }

    Eigen::Matrix<double, 28, 1> y;
    y << _ps, _vs, pzs;
    _xhat = _A * _xhat + _B * a;
    Eigen::Matrix<double, 18, 18> At = _A.transpose();
    Eigen::Matrix<double, 18, 18> Pm = _A * _P * At + Q;
    Eigen::Matrix<double, 18, 28> Ct = _C.transpose();
    Eigen::Matrix<double, 28, 1> yModel = _C * _xhat;
    Eigen::Matrix<double, 28, 1> ey = y - yModel;
    Eigen::Matrix<double, 28, 28> S = _C * Pm * Ct + R;

    // todo compute LU only once
    Eigen::Matrix<double, 28, 1> S_ey = S.lu().solve(ey);
    _xhat += Pm * Ct * S_ey;
    Eigen::Matrix<double, 28, 18> S_C = S.lu().solve(_C);
    _P = (Eigen::Matrix<double, 18, 18>::Identity() - Pm * Ct * S_C) * Pm;
    Eigen::Matrix<double, 18, 18> Pt = _P.transpose();
    _P = (_P + Pt) / double(2);

    if (_P.block(0, 0, 2, 2).determinant() > double(0.000001)) {
        _P.block(0, 2, 2, 16).setZero();
        _P.block(2, 0, 16, 2).setZero();
        _P.block(0, 0, 2, 2) /= double(10);
    }

    estimatedState->floatingBaseState.pos = _xhat.block(0, 0, 3, 1);
    estimatedState->floatingBaseState.vWorld = _xhat.block(3, 0, 3, 1);
    estimatedState->floatingBaseState.vBody =
            estimatedState->floatingBaseState.R_wb.transpose() *
            estimatedState->floatingBaseState.vWorld;
    computeModelData();
}

void Estimator::cheatMode() {
    // joint_num_related_value-1
    estimatedState->floatingBaseState.pos = measuredState->cheatData.qpos.head(3);
    estimatedState->floatingBaseState.quat = measuredState->cheatData.qpos.segment(3, 4);
    estimatedState->floatingBaseState.R_wb = quaternionToRotationMatrix(
            estimatedState->floatingBaseState.quat).transpose(); // TODO: transpose twice?
    estimatedState->floatingBaseState.vWorld = measuredState->cheatData.qvel.head(3);
    estimatedState->floatingBaseState.vBody =
            estimatedState->floatingBaseState.R_wb.transpose() * estimatedState->floatingBaseState.vWorld;
    estimatedState->floatingBaseState.omegaBody = measuredState->cheatData.qvel.segment(3, 3);
    estimatedState->floatingBaseState.omegaWorld =
            estimatedState->floatingBaseState.R_wb * estimatedState->floatingBaseState.omegaBody;
    estimatedState->floatingBaseState.rpy = quatToRPY(estimatedState->floatingBaseState.quat);
    estimatedState->floatingBaseState.aWorld.setZero();
    estimatedState->floatingBaseState.aBody.setZero();
    // joint_num_related_value
    estimatedState->jointsState.qpos = measuredState->cheatData.qpos.segment(7, 16);
    estimatedState->jointsState.qvel = measuredState->cheatData.qvel.segment(6, 16);

    std::cout << "cheatData.qpos: " << measuredState->cheatData.qpos.transpose() << std::endl;
    computeModelData();
}

void Estimator::legTestMode() {
    estimatedState->floatingBaseState.pos.setZero();
    estimatedState->floatingBaseState.vWorld.setZero();
    estimatedState->floatingBaseState.vBody.setZero();

    if (param->bool_sim) {
        for (int i = 0; i < 20; i++) {
            estimatedState->jointsState.qpos[i] = measuredState->jointsState.qpos[i] * param->jointsSigns[i];
            estimatedState->jointsState.qvel[i] = measuredState->jointsState.qvel[i] * param->jointsSigns[i];
        }
    } else {
        estimatedState->jointsState.qpos = measuredState->jointsState.qpos;
        estimatedState->jointsState.qvel = measuredState->jointsState.qvel;
    }

//    estimatedState->floatingBaseState.quat

    computeModelData();
}

void Estimator::computeModelData() {
    // joint_num_related_value
    Vec23 qpos_pin;
    qpos_pin
            << estimatedState->floatingBaseState.pos, estimatedState->floatingBaseState.quat.tail(
            3), estimatedState->floatingBaseState.quat(0), estimatedState->jointsState.qpos;
    Vec22 qvel_pin;
    qvel_pin
            << estimatedState->floatingBaseState.vBody, estimatedState->floatingBaseState.omegaBody, estimatedState->jointsState.qvel;

//     Vec5 qpos_pin;
//     qpos_pin << estimatedState->jointsState.qpos;
//     Vec5 qvel_pin;
//     qvel_pin << estimatedState->jointsState.qvel;

    pin::crba(robotModelData->model, robotModelData->data, qpos_pin);
    robotModelData->data.M.triangularView<Eigen::StrictlyLower>() = robotModelData->data.M.transpose().triangularView<Eigen::StrictlyLower>();
    robotModelData->M = robotModelData->data.M;
    pin::nonLinearEffects(robotModelData->model, robotModelData->data, qpos_pin, qvel_pin);
    robotModelData->bias = robotModelData->data.nle;
    pin::computeGeneralizedGravity(robotModelData->model, robotModelData->data, qpos_pin);
    robotModelData->generalizedGravity = robotModelData->data.g;

//    std::cout << "g: " << robotModelData->generalizedGravity << std::endl;

    size_t linkID[4] = {robotModelData->flwID, robotModelData->frwID, robotModelData->hlwID, robotModelData->hrwID};
    size_t hipID[4] = {robotModelData->flhID, robotModelData->frhID, robotModelData->hlhID, robotModelData->hrhID};
    size_t footID[4] = {robotModelData->flfootID, robotModelData->frfootID, robotModelData->hlfootID,
                        robotModelData->hrfootID}; // TODO: commented? and delete related codes

    for (int i = 0; i < 4; i++) {
        // footState.pos - floatingBaseState.pos ==== center of wheel position relative to base expressed in world(checked)
        pin::forwardKinematics(robotModelData->model, robotModelData->data, qpos_pin, qvel_pin);
        pin::updateFramePlacement(robotModelData->model, robotModelData->data, linkID[i]);
        auto v = pin::getFrameVelocity(robotModelData->model, robotModelData->data, linkID[i],
                                       pin::LOCAL_WORLD_ALIGNED);
        estimatedState->footState[i].pos = robotModelData->data.oMf[linkID[i]].translation();//
        estimatedState->footState[i].R_wf = robotModelData->data.oMf[linkID[i]].rotation();
        estimatedState->footState[i].vWorld = v.linear();
        estimatedState->footState[i].omegaWorld = v.angular();
        estimatedState->footState[i].rpy = rotationMatrixToRPY(estimatedState->footState[i].R_wf);
    }

     for (int i(0); i < 4; i++) {
         std::cout << "footState[" << i << "].pos:" <<  (estimatedState->footState[i].pos - estimatedState->floatingBaseState.pos).transpose() << std::endl;
     }

     std::cout << "floatingBaseState.pos: " << estimatedState->floatingBaseState.pos.transpose()
               << "\nfloatingBaseState.vWorld: " << estimatedState->floatingBaseState.vWorld.transpose() << std::endl;

     getchar();

    pin::computeJointJacobians(robotModelData->model, robotModelData->data, qpos_pin);
    pin::computeJointJacobiansTimeVariation(robotModelData->model, robotModelData->data, qpos_pin, qvel_pin);
    pin::forwardKinematics(robotModelData->model, robotModelData->data, qpos_pin, qvel_pin);

    // TODO: check current Jacobian
    Vec3 rHip_wc(0., 0., -param->rw);
    for (int i(0); i < 4; i++) { // wheel center and contact Jacobian
        D6Mat Jw(6, robotModelData->model.nv), Jwdot(6, robotModelData->model.nv);
        Jw.fill(0);
        Jwdot.fill(0);
        auto v = pin::getFrameVelocity(robotModelData->model, robotModelData->data, linkID[i],
                                       pin::LOCAL_WORLD_ALIGNED);
        pin::getFrameJacobian(robotModelData->model, robotModelData->data, linkID[i], pin::LOCAL_WORLD_ALIGNED, Jw);
        pin::getFrameJacobianTimeVariation(robotModelData->model, robotModelData->data, linkID[i],
                                           pin::LOCAL_WORLD_ALIGNED,
                                           Jwdot);
        robotModelData->Je[i] = Jw.block(0, 0, 3, robotModelData->model.nv);
        robotModelData->Je_dot_q_dot[i] =
                Jwdot.block(0, 0, 3, robotModelData->model.nv) * qvel_pin + v.angular().cross(v.linear());

        Vec3 rWorld_wc = robotModelData->data.oMi[hipID[i]].rotation() * rHip_wc;
        robotModelData->Jc[i] =
                robotModelData->Je[i] - pin::skew(rWorld_wc) * Jw.block(3, 0, 3, robotModelData->model.nv);
        D6Mat Jhip(6, robotModelData->model.nv);
        Jhip.fill(0.);
        pin::getJointJacobian(robotModelData->model, robotModelData->data, hipID[i], pin::LOCAL_WORLD_ALIGNED, Jhip);
        robotModelData->Jc_dot_q_dot[i] = robotModelData->Je_dot_q_dot[i] -
                                          pin::skew(rWorld_wc) * Jwdot.block(3, 0, 3, robotModelData->model.nv) *
                                          qvel_pin +
                                          pin::skew(
                                                  pin::skew(rWorld_wc) * Jhip.block(3, 0, 3, robotModelData->model.nv) *
                                                  qvel_pin) * Jw.block(3, 0, 3, robotModelData->model.nv) * qvel_pin;
    }

    /*std::cout << "pin quat\n: " << estimatedState->floatingBaseState.quat.transpose() << std::endl;
    std::cout << "pin rot\n: " << robotModelData->data.oMf[robotModelData->model.getFrameId("base_link")].rotation()
              << std::endl;
    std::cout << "es rot\n: " << estimatedState->floatingBaseState.R_wb << std::endl;
    std::cout << "pin euler: "
              << robotModelData->data.oMf[robotModelData->model.getFrameId("base_link")].rotation().eulerAngles(2, 1,
                                                                                                                0).transpose()
              << std::endl;
    std::cout << "es euler: " << estimatedState->floatingBaseState.rpy.transpose() << std::endl;*/
    pin::computeTotalMass(robotModelData->model, robotModelData->data);
    pin::ccrba(robotModelData->model, robotModelData->data, qpos_pin, qvel_pin);
//     std::cout << "mass: " << robotModelData->data.mass[0] << std::endl;
//     std::cout << "inertia: " << robotModelData->data.Ig << std::endl;
//     std::cout << "torsor: " << measuredState->cheatData.qpos.head(3).transpose() << std::endl;
//    std::cout << "bias " << " = " << robotModelData->bias << std::endl;
}

void Estimator::estimate() {

    if (param->estimation_mode == 0) //0-linearKFPositionVelocityEstimator  |  1-legTestMode  |  2-cheatMode
    {
        // joint_num_related_value
        orientationEstimator();
        linearKFPositionVelocityEstimator();
    } else if (param->estimation_mode == 1) {
        // joint_num_related_value
        orientationEstimator();
        legTestMode();
    } else if (param->estimation_mode == 2) {
        cheatMode();
    }

//     printf("time for estimator: %f\n", timer.getMs());
}