//
// Created by avenger on 22-4-29.
//

#include "SRGB_MPC/Wheel_SRB_MPC.h"
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <algorithm>
#include "Timer.h"

using namespace Wheel_SRB_MPC;

#define BIG_NUMBER 1e10

Wheel_SRB_MPC_IMPL::Wheel_SRB_MPC_IMPL(size_t horizon, RealNum dt) : _horizon(horizon), _dt(dt), _gravity(-9.81), _mu(0.4),
                                                           _fmax(500), _setDesiredTraj(false),
                                                           _setDesiredDiscreteTraj(false),
                                                           _ext_wrench(horizon)
{

    _mass = 0;
    _inertia.setZero();
    _Qx.setZero();
    _Q.resize(25 * _horizon, 25 * _horizon);
    _R.resize(16* _horizon, 16 * _horizon);
    _contactTable = MatInt::Ones(4, horizon);
    _desiredDiscreteTraj = Vec::Zero(25 * horizon);
    _desiredDiscreteTraj_bias = Vec::Zero(25 * horizon);
    _vel_des.setZero();

    _At.resize(25, 25);
    _At.setZero();
    _Bt.resize(25, 16);
    _Bt.setZero();

    Sx.resize(25 * _horizon, 25);
    Sx.setZero();
    Su.resize(25 * _horizon, 16 * _horizon);
    Su.setZero();

    _optimalControlInputs.resize(16 * _horizon);
    _optimalControlInputs.setZero();
    _xDot.resize(25 * _horizon);
    _x0.setZero();
    // _x0.tail(1) << _gravity;
    fill(_ext_wrench.begin(), _ext_wrench.end(), Vec6::Zero());

    _W.resize(8*horizon,8*horizon);
    _A.resize(8*horizon, 25*horizon);
    _Rxy.resize(8*horizon);
}

void Wheel_SRB_MPC_IMPL::setCurrentState(ConstVecRef x0)
{
    assert(x0.size() == 25);
    _x0 = x0; 
}

void Wheel_SRB_MPC_IMPL::setExternalWrench(ConstVec6Ref ext_wrench)
{
    fill(_ext_wrench.begin(), _ext_wrench.end(), ext_wrench);
}

void Wheel_SRB_MPC_IMPL::setExternalWrench(ConstVec6Ref ext_wrench, size_t k)
{
    assert(k < _horizon);
    _ext_wrench[k] = ext_wrench;
}

void Wheel_SRB_MPC_IMPL::setVelocityCmd(Vec6 vel_des)
{
    _vel_des = vel_des;
}

void Wheel_SRB_MPC_IMPL::setContactTable(ConstMatIntRef &contactTable)
{
    if (contactTable.rows() != 4 || contactTable.cols() != _horizon)
    {
        throw std::runtime_error("[Wheel_SRB_MPC_IMPL::setContactTable] contactTable size is wrong");
    }
    _contactTable = contactTable;
}

void Wheel_SRB_MPC_IMPL::setWeight(ConstVecRef Qx, const double alpha, ConstVecRef Wxy)
{
    assert(Qx.size() == 12);
    _Qx.setZero();
    _Qx.head(12) = Qx;
    _alpha = alpha;
    _Wxy = Wxy;
}

void Wheel_SRB_MPC_IMPL::setHipPosition(ConstVecRef hip) {
    assert(hip.size() == 8);
    _r_xy = hip;
}

void Wheel_SRB_MPC_IMPL::setDesiredTrajectory(const SixDimsPose_Trajectory &traj)
{
    _desiredTraj = traj;
    _setDesiredTraj = true;
}

const SixDimsPose_Trajectory &Wheel_SRB_MPC_IMPL::getContinuousOptimizedTrajectory()
{
    return _continuousOptimizedTraj;
}

ConstVecRef Wheel_SRB_MPC_IMPL::getDiscreteOptimizedStates()
{
    return Wheel_SRB_MPC::ConstVecRef(_discreteOptimizedStates);
}

ConstVecRef Wheel_SRB_MPC_IMPL::getOptimalControlInputs()
{
    return Wheel_SRB_MPC::ConstVecRef(_optimalControlInputs);
}

ConstVecRef Wheel_SRB_MPC_IMPL::getCurrentDesiredControlInputs()
{
    _input_des.setZero();
    int k = 0;
    for (int i = 0; i < 4; i++)
    {
        if (_contactTable.col(0)(i) == 1)
        {
            _input_des.segment(i * 4, 4) = _optimalControlInputs.segment(4 * k, 4); // contact forces
            k++;
        }
    }
    return Wheel_SRB_MPC::ConstVecRef(_input_des);
}

void Wheel_SRB_MPC_IMPL::setMassAndInertia(RealNum mass, Mat3Ref inertia)
{
    _mass = mass;
    _inertia = inertia;
}

void Wheel_SRB_MPC_IMPL::solve(RealNum t_now)
{
    if (_setDesiredTraj)
    {
        _setDesiredTraj = false;
        for (int i = 0; i < _horizon; i++)
        {
            _desiredDiscreteTraj.segment(i * 25, 25) << _desiredTraj.roll(t_now + RealNum(i) * _dt).data(),
                    _desiredTraj.pitch(t_now + RealNum(i) * _dt).data(),
                    _desiredTraj.yaw(t_now + RealNum(i) * _dt).data(),
                    _desiredTraj.x(t_now + RealNum(i) * _dt).data(),
                    _desiredTraj.y(t_now + RealNum(i) * _dt).data(),
                    _desiredTraj.z(t_now + RealNum(i) * _dt).data(),
                    _desiredTraj.roll(t_now + RealNum(i) * _dt).derivative(),
                    _desiredTraj.pitch(t_now + RealNum(i) * _dt).derivative(),
                    _desiredTraj.yaw(t_now + RealNum(i) * _dt).derivative(),
                    _desiredTraj.x(t_now + RealNum(i) * _dt).derivative(),
                    _desiredTraj.y(t_now + RealNum(i) * _dt).derivative(),
                    _desiredTraj.z(t_now + RealNum(i) * _dt).derivative(),
                    _gravity;
        }
    }
    else
    {
        if (!_setDesiredDiscreteTraj)
        {
            _desiredDiscreteTraj.setZero();
            for (int i = 0; i < _horizon; i++)
            {
                _desiredDiscreteTraj.segment(i * 25, 13)
                        << _x0.head(6) + RealNum(i) * _vel_des * _dt,
                        _vel_des, _gravity;
            }
        }
        else
        {
            _setDesiredDiscreteTraj = false;
        }
    }

    // compensate mpc desired trajectory

    //    std::cout << "_desiredDiscreteTraj: " << _desiredDiscreteTraj.transpose() << std::endl;
    _n_contact = _contactTable.sum();

    Timer tc1;
    computeSxSu();
    // cout << "Timer tc1: " << tc1.getMs() << endl;
    _desiredDiscreteTraj -= _desiredDiscreteTraj_bias;

    // set input constraints
    _C.setZero(6 * _n_contact, 4 * _n_contact);
    _ub.setZero(6 * _n_contact);
    _lb.setZero(6 * _n_contact);

    double mu = 1 / _mu;
    Mat Ci(6, 4);
    Ci << mu, 0, 1., 0.,
         -mu, 0, 1., 0.,
         0, mu, 1., 0., 
         0, -mu, 1., 0., 
         0, 0, 1., 0.,
         0., 0., 0., 1;


    _ub.fill(BIG_NUMBER);
    for (int i = 0; i < _n_contact; i++)
    {
        _C.block(i * 6, i * 4, 6, 4) = Ci;
        _ub(6 * i + 4) = _fmax;

        // wheel velocity
        _lb(6*i + 5) = -25;
        _ub(6*i + 5) = 25;
    }

    // set weight matrix
    _Q.setZero();
    _Q.diagonal() = _Qx.replicate(_horizon, 1);
    _R.setIdentity(4*_n_contact, 4*_n_contact);
    _R *= _alpha;

    // regulate distance bettween contact point and CoM
    Mat Ap(8,25);
    Ap.setZero();
    for (int i(0); i < 4; i++) {
        Ap.block(2*i,3,2,2).setIdentity();
        Ap.block(2*i,3,2,2) *= -1;
        Ap.block(2*i,13+3*i,2,2).setIdentity();
    }
    _A.setZero();
    _W.setZero();
    _W.diagonal() = _Wxy.replicate(_horizon, 1);
    for (int j(0); j < _horizon; j++) {
        _A.block(8*j,25*j,8,25) = Ap;
        _Rxy.segment(8*j,8) = _r_xy;
        for (int leg(0); leg < 4; leg++) { // no regulate swing leg
            _W.block(8*j+2*leg, 8*j+2*leg, 2, 2) *= _contactTable.col(j)(leg);
        }
    }

    // formulate QP
    Timer tc2;
    Mat par = _Q * Su;
    _H.noalias() = _R + Su.transpose() * par + Su.transpose()*_A.transpose()*_W*_A*Su;
    _g.noalias() = par.transpose() * (Sx * _x0 - _desiredDiscreteTraj) + Su.transpose()*_A.transpose()*(_A*Su*_x0 - _Rxy);
    // cout << "Timer tc2: " << tc2.getMs() << endl;
    Timer tc;
#ifdef USE_QPOASES
    solver = new qpOASES::QProblem(4 * _n_contact, 5 * _n_contact);
    qpOASES::int_t nWSR = 500;
    solver->reset();
    qpOASES::Options opt;
    opt.setToMPC();
    opt.enableEqualities = qpOASES::BT_TRUE;
    opt.printLevel = qpOASES::PL_NONE;
    solver->setOptions(opt);
    solver->init(_H.data(), _g.data(), _C.data(), nullptr, nullptr, _lb.data(), _ub.data(), nWSR);
    _optimalControlInputs.resize(4 * _n_contact);
    if (solver->isSolved())
    {
        solver->getPrimalSolution(_optimalControlInputs.data());
    }
    else
    {
        throw std::runtime_error("qp solver failed");
    }
    delete solver;
    // cout << "qp1: " << _optimalControlInputs.transpose() << endl;
#else
    eiquadprog_solver.reset(3 * _n_contact, 0, 10 * _n_contact);
    Mat Cin(_C.rows() * 2, 3 * _n_contact);
    Cin << _C, -_C;
    Vec cin(_C.rows() * 2);
    cin << -_lb, _ub;
    _optimalControlInputs.resize(4 * _n_contact);
    auto state = eiquadprog_solver.solve_quadprog(_H, _g, Mat::Zero(0, 3 * _n_contact), Vec::Zero(0), Cin, cin, _optimalControlInputs);
    if (state != eiquadprog::solvers::EIQUADPROG_FAST_OPTIMAL)
    {
        throw runtime_error("Wheel_SRB_MPC::solve() qp failed");
    }
    // cout << "qp2: " << _optimalControlInputs.transpose() << endl;
#endif
    // cout << "mpc solver time cost: " << tc.getMs() << endl;

    _discreteOptimizedStates = Sx * _x0 + Su * _optimalControlInputs;
    for (int i = 0; i < _horizon; i++)
    {
        computeAtBt(i);
        if (i == 0)
        {
            _xDot.segment(i * 25, 25) =
                    _At * _x0 + _Bt * _optimalControlInputs.segment(0, _contactTable.col(i).sum() * 4);
        }
        else
        {
            _xDot.segment(i * 25, 25) =
                    _At * _discreteOptimizedStates.segment(i * 25 - 25, 25) +
                    _Bt * _optimalControlInputs.segment(_contactTable.leftCols(i).sum() * 4,
                                                       _contactTable.col(i).sum() * 4);
        }
    }

    fill(_ext_wrench.begin(), _ext_wrench.end(), Vec6::Zero());
}

void Wheel_SRB_MPC_IMPL::computeSxSu()
{

    Sx.setZero(25 * _horizon, 25);
    Su.setZero(25 * _horizon, 4 * _n_contact);

    for (size_t k = 0; k < _horizon; k++)
    {
        computeAtBtAndBiasTraj(k);
        Mat P0(_At.rows() + _Bt.cols(), _At.cols() + _Bt.cols());
        P0.setZero();
        P0.topRows(_At.rows()) << _At, _Bt;
        P0 *= _dt;
        Mat P0_exp = P0.exp();
        _Ak = P0_exp.topLeftCorner(_At.rows(), _At.cols());
        _Bk = P0_exp.topRightCorner(_Bt.rows(), _Bt.cols());
        if (k == 0)
        {
            Sx.middleRows(k * 25, 25).noalias() = _Ak;
            Su.topLeftCorner(25, _contactTable.col(k).sum() * 4).noalias() = _Bk;
        }
        else
        {
            Sx.middleRows(k * 25, 25).noalias() = _Ak * Sx.middleRows((k - 1) * 25, 25);
            size_t sc = 4 * _contactTable.leftCols(k).sum();
            // printf("k = %ld, sc = %ld, nc = %ld\n", k, sc, _n_contact);
            Su.block(k * 25, 0, 25, sc).noalias() =
                    _Ak * Su.block((k - 1) * 25, 0, 25, sc);
            Su.block(k * 25, sc, 25, _contactTable.col(k).sum() * 4).noalias() = _Bk;
        }
    }
}

void Wheel_SRB_MPC_IMPL::setFrictionCoefficient(double mu)
{
    _mu = mu;
}

void Wheel_SRB_MPC_IMPL::setContactPointPos(vector<Vec3> contactPointPos)
{
    assert(contactPointPos.size() == 4);
    _contactPointPos = contactPointPos;
}

void Wheel_SRB_MPC_IMPL::setMaxForce(double fmax)
{
    _fmax = fmax;
}

void Wheel_SRB_MPC_IMPL::computeAtBt(size_t t)
{
    // TODO: compute At, Bt

    double pc = cos(_x0(1));
    double yc = cos(_x0(2));
    double ys = sin(_x0(2));
    double pt = tan(_x0(1));

    T_inv << yc / pc, ys / pc, 0,
            -ys, yc, 0,
            yc * pt, ys * pt, 1;
    R_wb = rpyToRotMat(_x0.head(3)).transpose();
    Rz << yc, -ys, 0.,
          ys, yc, 0.,
          0., 0., 1;

    _At.resize(25, 25);
    _At.setZero();
    _At.block(0, 6, 3, 3) = T_inv;
    _At.block(3, 9, 3, 3).setIdentity();
    _At(11, 12) = 1.;

    Iworld = R_wb * _inertia * R_wb.transpose();
    Iw_inv = Iworld.inverse();

    size_t n_support_contact = _contactTable.col(t).sum();
    _Bt.resize(25, 4 * n_support_contact);
    _Bt.setZero();
    Vec3 r ,Ix;
    Ix << 1, 0, 0;
    int k = 0;
    for (int leg(0); leg < 4; leg++)
    {
        if (_contactTable(leg, t) == 1)
        {
            r = _contactPointPos[leg] - _x0.segment(3, 3);
            Mat3 r_skew;
            r_skew << 0., -r(2), r(1),
                    r(2), 0., -r(0),
                    -r(1), r(0), 0.;
            _Bt.block(6, k * 4, 3, 3) = Iw_inv * r_skew;
            _Bt.block(9, k * 4, 3, 3) = 1 / _mass * Mat3::Identity();
            _Bt.block(13 + 3*k, 4*k+3, 3, 1) = Rz*Ix;
            k++;
        }
    }
}

void Wheel_SRB_MPC_IMPL::computeAtBtAndBiasTraj(size_t t)
{
    // TODO: compute At, Bt

    double pc = cos(_x0(1));
    double yc = cos(_x0(2));
    double ys = sin(_x0(2));
    double pt = tan(_x0(1));

    T_inv << yc / pc, ys / pc, 0,
            -ys, yc, 0,
            yc * pt, ys * pt, 1;

    R_wb = rpyToRotMat(_x0.head(3)).transpose();
    Rz << yc, -ys, 0.,
          ys, yc, 0.,
          0., 0., 1;

    _At.resize(25, 25);
    _At.setZero();
    _At.block(0, 6, 3, 3) = T_inv;
    _At.block(3, 9, 3, 3).setIdentity();
    _At(11, 12) = 1.;

    Iworld = R_wb * _inertia * R_wb.transpose();
    Iw_inv = Iworld.inverse();

    Vec3 f_ext, tau_ext;
    f_ext = _ext_wrench[t].head(3);
    tau_ext = _ext_wrench[t].tail(3);
    if (t == 0)
    {
        _desiredDiscreteTraj_bias.segment(25 * t + 6, 3) = _dt * Iw_inv * tau_ext;
        _desiredDiscreteTraj_bias.segment(25 * t + 9, 3) = _dt * f_ext / _mass;
        _desiredDiscreteTraj_bias.segment(25 * t + 3, 3) = 0.5 * _dt * _dt * f_ext / _mass;
        _desiredDiscreteTraj_bias.segment(25 * t, 3) = 0.5 * _dt * _dt * T_inv * (Iw_inv * tau_ext);
    }
    else
    {
        size_t _last = t - 1;
        RealNum tt = 0.5 * _dt * _dt;
        _desiredDiscreteTraj_bias.segment(25 * t + 6, 3) =
                _desiredDiscreteTraj_bias.segment(25 * _last + 6, 3) + _dt * Iw_inv * tau_ext;
        _desiredDiscreteTraj_bias.segment(25 * t + 9, 3) =
                _desiredDiscreteTraj_bias.segment(25 * _last + 9, 3) + _dt * f_ext / _mass;
        _desiredDiscreteTraj_bias.segment(25 * t + 3, 3) =
                _desiredDiscreteTraj_bias.segment(25 * _last + 3, 3) + tt * f_ext / _mass;
        _desiredDiscreteTraj_bias.segment(25 * t, 3) =
                _desiredDiscreteTraj_bias.segment(25 * _last, 3) + tt * T_inv * (Iw_inv * tau_ext);
    }

    size_t n_support_contact = _contactTable.col(t).sum();
    _Bt.resize(25, 4 * n_support_contact);
    _Bt.setZero();
    Vec3 r ,Ix;
    Ix << 1, 0, 0;
    int k = 0;
    for (int leg(0); leg < 4; leg++)
    {
        if (_contactTable(leg, t) == 1)
        {
            r = _contactPointPos[leg] - _x0.segment(3, 3);
            Mat3 r_skew;
            r_skew << 0., -r(2), r(1),
                    r(2), 0., -r(0),
                    -r(1), r(0), 0.;
            _Bt.block(6, k * 4, 3, 3) = Iw_inv * r_skew;
            _Bt.block(9, k * 4, 3, 3) = 1 / _mass * Mat3::Identity();
            _Bt.block(13 + 3*k, 4*k+3, 3, 1) = Rz*Ix;
            k++;
        }
    }
}

Mat3 Wheel_SRB_MPC_IMPL::coordinateRotation(CoordinateAxis axis, double theta)
{
    RealNum s = std::sin(theta);
    RealNum c = std::cos(theta);

    Mat3 R;

    if (axis == CoordinateAxis::X)
    {
        R << 1, 0, 0, 0, c, s, 0, -s, c;
    }
    else if (axis == CoordinateAxis::Y)
    {
        R << c, 0, -s, 0, 1, 0, s, 0, c;
    }
    else if (axis == CoordinateAxis::Z)
    {
        R << c, s, 0, -s, c, 0, 0, 0, 1;
    }

    return R;
}

Mat3 Wheel_SRB_MPC_IMPL::rpyToRotMat(Vec3Ref v)
{
    static_assert(v.ColsAtCompileTime == 1 && v.RowsAtCompileTime == 3,
                  "must have 3x1 vector");
    Mat3 m = coordinateRotation(CoordinateAxis::X, v[0]) *
             coordinateRotation(CoordinateAxis::Y, v[1]) *
             coordinateRotation(CoordinateAxis::Z, v[2]);
    return m;
}

ConstVecRef Wheel_SRB_MPC_IMPL::getXDot()
{

    return _xDot;
}

void Wheel_SRB_MPC_IMPL::setDesiredDiscreteTrajectory(ConstVecRef traj)
{
    assert(traj.size() == 25 * _horizon);
    _desiredDiscreteTraj = traj;
    _setDesiredDiscreteTraj = true;
}
