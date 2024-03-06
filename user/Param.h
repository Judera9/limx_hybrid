//
// Created by wenchun on 3/17/21.
//

#ifndef XIAOTIANHYBRID_PARAM_H
#define XIAOTIANHYBRID_PARAM_H

#include "ParamHandler.hpp"
#include "Configuration.h"

class UserParameterHandler {
public:
    explicit UserParameterHandler(std::string yaml_file) : paramHandler(yaml_file) {
        bool value = true;
        value &= paramHandler.getString("urdf", urdf);
        urdf = std::string(THIS_COM) + urdf;
        value &= paramHandler.getValue("horizons", horizons);
        value &= paramHandler.getValue("ctrl_num", ctrl_num);
        value &= paramHandler.getValue("dt", dt);

        value &= paramHandler.getString("lf_wheel", LF_wheel);
        value &= paramHandler.getString("rf_wheel", RF_wheel);
        value &= paramHandler.getString("lh_wheel", LH_wheel);
        value &= paramHandler.getString("rh_wheel", RH_wheel);

        value &= paramHandler.getString("lf_hip", LF_HFE);
        value &= paramHandler.getString("rf_hip", RF_HFE);
        value &= paramHandler.getString("lh_hip", LH_HFE);
        value &= paramHandler.getString("rh_hip", RH_HFE);

        value &= paramHandler.getString("lf_foot", LF_foot);
        value &= paramHandler.getString("rf_foot", RF_foot);
        value &= paramHandler.getString("lh_foot", LH_foot);
        value &= paramHandler.getString("rh_foot", RH_foot);

        value &= paramHandler.getString("base", BASE);
        value &= paramHandler.getValue("dtMPC", dtMPC);
        value &= paramHandler.getValue("mu", mu);
        value &= paramHandler.getVector("Q_rpy", Q_rpy);
        value &= paramHandler.getVector("Q_pos", Q_pos);
        value &= paramHandler.getVector("Q_omega", Q_omega);
        value &= paramHandler.getVector("Q_vel", Q_vel);
        value &= paramHandler.getValue("leg_index", leg_index);
        value &= paramHandler.getValue("estimation_mode", estimation_mode);
        value &= paramHandler.getVector("des_pos", des_pos);
        value &= paramHandler.getVector("des_vel", des_vel);
        value &= paramHandler.getVector("des_tau", des_tau);
        value &= paramHandler.getVector("des_foot_state", des_foot_state);
        value &= paramHandler.getValue("rw", rw);
        value &= paramHandler.getValue("height", height);
        value &= paramHandler.getValue("qLegDes", qLegDes);
        value &= paramHandler.getValue("vLegDes", vLegDes);
        value &= paramHandler.getValue("jointsSigns", jointsSigns);
        value &= paramHandler.getValue("KP", KP);
        value &= paramHandler.getValue("KD", KD);
        value &= paramHandler.getValue("test", test);
        value &= paramHandler.getValue("bool_sim", bool_sim);

        if (!value) {
            throw std::runtime_error("init param failed ...");
        }
    }

    std::string urdf, LF_wheel, RF_wheel, LH_wheel, RH_wheel,
            LF_foot, RF_foot, LH_foot, RH_foot,
            LF_HFE, RF_HFE, LH_HFE, RH_HFE, BASE;
    int horizons;
    int ctrl_num;
    int leg_index, estimation_mode;
    double dt;
    double dtMPC;
    double mu;
    std::vector<double> Q_rpy, Q_pos, Q_omega, Q_vel, des_pos, des_vel, des_tau, des_foot_state, test;
    double rw;
    double height;
    std::vector<double> qLegDes, vLegDes, jointsSigns, KP, KD;
    int bool_sim;

private:
    ParamHandler paramHandler;
};

#endif //XIAOTIANHYBRID_PARAM_H
