#pragma once
#include <Eigen/Dense>
#include <iostream>
#include "ConfigLoader.h"
#include <variant>
#include <vector>
#include <BaseParams.h>
#include <kinematics_model.h>



using namespace std;
typedef vector<Eigen::Vector2d> vv2d;

class ControlState {
public:
    vv2d tau;
    vv2d tau_ext;
    vv2d tau_star;
    vv2d u_x;
    vv2d u_x_star;
    vv2d q_x;
    vv2d q_x_hat;
    vv2d q_x_star;
    vv2d phi_b;
    vv2d phi_a;
    vv2d q_s;
    vv2d q_s_star;
    vv2d a;
    vv2d integral_a;

    int32_t frame;
    vv2d f;
    vv2d f_d;
    vv2d q_s_hat;
    vv2d q0;
    vv2d q0_dot;
    vv2d q0_ddot;
    KortexKinematics model;
    vv2d X0;
    vv2d X0_dot;
    vv2d X0_ddot;
    vv2d X;
    vv2d X_hat;
    vv2d X_x;
    Eigen::Matrix2d jacobian_last;
    Eigen::Matrix2d jacobian_now;

    ControlState(const BaseParams& params);
    void plot_tau();
    void plot_real_tau();
    void plotJointAngle();
    void plotJointSpeed();
    void plotExternalForce();
    void plotCartesianSpeed();
    void plotCartesianPosition();
    void plotGeneratedTrajectory();
    void generateJointTrajectory();

    private:
        const BaseParams& params_;

};
