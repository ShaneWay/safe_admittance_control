#pragma once
#include <iostream>
#include <vector>
#include <variant>
#include "ConfigLoader.h"
#include <kinematics_model.h>

using namespace std;
typedef vector<Eigen::Vector2d> vv2d;


enum class ControlTarget { ForceControl, PositionControl };

class BaseController {
    public:
        Eigen::Matrix2d M_x;
        Eigen::Matrix2d B_x;
        Eigen::Matrix2d K_x;

        Eigen::Matrix2d K;
        Eigen::Matrix2d B;
        Eigen::Matrix2d L;
        
        Eigen::Matrix2d M;

        Eigen::Vector2d F_max;
        Eigen::Vector2d f_d_tem;
        Eigen::Vector2d Q_max;
        double dt;
        double SimTime;
        std::string controller_name;
        std::string control_target;

        Eigen::Vector2d init_angle;
        Eigen::Vector2d init_pos;
        double omega;
        double sin_T;
        double amplitude;
        double x_amplitude;

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

        
    public:
        BaseController (const ConfigLoader& loader);
        virtual ~BaseController() = default;

        Eigen::Vector2d getTorque(Eigen::Vector2d & f_ext_from_sensor, Eigen::Vector2d & q_frome_sensor);
        void refresh();
        void parseTarget(YAML::Node &ctrl);
        void printParams() const;
        void plot_tau();
        void plot_real_tau();
        void plotJointAngle();
        void plotJointSpeed();
        void plotExternalForce();
        void plotCartesianSpeed();
        void plotCartesianPosition();
        void plotGeneratedTrajectory();
        void generateJointTrajectory();

        Eigen::Vector2d proj(Eigen::Vector2d &tao_star_input);

    protected:
        virtual Eigen::Vector2d getTorqueOnForceControl(Eigen::Vector2d & f_ext_from_sensor, Eigen::Vector2d& q_frome_sensor) = 0;
        virtual void refreshOnForceControl() = 0;
        virtual Eigen::Vector2d getTorqueOnPositionControl(Eigen::Vector2d & f_ext_from_sensor, Eigen::Vector2d& q_frome_sensor) = 0;
        virtual void refreshOnPositionControl() = 0;
    
    private:
        ControlTarget target_;
        int count_total;
        

};