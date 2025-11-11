#pragma once
#include <iostream>
#include <vector>
#include <variant>
#include "ConfigLoader.h"

using namespace std;

enum class ControlTarget { ForceControl, PositionControl };

class BaseController {
    public:
        double M_x;
        double B_x;
        double K_x;

        double K;
        double B;
        double L;
        
        double M;

        double init_angle;

        double F_max;
        double f_d_tem;
        double Q_max;
        double dt;
        double SimTime;
        std::string control_mode;
        std::string control_target;

        vector<double> tau;
        vector<double> tau_star;

        vector<double> u_x;
        vector<double> u_x_star;

        vector<double> q_x;
        vector<double> q_x_hat;
        vector<double> q_x_star;

        vector<double> phi_b;
        vector<double> phi_a;

        vector<double> q_s;
        vector<double> q_s_star;

        vector<double> a;
        vector<double> integral_a;
        int32_t frame;
        vector<double> f;
        vector<double> f_d;
        vector<double> q_s_hat;

        vector<double> q0;
        vector<double> q0_dot;
        vector<double> q0_ddot;
        
    public:
        BaseController (const Config& config);
        virtual ~BaseController() = default;

        double getTorque(double & f_ext_from_sensor, double& q_frome_sensor);
        void refresh();
        void parseTarget(const Config& config);
        void printParams() const;
        void plot_tau();
        void plot_real_tau();
        void plot_q();
        void plot_q0();
        void plot_q_hat();

        void generateJointTrajectory();

        double proj(double tao_star_input);

    protected:
        virtual double getTorqueOnForceControl(double & f_ext_from_sensor, double& q_frome_sensor) = 0;
        virtual void refreshOnForceControl() = 0;
        virtual double getTorqueOnPositionControl(double & f_ext_from_sensor, double& q_frome_sensor) = 0;
        virtual void refreshOnPositionControl() = 0;
    
    private:
        ControlTarget target_;

};