#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "ConfigLoader.h"

using namespace std;


enum class ControlMode
{
    NORMAL,
    SFC,
    SMC
};


class controller
{
    private:

        double M_x;
        double B_x;

        double K;
        double B;
        double L;

        double M;

        double init_angle;

        double F_max;
        double f_d_tem;
        double Q_max;
        double TimeUnit;
        
        std::string control_mode;

        
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

        ControlMode mode_;


    public:
        explicit controller(const Config& config);

        double getTorque(const double & T, double & f_ext_from_sensor, double& q_frome_sensor);

        double getTorqueNormal(const double & T, double & f_ext_from_sensor, double& q_frome_sensor);
        double getTorqueSMC(const double & T, double & f_ext_from_sensor, double& q_frome_sensor);
        double getTorqueSFC(const double & T, double & f_ext_from_sensor, double& q_frome_sensor);
        
        void refresh(const double & T);

        void refreshNormal(const double & T);
        void refreshSMC(const double & T);
        void refreshSFC(const double & T);


        void plot_tau();
        void plot_q();
        void plot_q_hat();
        // void plot_q_x_hat();

        virtual ~controller();
    
    private:
        ControlMode parseMode(const std::string& modeStr);
        double proj(double tao_star_input);
        double proj_Q(double u_x_star);

        


};




#endif