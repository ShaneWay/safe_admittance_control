#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace std;

#define TimeUnit 0.001

constexpr double M_x = 0.5;
constexpr double B_x = 1.;

constexpr double K = 5000.;
constexpr double B = 30.;
constexpr double L = 0.5;

constexpr double M = 0.3;

constexpr double init_angle = 120. / 180. * M_PI;

constexpr double F_max = 15.;



class controller
{
    private:
        
        vector<double> tao;
        vector<double> tao_star;

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
        int32_t frame;

        vector<double> f;
        vector<double> f_d;


    public:
        controller();
        double get_tau(const double & T, double & f_ext_from_sensor, double& q_frome_sensor);
        double get_tau_normal(const double & T, double & f_input, double& q_input);

        void refresh(const double & T);
        void refresh_normal(const double & T);

        static double proj(double tao_star_input);
        static double proj_Q(double u_x_star);

        void plot_tao();
        void plot_tao_realtime(vector<double>& f);
        void plot_q();
        void plot_q_realtime(vector<double>& qs);

        void plot_q0();
    
        ~controller();


};




#endif