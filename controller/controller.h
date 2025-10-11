#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace std;

#define TimeUnit 0.002

constexpr double M_x = 2;
constexpr double B_x = 2;

constexpr double K = 500.;
constexpr double B = 30.;
constexpr double L = 30;

constexpr double M = 0.5;

constexpr double init_angle = 122. / 180. * M_PI;

constexpr double F_max = 6.;
constexpr double f_d_tem = -1.;
constexpr double Q_max = 0.8;



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

        vector<double> q_s_hat;


    public:
        controller();
        double get_tau(const double & T, double & f_ext_from_sensor, double& q_frome_sensor);

        void refresh(const double & T);

        static double proj(double tao_star_input);
        static double proj_Q(double u_x_star);

        void plot_tao();
        void plot_tao_realtime(vector<double>& f);
        void plot_q();
        void plot_q_hat();
        // void plot_q_x_hat();

        void plot_q_realtime(vector<double>& qs);
    
        ~controller();


};




#endif