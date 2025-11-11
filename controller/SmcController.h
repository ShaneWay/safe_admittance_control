#pragma once
#include "BaseController.h"
#include <iostream>
#include <numeric>

class SmcController : public BaseController {
private:

public:
    SmcController(const Config& config) : BaseController(config)
    {

    }
    double getTorqueOnForceControl(const double & T, double & f_ext_from_sensor, double& q_frome_sensor) override
    {
        f.push_back(f_ext_from_sensor);
        q_s_hat.push_back((q_s[frame] - q_s[frame - 1])/T);
        q_s.push_back(q_frome_sensor);
        cout << "f[frame " << frame << "] = " << f[frame] << endl;
        cout << "q[frame " << frame << "] = " << q_s[frame] << endl;

        // f_d.push_back(f_d_tem);

        double K_hat = K + B / T + L * T;

        double u_x_star_tem = ((M_x * q_x_hat[frame - 1]) + T * (f[frame] + f_d[frame - 1]))
        / (M_x + B_x * T);
        u_x_star.push_back(u_x_star_tem);
        // std::cout << "f_d: " << f_d[frame] << std::endl;

        double q_x_star_hat = proj_Q(u_x_star_tem);
        std::cout << "q_x_star_hat: " << q_x_star_hat << std::endl;

        double lamda = f[frame] + f_d[frame] - M_x * (q_x_star_hat - q_x_hat[frame - 1])/ T - B_x * q_x_star_hat;
        // std::cout << "lamda: " << lamda << std::endl;

        double q_x_star_tem = q_x[frame - 1] + T * u_x_star[frame];
        // std::cout << "q_x_star: " << q_x_star_tem << std::endl;
        q_x_star.push_back(q_x_star_tem);

        double phi_b_tem = (B * (q_x[frame - 1] - q_s[frame - 1])) / T - L * a[frame - 1];
        phi_b.push_back(phi_b_tem);
        // std::cout << "phi_b_tem: " << phi_b_tem << std::endl;


        double phi_a_tem = M * (q_s[frame] - q_x[frame - 1] - T * q_x_hat[frame - 1]) / (T * T);
        phi_a.push_back(phi_a_tem);
        // std::cout << "phi_a_tem: " << phi_a_tem << std::endl;


        double q_s_star_tem = q_s[frame] + (phi_b[frame] - phi_a[frame]) / (K_hat + M / (T * T));
        q_s_star.push_back(q_s_star_tem);
        // std::cout << "q_s_star: " << q_s_star_tem << std::endl;


        double Mat = K_hat + M / (T * T);

        double tao_star_tem = Mat * ((q_x_star[frame] - q_s_star[frame]) - T * T * lamda / (M_x + T * B_x));
    
        tau_star.push_back(tao_star_tem);

        std::cout << "tao_star: " << tao_star_tem << endl;
        double tau_tem = proj(tau_star[frame]);
        tau.push_back(tau_tem);

        return tau[frame];
    }

    void refreshOnForceControl(const double & T) override
    {
        double K_hat = K + B / T + L * T;
        double q_x_tem = q_s_star[frame] + tau[frame] / (K_hat + M / (T * T));
        q_x.push_back(q_x_tem);

        double q_x_hat_tem = (q_x[frame] - q_x[frame - 1]) / T;
        q_x_hat.push_back(q_x_hat_tem);
        std::cout << "q_x_hat_tem: " << q_x_hat_tem << std::endl;
    
        double a_tem = a[frame - 1]  + T * (q_x[frame] - q_s[frame]);
        a.push_back(a_tem);

        frame++;
    }

    double proj_Q(double u_x_star)
    {   
        if (u_x_star < -Q_max)
        {
            return -Q_max;
        }
        else if (u_x_star > Q_max)
        {
            return Q_max;
        }
        else{
            return u_x_star;
        }
    }

    // double getTorqueOnPositionControl(const double & T, double & f_ext_from_sensor, double& q_frome_sensor) override
    // {
    //     f.push_back(f_ext_from_sensor);
    //     q_s.push_back(q_frome_sensor);
    //     q_s_hat.push_back((q_s[frame] - q_s[frame - 1])/T);
    //     cout << "f[frame " << frame << "] = " << f[frame] << endl;
    //     cout << "q[frame " << frame << "] = " << q_s[frame] << endl;

    //     double K_hat = K + B / T + L * T;
    //     double q0_star = M_x * q0_ddot[frame] + B_x * q0_dot[frame] + K_x * q0[frame];
    //     double u_x_star_tem = ((M_x * q_x_hat[frame - 1]) + T * q0_star - T * K_x * q_x[frame - 1])
    //     / (M_x + B_x * T + T * T * K_x);
    //     u_x_star.push_back(u_x_star_tem);

    //     double q_x_star_hat = proj_Q(u_x_star_tem);
        
    //     double q_x_tem = q_x_star_hat * T +q_x[frame - 1];

    //     double lamda = q0_star - M_x * (q_x_star_hat- q_x_hat[frame - 1])/ T - B_x * q_x_star_hat - K_x * q_x_tem;

    //     double q_x_star_tem = q_x[frame - 1] + T * u_x_star[frame];
    //     // std::cout << "q_x_star: " << q_x_star_tem << std::endl;
    //     q_x_star.push_back(q_x_star_tem);

    //     double phi_b_tem = (B * (q_x[frame - 1] - q_s[frame - 1])) / T - L * a[frame - 1];
    //     phi_b.push_back(phi_b_tem);
    //     // std::cout << "phi_b_tem: " << phi_b_tem << std::endl;


    //     double phi_a_tem = M * (q_s[frame] - q_x[frame - 1] - T * q_x_hat[frame - 1]) / (T * T);
    //     phi_a.push_back(phi_a_tem);
    //     // std::cout << "phi_a_tem: " << phi_a_tem << std::endl;


    //     double q_s_star_tem = q_s[frame] + (phi_b[frame] - phi_a[frame]) / (K_hat + M / (T * T));
    //     q_s_star.push_back(q_s_star_tem);
    //     // std::cout << "q_s_star: " << q_s_star_tem << std::endl;


    //     double Mat = K_hat + M / (T * T);

    //     double tao_star_tem = Mat * ((q_x_star[frame] - q_s_star[frame]) - T * T * lamda / (M_x + T * B_x +  T * T * K_x));
    
    //     tau_star.push_back(tao_star_tem);

    //     std::cout << "tao_star: " << tao_star_tem << endl;
    //     double tau_tem = proj(tau_star[frame]);
    //     tau.push_back(tau_tem);

    //     return tau[frame];


    // }

    // void refreshOnPositionControl(const double & T) override
    // {
    //     double K_hat = K + B / T + L * T;
    //     double q_x_tem = q_s_star[frame] + tau[frame] / (K_hat + M / (T * T));
    //     q_x.push_back(q_x_tem);

    //     double q_x_hat_tem = (q_x[frame] - q_x[frame - 1]) / T;
    //     q_x_hat.push_back(q_x_hat_tem);
    //     std::cout << "q_x_hat_tem: " << q_x_hat_tem << std::endl;
    
    //     double a_tem = a[frame - 1]  + T * (q_x[frame] - q_s[frame]);
    //     a.push_back(a_tem);

    //     frame++;
    // }


    double getTorqueOnPositionControl(const double & T, double & f_ext_from_sensor, double& q_frome_sensor) override
    {
        f.push_back(f_ext_from_sensor);
        q_s.push_back(q_frome_sensor);
        q_s_hat.push_back((q_s[frame] - q_s[frame - 1])/T);
        cout << "f[frame " << frame << "] = " << f[frame] << endl;
        cout << "q[frame " << frame << "] = " << q_s[frame] << endl;

        double K_hat = K + B / T + L * T;
        // get new u_x_star
        double u_x_star_tem =  (M_x * u_x[frame - 1] + T * (M_x * q0_ddot[frame] + B_x * q0_dot[frame] + K_x * q0[frame] + f[frame])) / (M_x + B_x * T );
        u_x_star.push_back(u_x_star_tem);
        
        // get new q_x_star
        double q_x_star_tem = q_x[frame - 1] + T * u_x_star[frame];
        q_x_star.push_back(q_x_star_tem);

        double h1 = 1. + T * T * K_x / (M_x + B_x * T );
        double h2 = K_hat + M / (T * T);

        double q_x_lim = q_x[frame - 1] + T * proj_Q((q_x_star[frame] / h1 - q_x[frame - 1]) / T);
        double lamda = (M_x + B_x * T ) / (T * T) *(q_x_star[frame] - h1 * q_x_lim);

        // get new phi_b
        double phi_b_tem;
        phi_b_tem = (B * (q_x[frame - 1] - q_s[frame - 1])) / T - L * a[frame-1];
        phi_b.push_back(phi_b_tem);

        // get new phi a
        double phi_a_tem;
        phi_a_tem = M * (q_s[frame] - q_x[frame - 1] - T * u_x[frame - 1]) / (T * T);
        phi_a.push_back(phi_a_tem);

        // get new q_star
        double q_star_tem;
        q_star_tem = q_s[frame] + (phi_b[frame] - phi_a[frame]) / (K_hat + M / (T * T));
        q_s_star.push_back(q_star_tem);

        // get new tau_star
        double tau_star_tem;
        tau_star_tem = h2 / h1 * q_x_star[frame] - h2 * q_s_star[frame] - h2 / h1 * T * T * lamda / (M_x + B_x * T );
        tau_star.push_back(tau_star_tem);

        // get new output tau
        double tau_tem;
        tau_tem = proj(tau_star_tem);
        tau.push_back(tau_tem);

        return tau_tem;

        

    }

    void refreshOnPositionControl(const double & T) override
    {
       double K_hat = K + B / T + L * T;

        // get new q_x
        double q_x_tem;
        q_x_tem = q_s_star[frame] + tau[frame] /  (K_hat + M / (T * T)) ;
        q_x.push_back(q_x_tem);

        //get new u_x
        double u_x_tem;
        u_x_tem = (q_x[frame] - q_x[frame - 1]) / T;
        u_x.push_back(u_x_tem);

        double q_x_hat_tem = (q_x[frame] - q_x[frame - 1]) / T;
        q_x_hat.push_back(q_x_hat_tem);
        std::cout << "q_x_hat_tem: " << q_x_hat_tem << std::endl;

        // get new a
        double a_tem;
        a_tem = a[frame - 1] + T * (q_x[frame] - q_s[frame]);
        a.push_back(a_tem);

        frame++;
    }
};
