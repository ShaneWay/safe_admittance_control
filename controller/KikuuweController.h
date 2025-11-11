#pragma once
#include "BaseController.h"
#include <iostream>
#include <numeric>

class KikuuweController : public BaseController {
public:
    KikuuweController(const Config& config) : BaseController(config)
    {

    }

    double getTorqueOnForceControl(const double & T, double & f_ext_from_sensor, double& q_frome_sensor) override
    {
        f.push_back(f_ext_from_sensor);
        q_s.push_back(q_frome_sensor);
        q_s_hat.push_back((q_s[frame] - q_s[frame - 1])/T);
        cout << "f[frame " << frame << "] = " << f[frame] << endl;
        cout << "q[frame " << frame << "] = " << q_s[frame] << endl;

        double K_hat = K + B / T + L * T;

        double u_x_star_tem = ((M_x * q_x_hat[frame - 1]) + T * (f[frame] + f_d[frame - 1]))
        / (M_x + B_x * T);
        u_x_star.push_back(u_x_star_tem);
        // std::cout << "f_d: " << f_d[frame] << std::endl;

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

        double tao_star_tem = Mat * (q_x_star[frame] - q_s_star[frame]);
    
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
    
        double a_tem = a[frame - 1]  + T * (q_x[frame] - q_s[frame]);
        a.push_back(a_tem);

        frame++;
    }


    double getTorqueOnPositionControl(const double & T, double & f_ext_from_sensor, double& q_frome_sensor) override
    {
        f.push_back(f_ext_from_sensor);
        q_s.push_back(q_frome_sensor);
        q_s_hat.push_back((q_s[frame] - q_s[frame - 1])/T);
        cout << "f[frame " << frame << "] = " << f[frame] << endl;
        cout << "q[frame " << frame << "] = " << q_s[frame] << endl;
        cout << "kikuuwe_position_control" << endl;
        cout << "q0[frame " << frame << "] = " << q0[frame] << endl;

        double K_hat = K + B / T + L * T;
        // get new u_x_star
        double u_x_star_tem =  (M_x * u_x[frame - 1] + T * (M_x * q0_ddot[frame] + B_x * q0_dot[frame] + K_x * q0[frame] + f[frame])) / (M_x + B_x * T );
        u_x_star.push_back(u_x_star_tem);
        
        // get new q_x_star
        double q_x_star_tem = q_x[frame - 1] + T * u_x_star[frame];
        q_x_star.push_back(q_x_star_tem);

        // get new phi_b
        double phi_b_tem;
        phi_b_tem = (B * (q_x[frame - 1] - q_s[frame - 1])) / T - L * a[frame - 1];
        phi_b.push_back(phi_b_tem);

        // get new phi a
        double phi_a_tem;
        phi_a_tem = M * (q_s[frame] - q_x[frame - 1] - T * u_x[frame - 1]) / (T * T);
        phi_a.push_back(phi_a_tem);

        // get new q_star
        double q_s_star_tem;
        q_s_star_tem = q_s[frame] + (phi_b[frame] - phi_a[frame]) / (K_hat + M / (T * T));
        q_s_star.push_back(q_s_star_tem);

        // get mat1 and mat2
        double Mat1  = 1 + K_x * (T * T) / (M_x + B_x * T );
        double Mat2 = K_hat + M / (T * T);

        // get new tau_star
        double tau_star_tem;
        tau_star_tem = Mat2 / Mat1 * q_x_star[frame] - Mat2 * q_s_star[frame];
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
