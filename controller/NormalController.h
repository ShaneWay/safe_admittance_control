#pragma once
#include "BaseController.h"
#include <iostream>
#include <numeric>

class NormalController : public BaseController {
public:
    NormalController(const Config& config) : BaseController(config)
    {
        
    }

    double getTorqueOnForceControl(double & f_ext_from_sensor, double& q_frome_sensor) override
    {
        f.push_back(f_ext_from_sensor);
        q_s.push_back(q_frome_sensor);
        q_s_hat.push_back((q_s[frame] - q_s[frame - 1])/dt);
        cout << "f[frame " << frame << "] = " << f[frame] << endl;
        cout << "q[frame " << frame << "] = " << q_s[frame] << endl;

        double K_hat = K + B / dt + L * dt;
        double u_x_star_tem = ((M_x * q_x_hat[frame - 1]) + dt * (f[frame] + f_d[frame - 1]))
        / (M_x + B_x * dt);
        u_x_star.push_back(u_x_star_tem);

        double q_x_tem = q_x[frame - 1] + dt * u_x_star[frame];
        q_x.push_back(q_x_tem);

        double phi_b_tem = (B * (q_x[frame - 1] - q_s[frame - 1])) / dt - L * a[frame - 1];
        phi_b.push_back(phi_b_tem);

        double phi_a_tem = M * (q_s[frame] - q_x[frame - 1] - dt * q_x_hat[frame - 1]) / (dt * dt);
        phi_a.push_back(phi_a_tem);

        double q_s_star_tem = q_s[frame] + (phi_b[frame] - phi_a[frame]) / (K_hat + M / (dt * dt));

        double Mat = K_hat + M / (dt * dt);
        double tau_tem = Mat * (q_x[frame] - q_s_star_tem);
        tau.push_back(proj(tau_tem));

        return tau[frame];
    }

    void refreshOnForceControl() override
    {
        double K_hat = K + B / dt + L * dt;
        double q_x_hat_tem = (q_x[frame] - q_x[frame - 1]) / dt;
        q_x_hat.push_back(q_x_hat_tem);

        double a_tem = a[frame - 1]  + dt * (q_x[frame] - q_s[frame]);
        a.push_back(a_tem);

        frame++;
    }

    double getTorqueOnPositionControl(double & f_ext_from_sensor, double& q_frome_sensor) override
    {
        f.push_back(f_ext_from_sensor);
        q_s.push_back(q_frome_sensor);
        q_s_hat.push_back((q_s[frame] - q_s[frame - 1])/dt);
        cout << "f[frame " << frame << "] = " << f[frame] << endl;
        cout << "q[frame " << frame << "] = " << q_s[frame] << endl;
        
        double K_hat = K + B / dt + L * dt;
        double q_x_ddot;
        q_x_ddot =(-B_x * (q_x_hat[frame - 1] -q0_dot[frame]) - K_x * (q_x[frame - 1] - q0[frame]) + f[frame]) / M_x + q0_ddot[frame];

        double q_x_dot_tem;
        q_x_dot_tem = q_x_hat[frame - 1] + dt * q_x_ddot;
        q_x_hat.push_back(q_x_dot_tem);

        double q_x_tem;
        q_x_tem = q_x[frame - 1] + dt * q_x_hat[frame];
        q_x.push_back(q_x_tem);

        double a_tem;
        a_tem = a[frame - 1] + dt * (q_x[frame] - q_s[frame]);
        a.push_back(a_tem);

        double q_s_dot;
        q_s_dot = (q_s[frame] - q_s[frame - 1]) / dt;

        double tau_star_tem;
        tau_star_tem = M * q_x_ddot + K * (q_x[frame] - q_s[frame]) + B * (q_x_hat[frame] - q_s_dot) + L * a[frame];
        tau_star.push_back(tau_star_tem);

        cout << "tau_star:" <<  tau_star_tem << endl;

        double tau_tem;
        tau_tem = proj(tau_star_tem);
        tau.push_back(tau_tem);

        return tau_tem;


    }

    void refreshOnPositionControl(const double & T) override
    {
        frame++;
    }


};
