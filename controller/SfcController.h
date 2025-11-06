#pragma once
#include "BaseController.h"
#include <iostream>
#include <numeric>

class SfcController : public BaseController {
public:
    SfcController(const Config& config) : BaseController(const Config& config)
    {

    }

    double getTorqueOnForceControl(const double & T, double & f_ext_from_sensor, double& q_frome_sensor) override
    {
        f.push_back(f_ext_from_sensor);
        q_s.push_back(q_frome_sensor);
        q_s_hat.push_back((q_s[frame] - q_s[frame - 1])/T);
        cout << "f[frame " << frame << "] = " << f[frame] << endl;
        cout << "q[frame " << frame << "] = " << q_s[frame] << endl;
    
        double a_tem = (f[frame] + f_d[frame - 1] - B_x * std::abs(q_x_hat[frame - 1]) * std::abs(q_x_hat[frame -   1]) * q_x_hat[frame -1]) / M_x;
        a.push_back(a_tem);
        double q_x_hat_tem = q_x_hat[frame - 1] + a[frame] * T;
        q_x_hat.push_back(q_x_hat_tem);

        double q_x_tem = q_x[frame - 1] + T * q_x_hat[frame];
        q_x.push_back(q_x_tem);

        double integral_a_tem = integral_a[frame - 1] + T * (q_x[frame] - q_s[frame]);
        integral_a.push_back(integral_a_tem);
        double tau_star_tem = M * a[frame] + K * (q_x[frame] - q_s[frame]) + B * (q_x_hat[frame] - q_s_hat[frame])  + L * integral_a[frame];
        tau_star.push_back(tau_star_tem);
        std::cout << "tau_star: " << tau_star_tem << endl;
    
        tau.push_back(proj(tau_star_tem));

        return tau[frame];
    }

    void refreshOnForceControl(const double & T) override
    {
        frame++;
    }


}
