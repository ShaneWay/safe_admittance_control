#pragma once
#include "BaseController.h"
#include <iostream>
#include <numeric>

class SfcController : public BaseController {
public:
    SfcController(const Config& config) : BaseController(config)
    {

    }

    double getTorqueOnForceControl(double & f_ext_from_sensor, double& q_frome_sensor) override
    {

    }

    void refreshOnForceControl() override
    {

    }

    double getTorqueOnPositionControl(double & f_ext_from_sensor, double& q_frome_sensor) override
    {
        f.push_back(f_ext_from_sensor);
        q_s.push_back(q_frome_sensor);
        q_s_hat.push_back((q_s[frame] - q_s[frame - 1]) / dt);
        cout << "f[frame " << frame << "] = " << f[frame].transpose() << endl;
        cout << "q[frame " << frame << "] = " << q_s[frame].transpose() << endl;
        model.getJacobianMatrixTwoDOF(q_from_robot, jacobian_now);
        //update q and f_ext , get them from sensor
        tau_ext.push_back(jacobian_now.transpose() * f_from_sensor);

        int n = 3;
        Eigen::Vector2d q_x_hat_hat = q0_ddot[frame] + M_x.inverse() * (tau_ext[frame] - B_x * std::pow(std::abs(q_x_hat[frame - 1] - q0_dot[frame - 1]), n-1))* (q_x_hat[frame - 1] - q0_dot[frame - 1]) + K_x * (q_x[frame - 1] - q0[frame - 1]);
        
        q_x_hat.push_back(q_x_hat[frame - 1] + q_x_hat_hat * dt);
        q_x.push_back(q_x[frame - 1] + q_x_hat[frame] * dt);

        a.push_back(a[frame - 1] + (q_x[frame] - q_s[frame] * dt));
        tau_star.pushback(M * q_x_hat_hat + K * (q_x[frame] - q_s[frame]) + B * (q_x_hat[frame] - q_s_hat[frame]) + L * a[frame]);
        
        tau.push_back(proj(tau_star[frame]));

        return tau[frame];
    }

    void refreshOnPositionControl() override
    {
        frame++;
    }

};
