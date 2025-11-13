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
    }

    void refreshOnForceControl() override
    {
    }

    double getTorqueOnPositionControl(double & f_ext_from_sensor, double& q_frome_sensor) override
    {
        f.push_back(f_ext_from_sensor);
        q_s.push_back(q_frome_sensor);
        cout << "f[frame " << frame << "] = " << f[frame].transpose() << endl;
        cout << "q[frame " << frame << "] = " << q_s[frame].transpose() << endl;
        model.getJacobianMatrixTwoDOF(q_from_robot, jacobian_now);
        //update q and f_ext , get them from sensor
        tau_ext.push_back(jacobian_now.transpose() * f_from_sensor);
        
        Eigen::Matrix2d K_hat = K + B / dt + L * dt;
        
        Eigen::Vector2d q_x_ddot;
        q_x_ddot = M_x.inverse() * (-B_x * (q_x_hat[frame - 1] -q0_dot[frame]) - K_x * (q_x[frame - 1] - q0[frame]) + tau_ext[frame]) + q0_ddot[frame];

        Eigen::Vector2d q_x_dot_tem;
        q_x_dot_tem = q_x_hat[frame - 1] + dt * q_x_ddot;
        q_x_hat.push_back(q_x_dot_tem);

        Eigen::Vector2d q_x_tem;
        q_x_tem = q_x[frame - 1] + dt * q_x_hat[frame];
        q_x.push_back(q_x_tem);

        Eigen::Vector2d a_tem;
        a_tem = a[frame - 1] + dt * (q_x[frame] - q_s[frame]);
        a.push_back(a_tem);

        Eigen::Vector2d q_s_dot;
        q_s_dot = (q_s[frame] - q_s[frame - 1]) / dt;

        Eigen::Vector2d tau_star_tem;
        tau_star_tem = M * q_x_ddot + K * (q_x[frame] - q_s[frame]) + B * (q_x_hat[frame] - q_s_dot) + L * a[frame];
        tau_star.push_back(tau_star_tem);

        Eigen::Vector2d tau_tem;
        tau_tem = proj(tau_star_tem);
        tau.push_back(tau_tem);

        return tau_tem;


    }

    void refreshOnPositionControl(const double & T) override
    {
        frame++;
    }


};
