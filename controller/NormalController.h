#pragma once
#include "BaseController.h"
#include <iostream>
#include <numeric>

class NormalController : public BaseController {
public:
    NormalController(const ConfigLoader& loader) : BaseController(loader)
    {
        
    }

    Eigen::Vector2d getTorqueOnForceControl(Eigen::Vector2d & f_ext_from_sensor, Eigen::Vector2d& q_frome_sensor) override
    {
        f.push_back(f_ext_from_sensor);
        q_s.push_back(q_frome_sensor);
        cout << "f[frame " << frame << "] = " << f[frame].transpose() << endl;
        cout << "q[frame " << frame << "] = " << q_s[frame].transpose() << endl;
        q_s_hat.push_back((q_s[frame] - q_s[frame - 1])/dt);
        model.getJacobianMatrixTwoDOF(q_frome_sensor, jacobian_now);
        //update q and f_ext , get them from sensor
        tau_ext.push_back(jacobian_now.transpose() * f_ext_from_sensor);
        Eigen::Vector2d f_d_joint = jacobian_now.transpose() * f_d[frame - 1];

        Eigen::Matrix2d K_hat = K + B / dt + L * dt;
        Eigen::Vector2d u_x_star_tem = (M_x + B_x * dt).inverse() * ((M_x * q_x_hat[frame - 1]) + dt * (tau_ext[frame] + f_d_joint));
        u_x_star.push_back(u_x_star_tem);

        Eigen::Vector2d q_x_tem = q_x[frame - 1] + dt * u_x_star[frame];
        q_x.push_back(q_x_tem);

        Eigen::Vector2d phi_b_tem = (B * (q_x[frame - 1] - q_s[frame - 1])) / dt - L * a[frame - 1];
        phi_b.push_back(phi_b_tem);

        Eigen::Vector2d phi_a_tem = M * (q_s[frame] - q_x[frame - 1] - dt * q_x_hat[frame - 1]) / (dt * dt);
        phi_a.push_back(phi_a_tem);

        Eigen::Vector2d q_s_star_tem = q_s[frame] + (K_hat + M / (dt * dt)).inverse() * (phi_b[frame] - phi_a[frame]);

        Eigen::Matrix2d Mat = K_hat + M / (dt * dt);
        Eigen::Vector2d tau_tem = Mat * (q_x[frame] - q_s_star_tem);
        tau.push_back(proj(tau_tem));

        return tau[frame];
    }

    void refreshOnForceControl() override
    {
        Eigen::Matrix2d K_hat = K + B / dt + L * dt;
        Eigen::Vector2d q_x_hat_tem = (q_x[frame] - q_x[frame - 1]) / dt;
        q_x_hat.push_back(q_x_hat_tem);

        Eigen::Vector2d a_tem = a[frame - 1]  + dt * (q_x[frame] - q_s[frame]);
        a.push_back(a_tem);

        frame++;
    }

    Eigen::Vector2d getTorqueOnPositionControl(Eigen::Vector2d & f_ext_from_sensor, Eigen::Vector2d& q_frome_sensor) override
    {
        f.push_back(f_ext_from_sensor);
        q_s.push_back(q_frome_sensor);
        cout << "f[frame " << frame << "] = " << f[frame].transpose() << endl;
        cout << "q[frame " << frame << "] = " << q_s[frame].transpose() << endl;
        q_s_hat.push_back((q_s[frame] - q_s[frame - 1])/dt);
        model.getJacobianMatrixTwoDOF(q_frome_sensor, jacobian_now);
        //update q and f_ext , get them from sensor
        tau_ext.push_back(jacobian_now.transpose() * f_ext_from_sensor);
        
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

        Eigen::Vector2d q_s_hat_tem;
        q_s_hat_tem = (q_s[frame] - q_s[frame - 1]) / dt;
        q_s_hat.push_back(q_s_hat_tem);

        Eigen::Vector2d tau_star_tem;
        tau_star_tem = M * q_x_ddot + K * (q_x[frame] - q_s[frame]) + B * (q_x_hat[frame] - q_s_hat[frame]) + L * a[frame];
        cout << tau_star_tem << endl;
        tau_star.push_back(tau_star_tem);

        Eigen::Vector2d tau_tem;
        tau_tem = proj(tau_star_tem);
        tau.push_back(tau_tem);

        return tau_tem;


    }

    void refreshOnPositionControl() override
    {
        frame++;
    }


};
