#pragma once
#include "BaseController.h"
#include <iostream>
#include <numeric>

class SfcController : public BaseController {
public:
    SfcController(BaseParams& params, ControlState& state) : BaseController(params, state)
    {

    }

    Eigen::Vector2d getTorqueOnForceControl(Eigen::Vector2d & f_ext_from_sensor, Eigen::Vector2d& q_frome_sensor) override
    {
        f.push_back(f_ext_from_sensor);
        q_s.push_back(q_frome_sensor);
        q_s_hat.push_back((q_s[frame] - q_s[frame - 1])/dt);
        cout << "f[frame " << frame << "] = " << f[frame] << endl;
        cout << "q[frame " << frame << "] = " << q_s[frame] << endl;

        model.getJacobianMatrixTwoDOF(q_frome_sensor, jacobian_now);
        //update q and f_ext , get them from sensor
        tau_ext.push_back(jacobian_now.transpose() * f_ext_from_sensor);
        Eigen::Vector2d f_d_joint = jacobian_now.transpose() * f_d[frame - 1];

        int n = 3;
        Eigen::Vector2d tem = (q_x_hat[frame - 1] );
        Eigen::Vector2d tem_abs = (q_x_hat[frame - 1] ).array().abs();
        Eigen::Vector2d tem_abs_pow = tem_abs.array().pow(n-1).matrix();
    
        Eigen::Vector2d  a_tem = M_x.inverse() * (tau_ext[frame] + f_d_joint - B_x_sfc * tem_abs_pow.cwiseProduct(tem));
        a.push_back(a_tem);
        Eigen::Vector2d  q_x_hat_tem = q_x_hat[frame - 1] + a[frame] * dt;
        q_x_hat.push_back(q_x_hat_tem);

        Eigen::Vector2d  q_x_tem = q_x[frame - 1] + dt * q_x_hat[frame];
        q_x.push_back(q_x_tem);

        Eigen::Vector2d  integral_a_tem = integral_a[frame - 1] + dt * (q_x[frame] - q_s[frame]);
        integral_a.push_back(integral_a_tem);
        Eigen::Vector2d  tau_star_tem = M * a[frame] + K * (q_x[frame] - q_s[frame]) + B * (q_x_hat[frame] - q_s_hat[frame])  + L * integral_a[frame];
        tau_star.push_back(tau_star_tem);
    
        tau.push_back(proj(tau_star_tem));

        return tau[frame];
    }

    void refreshOnForceControl() override
    {
        frame++;
    }

    Eigen::Vector2d getTorqueOnPositionControl(Eigen::Vector2d & f_ext_from_sensor, Eigen::Vector2d& q_frome_sensor) override
    {
        f.push_back(f_ext_from_sensor);
        q_s.push_back(q_frome_sensor);
        q_s_hat.push_back((q_s[frame] - q_s[frame - 1]) / dt);
        cout << "f[frame " << frame << "] = " << f[frame].transpose() << endl;
        cout << "q[frame " << frame << "] = " << q_s[frame].transpose() << endl;
        model.getJacobianMatrixTwoDOF(q_frome_sensor, jacobian_now);
        //update q and f_ext , get them from sensor
        tau_ext.push_back(jacobian_now.transpose() * f_ext_from_sensor);

        int n = 5;
        Eigen::Vector2d tem = (q_x_hat[frame - 1] - q0_dot[frame - 1]);
        Eigen::Vector2d tem_abs = (q_x_hat[frame - 1] - q0_dot[frame - 1]).array().abs();
        Eigen::Vector2d tem_abs_pow = tem_abs.array().pow(n-1).matrix();

        // Eigen::Vector2d q_x_hat_hat = q0_ddot[frame] + M_x.inverse() * (tau_ext[frame] - B_x_sfc * tem )* (q_x_hat[frame - 1] - q0_dot[frame - 1]) + K_x * (q_x[frame - 1] - q0[frame - 1]);
        Eigen::Vector2d q_x_hat_hat = q0_ddot[frame] + M_x.inverse() *(tau_ext[frame]-B_x_sfc * tem_abs_pow.cwiseProduct(tem) - K_x * (q_x[frame - 1] - q0[frame - 1]) );
        q_x_hat.push_back(q_x_hat[frame - 1] + q_x_hat_hat * dt);
        q_x.push_back(q_x[frame - 1] + q_x_hat[frame] * dt);

        a.push_back(a[frame - 1] + (q_x[frame] - q_s[frame] )* dt);
        tau_star.push_back(M * q_x_hat_hat + K * (q_x[frame] - q_s[frame]) + B * (q_x_hat[frame] - q_s_hat[frame]) + L * a[frame]);
        
        tau.push_back(proj(tau_star[frame]));

        return tau[frame];
    }

    void refreshOnPositionControl() override
    {
        frame++;
    }

};
