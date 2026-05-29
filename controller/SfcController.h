#pragma once

#include "BaseController.h"

#include <algorithm>
#include <iostream>

class SfcController : public BaseController
{
  public:
    explicit SfcController(const std::string& config_file, DataLogger& logger) : BaseController(config_file, logger) {}

    Eigen::Vector2d getTorqueOnForceControl(Eigen::Vector2d& f_ext_from_sensor, Eigen::Vector2d& q_from_sensor) override
    {
        Eigen::Vector2d f_d_joint = getDesiredForceJoint();

        const int n = 3;

        Eigen::Vector2d tem = q_x_hat_last;
        Eigen::Vector2d tem_abs = tem.array().abs();
        Eigen::Vector2d tem_abs_pow = tem_abs.array().pow(n - 1).matrix();

        a = M_x.inverse() * (tau_ext + f_d_joint - B_x_sfc * tem_abs_pow.cwiseProduct(tem));

        q_x_hat = q_x_hat_last + a * dt;
        q_x = q_x_last + dt * q_x_hat;

        integral_a = integral_a_last + dt * (q_x - q_s);

        tau_star = M * a + K * (q_x - q_s) + B * (q_x_hat - q_s_hat) + L * integral_a;

        tau = proj(tau_star);

        return tau;
    }

    Eigen::Vector2d getTorqueOnPositionControl(Eigen::Vector2d& f_ext_from_sensor, Eigen::Vector2d& q_from_sensor) override
    {
        const size_t idx = std::min<size_t>(static_cast<size_t>(frame), q0_traj.size() - 1);

        q0 = q0_traj[idx];
        q0_dot = q0_dot_traj[idx];
        q0_ddot = q0_ddot_traj[idx];

        X0 = X0_traj[idx];
        X0_dot = X0_dot_traj[idx];
        X0_ddot = X0_ddot_traj[idx];

        const int n = 5;

        Eigen::Vector2d q0_last = q0;
        Eigen::Vector2d q0_dot_last = q0_dot;

        if (idx > 0) {
            q0_last = q0_traj[idx - 1];
            q0_dot_last = q0_dot_traj[idx - 1];
        }

        Eigen::Vector2d tem = q_x_hat_last - q0_dot_last;
        Eigen::Vector2d tem_abs = tem.array().abs();
        Eigen::Vector2d tem_abs_pow = tem_abs.array().pow(n - 1).matrix();

        Eigen::Vector2d q_x_hat_hat =
            q0_ddot + M_x.inverse() * (tau_ext - B_x_sfc * tem_abs_pow.cwiseProduct(tem) - K_x * (q_x_last - q0_last));

        q_x_hat = q_x_hat_last + q_x_hat_hat * dt;
        q_x = q_x_last + q_x_hat * dt;

        a = a_last + (q_x - q_s) * dt;

        tau_star = M * q_x_hat_hat + K * (q_x - q_s) + B * (q_x_hat - q_s_hat) + L * a;

        tau = proj(tau_star);

        return tau;
    }

    void refreshOnForceControl() override
    {
        /*
         * 新结构中不在控制器 refresh 里 frame++。
         * BaseController::refresh() 会统一更新 *_last 并 frame++。
         */
    }

    void refreshOnPositionControl() override
    {
        /*
         * 新结构中不在控制器 refresh 里 frame++。
         */
    }
};