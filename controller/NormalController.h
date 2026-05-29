#pragma once

#include "BaseController.h"

#include <algorithm>
#include <iostream>

class NormalController : public BaseController
{
  public:
    explicit NormalController(const std::string& config_file, DataLogger& logger) : BaseController(config_file, logger) {}

    Eigen::Vector2d getTorqueOnForceControl(Eigen::Vector2d& f_ext_from_sensor, Eigen::Vector2d& q_from_sensor) override
    {
        Eigen::Vector2d f_d_joint = getDesiredForceJoint();

        Eigen::Matrix2d K_hat = K + B / dt + L * dt;
        Eigen::Matrix2d Mat = K_hat + M / (dt * dt);

        u_x_star = (M_x + B_x * dt).inverse() * (M_x * q_x_hat_last + dt * (tau_ext + f_d_joint));

        q_x = q_x_last + dt * u_x_star;

        phi_b = B * (q_x_last - q_s_last) / dt - L * a_last;

        phi_a = M * (q_s - q_x_last - dt * q_x_hat_last) / (dt * dt);

        q_s_star = q_s + Mat.inverse() * (phi_b - phi_a);

        tau_star = Mat * (q_x - q_s_star);

        tau = proj(tau_star);

        q_x_hat = (q_x - q_x_last) / dt;
        a = a_last + dt * (q_x - q_s);

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

        Eigen::Vector2d q_x_ddot = M_x.inverse() * (-B_x * (q_x_hat_last - q0_dot) - K_x * (q_x_last - q0) + tau_ext) + q0_ddot;

        q_x_hat = q_x_hat_last + dt * q_x_ddot;

        q_x = q_x_last + dt * q_x_hat;

        a = a_last + dt * (q_x - q_s);

        tau_star = M * q_x_ddot + K * (q_x - q_s) + B * (q_x_hat - q_s_hat) + L * a;

        tau = proj(tau_star);

        return tau;
    }

    void refreshOnForceControl() override {}

    void refreshOnPositionControl() override
    {
        /*
         * 原始 position refresh 只 frame++。
         * 新结构中 frame++ 由 BaseController::refresh() 统一完成。
         */
    }
};