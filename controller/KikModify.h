#pragma once

#include "BaseController.h"

#include <algorithm>
#include <iostream>

class KikModify : public BaseController
{
  public:
    explicit KikModify(const std::string& config_file, DataLogger& logger) : BaseController(config_file, logger) {}

    Eigen::Vector2d getTorqueOnForceControl(Eigen::Vector2d& f_ext_from_sensor, Eigen::Vector2d& q_from_sensor) override
    {
        Eigen::Vector2d f_d_joint = getDesiredForceJoint();

        Eigen::Matrix2d K_hat = K + B / dt + L * dt;

        u_x_star = (M_x + B_x * dt).inverse() * (M_x * q_x_hat_last + dt * (tau_ext + f_d_joint));

        q_x_star = q_x_last + dt * u_x_star;

        phi_b = B * (q_x_last - q_s_last) / dt - L * a_last;

        Eigen::Vector2d phi_a_tem = M * (u_x_star - q_x_hat_last) / dt;
        phi_a = proj(phi_a_tem);

        q_s_star = q_s + K_hat.inverse() * (phi_b - phi_a);

        tau_star = K_hat * (q_x_star - q_s_star);

        tau = proj(tau_star);

        q_x = q_s_star + K_hat.inverse() * tau;
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

        Eigen::Matrix2d K_hat = K + B / dt + L * dt;

        u_x_star = (M_x + B_x * dt).inverse() * (M_x * q_x_hat_last + dt * (M_x * q0_ddot + B_x * q0_dot + K_x * q0 + tau_ext));

        q_x_star = q_x_last + dt * u_x_star;

        phi_b = B * (q_x_last - q_s_last) / dt - L * a_last;

        Eigen::Vector2d phi_a_tem = M * (u_x_star - q_x_hat_last) / dt;
        phi_a = proj(phi_a_tem);

        q_s_star = q_s + K_hat.inverse() * (phi_b - phi_a);

        tau_star = K_hat * (q_x_star - q_s_star);

        tau = proj(tau_star);

        q_x = q_s_star + K_hat.inverse() * tau;
        q_x_hat = (q_x - q_x_last) / dt;
        a = a_last + dt * (q_x - q_s);

        return tau;
    }

    void refreshOnForceControl() override
    {
        /*
         * 当前帧 q_x、q_x_hat、u_x、a 已在 getTorqueOnForceControl() 中更新。
         * BaseController::refresh() 统一更新 *_last 和 frame。
         */
    }

    void refreshOnPositionControl() override
    {
        /*
         * 当前帧 q_x、u_x、q_x_hat、a 已在 getTorqueOnPositionControl() 中更新。
         */
    }
};