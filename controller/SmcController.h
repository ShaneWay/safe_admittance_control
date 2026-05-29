#pragma once

#include "BaseController.h"

#include <algorithm>
#include <iostream>

class SmcController : public BaseController
{
  public:
    explicit SmcController(const std::string& config_file, DataLogger& logger) : BaseController(config_file, logger) {}

    Eigen::Vector2d getTorqueOnForceControl(Eigen::Vector2d& f_ext_from_sensor, Eigen::Vector2d& q_from_sensor) override
    {
        Eigen::Vector2d f_d_joint = getDesiredForceJoint();

        Eigen::Matrix2d K_hat = K + B / dt + L * dt;
        Eigen::Matrix2d Mat = K_hat + M / (dt * dt);

        u_x_star = (M_x + B_x * dt).inverse() * (M_x * q_x_hat_last + dt * (tau_ext + f_d_joint));

        updateQmaxByForceForForceControl();

        Eigen::Vector2d q_x_star_hat = proj_Q(u_x_star);

        Eigen::Vector2d lamda = tau_ext + f_d_joint - M_x * (q_x_star_hat - q_x_hat_last) / dt - B_x * q_x_star_hat;

        q_x_star = q_x_last + dt * u_x_star;

        phi_b = B * (q_x_last - q_s_last) / dt - L * a_last;

        phi_a = M * (q_s - q_x_last - dt * q_x_hat_last) / (dt * dt);

        q_s_star = q_s + Mat.inverse() * (phi_b - phi_a);

        tau_star = Mat * ((q_x_star - q_s_star) - (M_x + dt * B_x).inverse() * dt * dt * lamda);

        tau = proj(tau_star);

        q_x = q_s_star + Mat.inverse() * tau;
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

        Eigen::Matrix2d h1 = Eigen::Matrix2d::Identity() + (M_x + B_x * dt).inverse() * dt * dt * K_x;

        Eigen::Matrix2d h2 = K_hat + M / (dt * dt);

        updateQmaxByForceForPositionControl();

        Eigen::Vector2d q_x_lim = q_x_last + dt * proj_Q((h1.inverse() * q_x_star - q_x_last) / dt);

        Eigen::Vector2d lamda = (M_x + B_x * dt) * (1.0 / (dt * dt)) * (q_x_star - h1 * q_x_lim);

        phi_b = B * (q_x_last - q_s_last) / dt - L * a_last;

        phi_a = M * (q_s - q_x_last - dt * q_x_hat_last) / (dt * dt);

        q_s_star = q_s + h2.inverse() * (phi_b - phi_a);

        tau_star = h2 * h1.inverse() * q_x_star - h2 * q_s_star - h2 * h1.inverse() * (M_x + B_x * dt).inverse() * dt * dt * lamda;

        tau = proj(tau_star);

        q_x = q_s_star + h2.inverse() * tau;
        q_x_hat = (q_x - q_x_last) / dt;
        a = a_last + dt * (q_x - q_s);

        return tau;
    }

    void refreshOnForceControl() override
    {
        /*
         * SMC 的 q_x、q_x_hat、a 已经在 getTorqueOnForceControl() 里更新。
         * BaseController::refresh() 会统一把当前帧变量保存到 *_last。
         */
    }

    void refreshOnPositionControl() override
    {
        /*
         * SMC 的 q_x、u_x、q_x_hat、a 已经在 getTorqueOnPositionControl() 里更新。
         */
    }

  private:
    void updateQmaxByForceForForceControl()
    {
        if (frame <= 2000) {
            Q_max = (f[0] > 4.0 || f[1] > 4.0) ? Eigen::Vector2d(0.01, 0.01) : Eigen::Vector2d(0.5, 0.5);
            return;
        }

        if (frame <= 4000) {
            Q_max = (f[0] > 4.0 || f[1] > 4.0) ? Eigen::Vector2d(0.0001, 0.0001) : Eigen::Vector2d(0.5, 0.5);
            return;
        }

        if (frame <= 8000) {
            Q_max = (f[0] > 8.0 || f[1] > 8.0) ? Eigen::Vector2d(0.0001, 0.0001) : Eigen::Vector2d(0.5, 0.5);
            return;
        }

        if (frame <= 12000) {
            Q_max = (f[0] > 12.0 || f[1] > 12.0) ? Eigen::Vector2d(0.0001, 0.0001) : Eigen::Vector2d(0.5, 0.5);
            return;
        }

        Q_max = Eigen::Vector2d(0.5, 0.5);
    }

    void updateQmaxByForceForPositionControl()
    {
        if (f[0] > 6.0 || f[1] > 6.0) {
            Q_max = Eigen::Vector2d(0.001, 0.001);
        } else {
            Q_max = Eigen::Vector2d(1.0, 1.0);
        }
    }
};