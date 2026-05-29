#pragma once

#include "BaseController.h"

#include <algorithm>
#include <iostream>

class DismcController : public BaseController
{
  private:
    Eigen::Matrix2d M_s = 2.5 * Eigen::Matrix2d::Identity();
    Eigen::Matrix2d D_s = 5.0 * Eigen::Matrix2d::Identity();
    Eigen::Matrix2d Lamda_1 = 20.0 * Eigen::Matrix2d::Identity();
    Eigen::Matrix2d Lamda_2 = 40.0 * Eigen::Matrix2d::Identity();

    Eigen::Vector2d F0 = 4.52 * Eigen::Vector2d::Ones();

  public:
    explicit DismcController(const std::string& config_file, DataLogger& logger) : BaseController(config_file, logger) {}

    Eigen::Vector2d getTorqueOnForceControl(Eigen::Vector2d& f_ext_from_sensor, Eigen::Vector2d& q_from_sensor) override
    {
        const double h = dt;

        Eigen::Vector2d f_d_joint = getDesiredForceJoint();

        Eigen::Matrix2d h_x = (M_x + h * B_x).inverse() * h;
        Eigen::Matrix2d h_s = (M_s + h * D_s).inverse() * h;

        Eigen::Matrix2d lamda_1 = (Eigen::Matrix2d::Identity() + h * Lamda_1).inverse();
        Eigen::Matrix2d lamda_2 = (Eigen::Matrix2d::Identity() + h * Lamda_2).inverse();

        /*
         * 原文件 force control:
         * set = -tau_ext[frame] - f_d_joint
         */
        Eigen::Vector2d set = -tau_ext - f_d_joint;

        /*
         * 原文件:
         * q1 = lamda_1 * (q_x[frame-1] + h*Lamda_1*q_s[frame] + h_s*M_s*e_r[frame-1])
         */
        Eigen::Vector2d q1 = lamda_1 * (q_x_last + h * Lamda_1 * q_s + h_s * M_s * e_r_last);

        /*
         * 原文件:
         * q2 = q_x[frame-1] + h_x * (M_x*q_x_hat[frame-1] - h*set)
         */
        Eigen::Vector2d q2 = q_x_last + h_x * (M_x * q_x_hat_last - h * set);

        /*
         * 原文件:
         * q3 = q_s[frame] + lamda_2 * e_q[frame-1]
         */
        Eigen::Vector2d q3 = q_s + lamda_2 * e_q_last;

        Eigen::Matrix2d S1;
        S1.block(0, 0, 2, 1) = lamda_1 * h_s * h * (F_max - F0);
        S1.block(0, 1, 2, 1) = lamda_1 * h_s * h * (F_max + F0);

        Eigen::Matrix2d _S1;
        _S1.block(0, 0, 2, 1) = -(lamda_1 * h_s * h * (F_max + F0));
        _S1.block(0, 1, 2, 1) = -(lamda_1 * h_s * h * (F_max - F0));

        Eigen::Matrix2d S0;
        S0.block(0, 0, 2, 1) = projectInterval(_S1, q3 - q1);
        S0.block(0, 1, 2, 1) = projectInterval(S1, q3 - q1);

        /*
         * 原文件:
         * q_x[frame] = q1 + project(S0, q2 - q1)
         */
        q_x = q1 + projectInterval(S0, q2 - q1);

        /*
         * 原文件:
         * if frame == 1:
         *     q4 = q_s[frame-1] + h_s*h*tau_ext[frame]
         * else:
         *     q4 = q_s[frame-1] + h_s*M_s*(q_s[frame-1]-q_s[frame-2])/h + h_s*h*tau_ext[frame]
         *
         * 新结构:
         * q_s[frame-1] -> q_s_last
         * (q_s[frame-1]-q_s[frame-2])/h -> q_s_hat_last
         */
        Eigen::Vector2d q4;
        if (frame == 1) {
            q4 = q_s_last + h_s * h * tau_ext;
        } else {
            q4 = q_s_last + h_s * M_s * q_s_hat_last + h_s * h * tau_ext;
        }

        Eigen::Vector2d q5 = (h_s * h).inverse() * (q_x - lamda_2 * e_q_last - q4);
        Eigen::Vector2d q6 = (lamda_1 * h_s * h).inverse() * (q_x - q1);

        Eigen::Matrix2d F_h;
        F_h.block(0, 0, 2, 1) = -F0;
        F_h.block(0, 1, 2, 1) = F0;

        tau = q6 + projectInterval(F_h, q5 - q6);

        /*
         * 原文件 refresh 中做：
         * q_x_hat = (q_x[frame] - q_x[frame-1]) / dt
         * e_q = q_x[frame] - q_s[frame]
         * e_r = q_x_hat[frame] + Lamda_1 * e_q[frame]
         *
         * 新结构中为了 DataLogger 能记录当前帧，直接在这里更新。
         */
        q_x_hat = (q_x - q_x_last) / dt;
        e_q = q_x - q_s;
        e_r = q_x_hat + Lamda_1 * e_q;

        /*
         * DISMC 原文件没有 tau_star，这里为了绘图和记录一致，令 tau_star = tau。
         */
        tau_star = tau;

        return tau;
    }

    Eigen::Vector2d getTorqueOnPositionControl(Eigen::Vector2d& f_ext_from_sensor, Eigen::Vector2d& q_from_sensor) override
    {
        const double h = dt;

        const size_t idx = std::min<size_t>(static_cast<size_t>(frame), q0_traj.size() - 1);

        q0 = q0_traj[idx];
        q0_dot = q0_dot_traj[idx];
        q0_ddot = q0_ddot_traj[idx];

        X0 = X0_traj[idx];
        X0_dot = X0_dot_traj[idx];
        X0_ddot = X0_ddot_traj[idx];

        Eigen::Matrix2d h_x = (M_x + h * B_x).inverse() * h;
        Eigen::Matrix2d h_s = (M_s + h * D_s).inverse() * h;

        Eigen::Matrix2d lamda_1 = (Eigen::Matrix2d::Identity() + h * Lamda_1).inverse();
        Eigen::Matrix2d lamda_2 = (Eigen::Matrix2d::Identity() + h * Lamda_2).inverse();

        /*
         * 原文件 position control:
         * set = -tau_ext[frame] - (M_x*q0_ddot[frame] + B_x*q0_dot[frame] + K_x*q0[frame])
         */
        Eigen::Vector2d set = -tau_ext - (M_x * q0_ddot + B_x * q0_dot + K_x * q0);

        /*
         * 原文件:
         * h_x_star = (I + h*h_x*K_x)^-1
         */
        Eigen::Matrix2d h_x_star = (Eigen::Matrix2d::Identity() + h * h_x * K_x).inverse();

        Eigen::Vector2d q1 = lamda_1 * (q_x_last + h * Lamda_1 * q_s + h_s * M_s * e_r_last);

        /*
         * 原文件 position control:
         * q2 = h_x_star*q_x[frame-1] + h_x_star*h_x*(M_x*q_x_hat[frame-1] - h*set)
         */
        Eigen::Vector2d q2 = h_x_star * q_x_last + h_x_star * h_x * (M_x * q_x_hat_last - h * set);

        Eigen::Vector2d q3 = q_s + lamda_2 * e_q_last;

        Eigen::Matrix2d S1;
        S1.block(0, 0, 2, 1) = lamda_1 * h_s * h * (F_max - F0);
        S1.block(0, 1, 2, 1) = lamda_1 * h_s * h * (F_max + F0);

        Eigen::Matrix2d _S1;
        _S1.block(0, 0, 2, 1) = -(lamda_1 * h_s * h * (F_max + F0));
        _S1.block(0, 1, 2, 1) = -(lamda_1 * h_s * h * (F_max - F0));

        Eigen::Matrix2d S0;
        S0.block(0, 0, 2, 1) = projectInterval(_S1, q3 - q1);
        S0.block(0, 1, 2, 1) = projectInterval(S1, q3 - q1);

        q_x = q1 + projectInterval(S0, q2 - q1);

        Eigen::Vector2d q4;
        q4 = q_s_last + h_s * M_s * q_s_hat_last + h_s * h * tau_ext;

        Eigen::Vector2d q5 = (h_s * h).inverse() * (q_x - lamda_2 * e_q_last - q4);
        Eigen::Vector2d q6 = (lamda_1 * h_s * h).inverse() * (q_x - q1);

        Eigen::Matrix2d F_h;
        F_h.block(0, 0, 2, 1) = -F0;
        F_h.block(0, 1, 2, 1) = F0;

        tau = q6 + projectInterval(F_h, q5 - q6);

        q_x_hat = (q_x - q_x_last) / dt;
        e_q = q_x - q_s;
        e_r = q_x_hat + Lamda_1 * e_q;

        tau_star = tau;

        return tau;
    }

    void refreshOnForceControl() override
    {
        /*
         * q_x_hat、e_q、e_r 已在 getTorqueOnForceControl() 中更新。
         * BaseController::refresh() 统一更新 *_last 和 frame。
         */
    }

    void refreshOnPositionControl() override
    {
        /*
         * q_x_hat、e_q、e_r 已在 getTorqueOnPositionControl() 中更新。
         */
    }

  private:
    Eigen::Vector2d projectInterval(const Eigen::Matrix2d& S, const Eigen::Vector2d& q) const
    {
        Eigen::Vector2d q_output;

        for (int i = 0; i < 2; ++i) {
            if (q[i] < S(i, 0)) {
                q_output[i] = S(i, 0);
            } else if (q[i] > S(i, 1)) {
                q_output[i] = S(i, 1);
            } else {
                q_output[i] = q[i];
            }
        }

        return q_output;
    }
};