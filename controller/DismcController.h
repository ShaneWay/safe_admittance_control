#pragma once
#include "BaseController.h"
#include <iostream>
#include <numeric>

class DismcController : public BaseController {
private:
    Eigen::Matrix2d M_s      = 2.5  * Eigen::Matrix2d::Identity();
    Eigen::Matrix2d D_s      = 5.0  * Eigen::Matrix2d::Identity();
    Eigen::Matrix2d Lamda_1  = 20.0 * Eigen::Matrix2d::Identity();
    Eigen::Matrix2d Lamda_2  = 40.0 * Eigen::Matrix2d::Identity();

    Eigen::Vector2d F0 = 4.52 * Eigen::Vector2d::Identity();

    double h = dt;

public:
    DismcController(BaseParams& params, ControlState& state) : BaseController(params, state)
    {

    }
    Eigen::Vector2d getTorqueOnForceControl(Eigen::Vector2d & f_ext_from_sensor, Eigen::Vector2d& q_frome_sensor) override
    {
        f.push_back(f_ext_from_sensor);
        q_s.push_back(q_frome_sensor);
        q_s_hat.push_back((q_s[frame] - q_s[frame - 1])/dt);
        cout << "f[frame " << frame << "] = " << f[frame].transpose() << endl;
        cout << "q[frame " << frame << "] = " << q_s[frame].transpose() << endl;
        model.getJacobianMatrixTwoDOF(q_frome_sensor, jacobian_now);
        //update q and f_ext , get them from sensor
        tau_ext.push_back(jacobian_now.transpose() * f_ext_from_sensor);
        Eigen::Vector2d f_d_joint = jacobian_now.transpose() * f_d[frame - 1];

        Eigen::Matrix2d h_x;
        h_x = (M_x + h * B_x).inverse() * h;

        Eigen::Matrix2d h_s;
        h_s = (M_s + h * D_s).inverse() * h;

        Eigen::Matrix2d lamda_1;
        lamda_1 = (Eigen::Matrix2d::Identity() + h * Lamda_1).inverse();

        Eigen::Matrix2d lamda_2;
        lamda_2 = (Eigen::Matrix2d::Identity() + h * Lamda_2).inverse();

        Eigen::Vector2d set;
        set = -tau_ext[frame] - f_d_joint;

        Eigen::Vector2d q1;
        q1 = lamda_1 * (q_x[frame - 1] + h* Lamda_1 * q_s[frame] + h_s * M_s * e_r[frame - 1]);

        Eigen::Vector2d q2;
        q2 = q_x[frame - 1] + h_x * (M_x * q_x_hat[frame - 1] - h * set);

        Eigen::Vector2d q3;
        q3 = q_s[frame] + lamda_2 * e_q[frame - 1];

        Eigen::Matrix2d S1;
        S1.block(0,0,2,1) = (lamda_1 * h_s * h * (F_max - F0));
        S1.block(0,1,2,1) = (lamda_1 * h_s * h * (F_max + F0));

        Eigen::Matrix2d _S1;
        _S1.block(0,0,2,1) = - (lamda_1 * h_s * h * (F_max + F0));
        _S1.block(0,1,2,1) = - (lamda_1 * h_s * h * (F_max - F0));

        Eigen::Matrix2d S0;
        S0.block(0,0,2,1) = project(_S1, q3 - q1);
        S0.block(0,1,2,1) = project(S1, q3 - q1);

        q_x.push_back(q1 + project(S0, q2 - q1));

        Eigen::Vector2d q4;
        if (frame == 1)
        {
            q4 = q_s[frame - 1] + h_s * h * tau_ext[frame];
        }
        else
        {
            q4 = q_s[frame - 1] + h_s * M_s * (q_s[frame - 1] - q_s[frame -2]) / h + h_s * h * tau_ext[frame];
        }

        Eigen::Vector2d q5;
        q5 = (h_s * h).inverse() * (q_x[frame] - lamda_2 * e_q[frame - 1] - q4);

        Eigen::Vector2d q6;
        q6 = (lamda_1 * h_s * h).inverse() * (q_x[frame] - q1);

        Eigen::Matrix2d F_h;
        F_h.block(0,0,2,1) = -F0;
        F_h.block(0,1,2,1) = F0;

        Eigen::Vector2d tau_tem;
        tau_tem = q6 + project(F_h, q5 - q6);
        tau.push_back(tau_tem);

        return tau[frame];
    }

    void refreshOnForceControl() override
    {
        Eigen::Vector2d q_x_hat_tem;
        q_x_hat_tem = (q_x[frame] - q_x[frame - 1]) / dt;
        q_x_hat.push_back(q_x_hat_tem);

        Eigen::Vector2d e_q_tem;
        e_q_tem = q_x[frame] - q_s[frame];
        e_q.push_back(e_q_tem);

        Eigen::Vector2d e_r_tem;
        e_r_tem = q_x_hat[frame] + Lamda_1 * e_q[frame];
        e_r.push_back(e_r_tem);

        frame += 1;
    }


    Eigen::Vector2d getTorqueOnPositionControl(Eigen::Vector2d & f_ext_from_sensor, Eigen::Vector2d& q_frome_sensor) override
    {
        f.push_back(f_ext_from_sensor);
        q_s.push_back(q_frome_sensor);
        q_s_hat.push_back((q_s[frame] - q_s[frame - 1])/dt);
        cout << "f[frame " << frame << "] = " << f[frame].transpose() << endl;
        cout << "q[frame " << frame << "] = " << q_s[frame].transpose() << endl;
        model.getJacobianMatrixTwoDOF(q_frome_sensor, jacobian_now);
        //update q and f_ext , get them from sensor
        tau_ext.push_back(jacobian_now.transpose() * f_ext_from_sensor);
        

        Eigen::Matrix2d h_x;
        h_x = (M_x + h * B_x).inverse() * h;

        Eigen::Matrix2d h_s;
        h_s = (M_s + h * D_s).inverse() * h;

        Eigen::Matrix2d lamda_1;
        lamda_1 = (Eigen::Matrix2d::Identity() + h * Lamda_1).inverse();

        Eigen::Matrix2d lamda_2;
        lamda_2 = (Eigen::Matrix2d::Identity() + h * Lamda_2).inverse();

        Eigen::Vector2d set;
        set = -tau_ext[frame] - (M_x * q0_ddot[frame] + B_x * q0_dot[frame] + K_x * q0[frame]);

        Eigen::Matrix2d h_x_star;
        h_x_star = (Eigen::Matrix2d::Identity() + h * h_x * K_x).inverse();

        Eigen::Vector2d q1;
        q1 = lamda_1 * (q_x[frame - 1] + h* Lamda_1 * q_s[frame] + h_s * M_s * e_r[frame - 1]);

        Eigen::Vector2d q2;
        q2 = h_x_star * q_x[frame - 1] + h_x_star * h_x * (M_x * q_x_hat[frame - 1] - h * set);

        Eigen::Vector2d q3;
        q3 = q_s[frame] + lamda_2 * e_q[frame - 1];

        Eigen::Matrix2d S1;
        S1.block(0,0,2,1) = (lamda_1 * h_s * h * (F_max - F0));
        S1.block(0,1,2,1) = (lamda_1 * h_s * h * (F_max + F0));

        Eigen::Matrix2d _S1;
        _S1.block(0,0,2,1) = - (lamda_1 * h_s * h * (F_max + F0));
        _S1.block(0,1,2,1) = - (lamda_1 * h_s * h * (F_max - F0));

        Eigen::Matrix2d S0;
        S0.block(0,0,2,1) = project(_S1, q3 - q1);
        S0.block(0,1,2,1) = project(S1, q3 - q1);

        q_x.push_back(q1 + project(S0, q2 - q1));

        Eigen::Vector2d q4;
        if (frame == 1)
        {
            q4 = q_s[frame - 1] + h_s * h * tau_ext[frame];
        }
        else
        {
            q4 = q_s[frame - 1] + h_s * M_s * (q_s[frame - 1] - q_s[frame -2]) / h + h_s * h * tau_ext[frame];
        }

        Eigen::Vector2d q5;
        q5 = (h_s * h).inverse() * (q_x[frame] - lamda_2 * e_q[frame - 1] - q4);

        Eigen::Vector2d q6;
        q6 = (lamda_1 * h_s * h).inverse() * (q_x[frame] - q1);

        Eigen::Matrix2d F_h;
        F_h.block(0,0,2,1) = -F0;
        F_h.block(0,1,2,1) = F0;

        Eigen::Vector2d tau_tem;
        tau_tem = q6 + project(F_h, q5 - q6);
        tau.push_back(tau_tem);

        return tau[frame];

    }

    void refreshOnPositionControl() override
    {
        Eigen::Vector2d q_x_hat_tem;
        q_x_hat_tem = (q_x[frame] - q_x[frame - 1]) / dt;
        q_x_hat.push_back(q_x_hat_tem);

        Eigen::Vector2d e_q_tem;
        e_q_tem = q_x[frame] - q_s[frame];
        e_q.push_back(e_q_tem);

        Eigen::Vector2d e_r_tem;
        e_r_tem = q_x_hat[frame] + Lamda_1 * e_q[frame];
        e_r.push_back(e_r_tem);

        frame += 1;

    }

    Eigen::Vector2d project(Eigen::Matrix2d S, Eigen::Vector2d q)
    {
        Eigen::Vector2d q_output;
        for (int i = 0; i < 2; i ++)
        {
            if (q[i] < S(i,0))
            {
                q_output[i] = S(i,0);
            }
            else if(q[i] > S(i,1))
            {
                q_output[i] = S(i,1);
            }else
            {
                q_output[i] = q[i];
            }
        }
        return q_output;
    }
};
