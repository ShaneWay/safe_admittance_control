#pragma once
#include "BaseController.h"
#include <iostream>
#include <numeric>

class KikuuweController : public BaseController {
public:
    KikuuweController(const Config& config) : BaseController(config)
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
        // get new u_x_star
        Eigen::Vector2d  u_x_star_tem = (M_x + B_x * dt).inverse() * (M_x * u_x[frame - 1] + dt * (M_x * q0_ddot[frame] + B_x * q0_dot[frame] + K_x * q0[frame] + tau_ext[frame])) ;
        u_x_star.push_back(u_x_star_tem);

        // get new q_x_star
        Eigen::Vector2d q_x_star_tem = q_x[frame - 1] + dt * u_x_star[frame];
        q_x_star.push_back(q_x_star_tem);

        // get new phi_b
        Eigen::Vector2d phi_b_tem;
        phi_b_tem = (B * (q_x[frame - 1] - q_s[frame - 1])) / dt - L * a[frame - 1];
        phi_b.push_back(phi_b_tem);

        // get new phi a
        Eigen::Vector2d phi_a_tem;
        phi_a_tem = M * (q_s[frame] - q_x[frame - 1] - T * u_x[frame - 1]) / (dt * dt);
        phi_a.push_back(phi_a_tem);

        // get new q_star
        Eigen::Vector2d q_s_star_tem;
        q_s_star_tem = q_s[frame] + (K_hat + M / (dt * dt)).inverse() * (phi_b[frame] - phi_a[frame]);
        q_s_star.push_back(q_s_star_tem);

        // get mat1 and mat2
        Eigen::Matrix2d  = Eigen::Matrix2d::Identity() + (M_x + B_x * dt).inverse() * K_x * (dt * dt);
        Eigen::Matrix2d = K_hat + M / (dt * dt);

        // get new tau_star
        Eigen::Vector2d tau_star_tem;
        tau_star_tem = Mat2 * Mat1.inverse() * q_x_star[frame] - Mat2 * q_s_star[frame];
        tau_star.push_back(tau_star_tem);

        // get new output tau
        Eigen::Vector2d tau_tem;
        tau_tem = proj(tau_star_tem);
        tau.push_back(tau_tem);

        return tau_tem;
    }

    void refreshOnPositionControl() override
    {
        Eigen::Matrix2d K_hat = K + B / dt + L * dt;

        // get new q_x
        Eigen::Vector2d q_x_tem;
        q_x_tem = q_s_star[frame] + (K_hat + M / (dt * dt)).inverse() * tau[frame];
        q_x.push_back(q_x_tem);

        //get new u_x
        Eigen::Vector2d u_x_tem;
        u_x_tem = (q_x[frame] - q_x[frame - 1]) / dt;
        u_x.push_back(u_x_tem);

        Eigen::Vector2d q_x_hat_tem = (q_x[frame] - q_x[frame - 1]) / dt;
        q_x_hat.push_back(q_x_hat_tem);

        // get new a
        Eigen::Vector2d a_tem;
        a_tem = a[frame - 1] + dt * (q_x[frame] - q_s[frame]);
        a.push_back(a_tem);

        frame++;
    }


};
