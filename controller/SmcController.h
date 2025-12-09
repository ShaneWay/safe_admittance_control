#pragma once
#include "BaseController.h"
#include <iostream>
#include <numeric>

class SmcController : public BaseController {
private:

public:
    SmcController(const ConfigLoader& loader) : BaseController(loader)
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

        Eigen::Matrix2d K_hat = K + B / dt + L * dt;

        Eigen::Vector2d u_x_star_tem = (M_x + B_x * dt).inverse() * ((M_x * q_x_hat[frame - 1]) + dt * (tau_ext[frame] + f_d_joint));
        u_x_star.push_back(u_x_star_tem);
        // std::cout << "f_d: " << f_d[frame] << std::endl;

        if(f[frame][0] > 5 || f[frame][1] > 5)
        {
            Q_max = Eigen::Vector2d(0.01, 0.01);
        }
        else
        {
            Q_max = Eigen::Vector2d(0.6, 0.6);
        }


        Eigen::Vector2d q_x_star_hat = proj_Q(u_x_star_tem);

        Eigen::Vector2d lamda = tau_ext[frame] + f_d_joint - M_x * (q_x_star_hat - q_x_hat[frame - 1])/ dt - B_x * q_x_star_hat;
        // std::cout << "lamda: " << lamda << std::endl;

        Eigen::Vector2d q_x_star_tem = q_x[frame - 1] + dt * u_x_star[frame];
        // std::cout << "q_x_star: " << q_x_star_tem << std::endl;
        q_x_star.push_back(q_x_star_tem);

        Eigen::Vector2d phi_b_tem = (B * (q_x[frame - 1] - q_s[frame - 1])) / dt - L * a[frame - 1];
        phi_b.push_back(phi_b_tem);
        // std::cout << "phi_b_tem: " << phi_b_tem << std::endl;


        Eigen::Vector2d phi_a_tem = M * (q_s[frame] - q_x[frame - 1] - dt * q_x_hat[frame - 1]) / (dt * dt);
        phi_a.push_back(phi_a_tem);
        // std::cout << "phi_a_tem: " << phi_a_tem << std::endl;


        Eigen::Vector2d q_s_star_tem = q_s[frame] + (K_hat + M / (dt * dt)).inverse() * (phi_b[frame] - phi_a[frame]);
        q_s_star.push_back(q_s_star_tem);
        // std::cout << "q_s_star: " << q_s_star_tem << std::endl;


        Eigen::Matrix2d Mat = K_hat + M / (dt * dt);

        Eigen::Vector2d tao_star_tem = Mat * ((q_x_star[frame] - q_s_star[frame]) - (M_x + dt * B_x).inverse() * dt * dt * lamda);
    
        tau_star.push_back(tao_star_tem);

        Eigen::Vector2d tau_tem = proj(tau_star[frame]);
        tau.push_back(tau_tem);

        return tau[frame];

    }

    void refreshOnForceControl() override
    {
        Eigen::Matrix2d K_hat = K + B / dt + L * dt;
        Eigen::Vector2d q_x_tem = q_s_star[frame] + (K_hat + M / (dt * dt)).inverse() * tau[frame];
        q_x.push_back(q_x_tem);

        Eigen::Vector2d q_x_hat_tem = (q_x[frame] - q_x[frame - 1]) / dt;
        q_x_hat.push_back(q_x_hat_tem);
    
        Eigen::Vector2d a_tem = a[frame - 1]  + dt * (q_x[frame] - q_s[frame]);
        a.push_back(a_tem);

        // adaptive Q_max
        // Eigen::Vector2d vdot_abs = (q_x_star[frame] - q_x[frame] ).array().abs();
        // Eigen::Vector2d Q_max_term = Q_max - 180000* vdot_abs * dt;
        // for (int i = 0; i < 2; i++)
        // {
        //     if (Q_max_term[i] > 0.01)
        //     {
        //         Q_max[i] =  Q_max_term[i];
        //     }
        //     else
        //     {
        //         Q_max[i] = Q_max[i];
        //     }
        // }
        // std::cout << "Q_max: " << Q_max << std::endl; 


        frame++;
    }

    Eigen::Vector2d proj_Q(Eigen::Vector2d u_x_star)
    {   
        Eigen::Vector2d u_x_star_projected;
        for (int i = 0; i < 2; i++)
        {
            if (u_x_star[i] < -Q_max[i])
            {
                u_x_star_projected[i] =  -Q_max[i];
            }
            else if (u_x_star[i] > Q_max[i])
            {
                u_x_star_projected[i] = Q_max[i];
            }
            else
            {
                u_x_star_projected[i] = u_x_star[i];
            }
        }
        return u_x_star_projected;
    }

    // double getTorqueOnPositionControl(double & f_ext_from_sensor, double& q_frome_sensor) override
    // {
    //     f.push_back(f_ext_from_sensor);
    //     q_s.push_back(q_frome_sensor);
    //     q_s_hat.push_back((q_s[frame] - q_s[frame - 1])/dt);
    //     cout << "f[frame " << frame << "] = " << f[frame] << endl;
    //     cout << "q[frame " << frame << "] = " << q_s[frame] << endl;

    //     double K_hat = K + B / dt+ L * dt;
    //     double q0_star = M_x * q0_ddot[frame] + B_x * q0_dot[frame] + K_x * q0[frame];
    //     double u_x_star_tem = ((M_x * q_x_hat[frame - 1]) + dt * q0_star - dt * K_x * q_x[frame - 1])
    //     / (M_x + B_x * dt + dt * dt * K_x);
    //     u_x_star.push_back(u_x_star_tem);

    //     double q_x_star_hat = proj_Q(u_x_star_tem);
        
    //     double q_x_tem = q_x_star_hat * dt +q_x[frame - 1];

    //     double lamda = q0_star - M_x * (q_x_star_hat- q_x_hat[frame - 1])/ dt - B_x * q_x_star_hat - K_x * q_x_tem;

    //     double q_x_star_tem = q_x[frame - 1] + dt * u_x_star[frame];
    //     // std::cout << "q_x_star: " << q_x_star_tem << std::endl;
    //     q_x_star.push_back(q_x_star_tem);

    //     double phi_b_tem = (B * (q_x[frame - 1] - q_s[frame - 1])) / dt - L * a[frame - 1];
    //     phi_b.push_back(phi_b_tem);
    //     // std::cout << "phi_b_tem: " << phi_b_tem << std::endl;


    //     double phi_a_tem = M * (q_s[frame] - q_x[frame - 1] - dt * q_x_hat[frame - 1]) / (dt * dt);
    //     phi_a.push_back(phi_a_tem);
    //     // std::cout << "phi_a_tem: " << phi_a_tem << std::endl;


    //     double q_s_star_tem = q_s[frame] + (phi_b[frame] - phi_a[frame]) / (K_hat + M / (dt * dt));
    //     q_s_star.push_back(q_s_star_tem);
    //     // std::cout << "q_s_star: " << q_s_star_tem << std::endl;


    //     double Mat = K_hat + M / (dt * dt);

    //     double tao_star_tem = Mat * ((q_x_star[frame] - q_s_star[frame]) - dt * dt * lamda / (M_x + dt * B_x +  dt * dt * K_x));
    
    //     tau_star.push_back(tao_star_tem);

    //     std::cout << "tao_star: " << tao_star_tem << endl;
    //     double tau_tem = proj(tau_star[frame]);
    //     tau.push_back(tau_tem);

    //     return tau[frame];


    // }

    // void refreshOnPositionControl() override
    // {
    //     double K_hat = K + B / dt + L * dt;
    //     double q_x_tem = q_s_star[frame] + tau[frame] / (K_hat + M / (dt * dt));
    //     q_x.push_back(q_x_tem);

    //     double q_x_hat_tem = (q_x[frame] - q_x[frame - 1]) / dt;
    //     q_x_hat.push_back(q_x_hat_tem);
    //     std::cout << "q_x_hat_tem: " << q_x_hat_tem << std::endl;
    
    //     double a_tem = a[frame - 1]  + dt * (q_x[frame] - q_s[frame]);
    //     a.push_back(a_tem);

    //     frame++;
    // }


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

        Eigen::Matrix2d K_hat = K + B / dt + L * dt;
        // get new u_x_star
        Eigen::Vector2d u_x_star_tem = (M_x + B_x * dt ).inverse() * (M_x * u_x[frame - 1] + dt * (M_x * q0_ddot[frame] + B_x * q0_dot[frame] + K_x * q0[frame] + tau_ext[frame]));
        u_x_star.push_back(u_x_star_tem);
        
        // get new q_x_star
        Eigen::Vector2d q_x_star_tem = q_x[frame - 1] + dt * u_x_star[frame];
        q_x_star.push_back(q_x_star_tem);

        Eigen::Matrix2d h1 = Eigen::Matrix2d::Identity() + (M_x + B_x * dt ).inverse() * dt * dt * K_x;
        Eigen::Matrix2d h2 = K_hat + M / (dt * dt);

        if(f[frame][0] > 2 || f[frame][1] > 2)
        {
            Q_max = Eigen::Vector2d(0.1, 0.1);
        }
        else
        {
            Q_max = Eigen::Vector2d(0.6, 0.6);
        }

        Eigen::Vector2d q_x_lim = q_x[frame - 1] + dt * proj_Q((h1.inverse() * q_x_star[frame] - q_x[frame - 1]) / dt);
        Eigen::Vector2d lamda = (M_x + B_x * dt ) / (dt * dt) *(q_x_star[frame] - h1 * q_x_lim);

        // get new phi_b
        Eigen::Vector2d phi_b_tem;
        phi_b_tem = (B * (q_x[frame - 1] - q_s[frame - 1])) / dt - L * a[frame-1];
        phi_b.push_back(phi_b_tem);

        // get new phi a
        Eigen::Vector2d phi_a_tem;
        phi_a_tem = M * (q_s[frame] - q_x[frame - 1] - dt * u_x[frame - 1]) / (dt * dt);
        phi_a.push_back(phi_a_tem);

        // get new q_star
        Eigen::Vector2d q_star_tem;
        //q_star_tem = q_s[frame] + (phi_b[frame] - (K_hat + M / (dt * dt)).inverse() * phi_a[frame]);
        q_star_tem = q_s[frame] + (K_hat + M / (dt * dt)).inverse() * (phi_b[frame] - phi_a[frame]);
        q_s_star.push_back(q_star_tem);

        // get new tau_star
        Eigen::Vector2d tau_star_tem;
        tau_star_tem = h2 * h1.inverse() * q_x_star[frame] - h2 * q_s_star[frame] - h2 * h1.inverse() * (M_x + B_x * dt ).inverse() * dt * dt * lamda;
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
