#pragma once
#include "BaseController.h"
#include <iostream>
#include <numeric>

class KikModify : public BaseController {
public:
    KikModify(BaseParams& params, ControlState& state) : BaseController(params, state)
    {

    }

    Eigen::Vector2d getTorqueOnForceControl(Eigen::Vector2d & f_ext_from_sensor, Eigen::Vector2d& q_frome_sensor) override
    {
        std::cout << "kik ============================" << std::endl;

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
        // u_x_star.push_back(dzn_vx(u_x_star_tem));
        u_x_star.push_back(u_x_star_tem);
        // std::cout << "f_d: " << f_d[frame] << std::endl;
         cout << "f_d[frame " << frame << "] = " << f_d[frame-1].transpose() << endl;

        Eigen::Vector2d q_x_star_tem = q_x[frame - 1] + dt * u_x_star[frame];
        // std::cout << "q_x_star: " << q_x_star_tem << std::endl;
        q_x_star.push_back(q_x_star_tem);

        Eigen::Vector2d phi_b_tem = (B * (q_x[frame - 1] - q_s[frame - 1])) / dt - L * a[frame - 1];
        phi_b.push_back(phi_b_tem);
        // std::cout << "phi_b_tem: " << phi_b_tem << std::endl;

        Eigen::Vector2d phi_a_tem = M * (u_x_star[frame] - q_x_hat[frame - 1]) / dt;
        // cout << "q_x[frame " << frame - 1 << "] = " << q_x[frame - 1].transpose() << endl;

        phi_a.push_back(proj(phi_a_tem));
        // std::cout << "phi_a_tem: " << phi_a_tem << std::endl;

        Eigen::Vector2d q_s_star_tem = q_s[frame] + (K_hat).inverse() * (phi_b[frame] - phi_a[frame]);
        q_s_star.push_back(q_s_star_tem);
        // std::cout << "q_s_star: " << q_s_star_tem << std::endl;

        Eigen::Matrix2d Mat = K_hat + M / (dt * dt);

        Eigen::Vector2d tao_star_tem = K_hat * (q_x_star[frame] - q_s_star[frame]);
    
        tau_star.push_back(tao_star_tem);

        Eigen::Vector2d tau_tem = proj(tau_star[frame]);
        tau.push_back(tau_tem);

        return tau[frame];

    }

    void refreshOnForceControl() override
    {
        Eigen::Matrix2d K_hat = K + B / dt + L * dt;
        Eigen::Vector2d q_x_tem = q_s_star[frame] + K_hat.inverse() * tau[frame];
        q_x.push_back(q_x_tem);

        Eigen::Vector2d q_x_hat_tem = (q_x[frame] - q_x[frame - 1]) / dt;
        q_x_hat.push_back(proj_co(q_x_hat_tem));
    
        Eigen::Vector2d a_tem = a[frame - 1]  + dt * (q_x[frame] - q_s[frame]);
        a.push_back(a_tem);

        frame++;
    }


    Eigen::Vector2d getTorqueOnPositionControl(Eigen::Vector2d & f_ext_from_sensor, Eigen::Vector2d & q_frome_sensor) override
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
        Eigen::Vector2d  u_x_star_tem = (M_x + B_x * dt).inverse() * (M_x * q_x_hat[frame - 1] + dt * (M_x * q0_ddot[frame] + B_x * q0_dot[frame] + K_x * q0[frame] + tau_ext[frame])) ;
        u_x_star.push_back(u_x_star_tem);

        // get new q_x_star
        Eigen::Vector2d q_x_star_tem = q_x[frame - 1] + dt * u_x_star[frame];
        q_x_star.push_back(q_x_star_tem);

        // get new phi_b
        Eigen::Vector2d phi_b_tem;
        phi_b_tem = (B * (q_x[frame - 1] - q_s[frame - 1])) / dt - L * a[frame - 1];
        phi_b.push_back(phi_b_tem);

        // get new phi a
        Eigen::Vector2d phi_a_tem = M * (u_x_star[frame] - q_x_hat[frame - 1]) / dt;

        phi_a.push_back(proj(phi_a_tem));

        // get new q_star
        Eigen::Vector2d q_s_star_tem;
        q_s_star_tem = q_s[frame] + (K_hat).inverse() * (phi_b[frame] - phi_a[frame]);
        q_s_star.push_back(q_s_star_tem);

        // get mat1 and mat2
        Eigen::Matrix2d  Mat1 = Eigen::Matrix2d::Identity() + (M_x + B_x * dt).inverse() * K_x * (dt * dt);
        Eigen::Matrix2d  Mat2 = K_hat;

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
        q_x_tem = q_s_star[frame] + (K_hat).inverse() * tau[frame];
        q_x.push_back(q_x_tem);

        //get new u_x
        Eigen::Vector2d u_x_tem;
        u_x_tem = (q_x[frame] - q_x[frame - 1]) / dt;
        u_x.push_back(proj_co(u_x_tem));

        Eigen::Vector2d q_x_hat_tem = (q_x[frame] - q_x[frame - 1]) / dt;
        q_x_hat.push_back(proj_co(q_x_hat_tem));

        // get new a
        Eigen::Vector2d a_tem;
        a_tem = a[frame - 1] + dt * (q_x[frame] - q_s[frame]);
        a.push_back(a_tem);

        frame++;
    }

    Eigen::Vector2d dzn_vx(Eigen::Vector2d u_x_star)
    {   
        Eigen::Vector2d u_x_star_projected;
        for (int i = 0; i < 2; i++)
        {
            if (u_x_star[i] < -Q_max[i])
            {
                u_x_star_projected[i] = u_x_star[i] - Q_max[i];
            }
            else if (u_x_star[i] > Q_max[i])
            {
                u_x_star_projected[i] = u_x_star[i] - Q_max[i];
            }
            else
            {
                u_x_star_projected[i] = 0;
            }
        }
        return u_x_star_projected;
    }

     Eigen::Vector2d proj_co(Eigen::Vector2d q_x_hat_tem)
    {   
        Eigen::Vector2d q_x_hat_projected;
        
        for (int i = 0; i < 2; i++)
        {
            double u_x_star_abs = std::abs(u_x_star[frame][i]);
            if (q_x_hat_tem[i] < -u_x_star_abs)
            {
                q_x_hat_projected[i] = -u_x_star_abs;
            }
            else if (q_x_hat_tem[i] >u_x_star_abs)
            {
                q_x_hat_projected[i] =u_x_star_abs;
            }
            else
            {
                q_x_hat_projected[i] = q_x_hat_tem[i];
            }
        }
        return q_x_hat_projected;
    }

};
