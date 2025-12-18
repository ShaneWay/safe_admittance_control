#include <ControlState.h>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

ControlState::ControlState(const BaseParams& params): params_(params)
{
    q0.push_back(params_.init_angle);
    q0_dot.push_back(Eigen::Vector2d::Zero());
    q0_ddot.push_back(Eigen::Vector2d::Zero());
    X.push_back(params_.init_pos);
    X_x.push_back(params_.init_pos);

    X0.push_back(params_.init_pos); 
    X0_dot.push_back(Eigen::Vector2d::Zero());
    X0_ddot.push_back(Eigen::Vector2d::Zero());

    tau.push_back(Eigen::Vector2d::Zero());
    tau_star.push_back(Eigen::Vector2d::Zero());
    tau_ext.push_back(Eigen::Vector2d::Zero());

    u_x.push_back(Eigen::Vector2d::Zero());
    u_x_star.push_back(Eigen::Vector2d::Zero());
    
    q_x.push_back(params_.init_angle);
    q_x_star.push_back(Eigen::Vector2d::Zero());
    q_x_hat.push_back(Eigen::Vector2d::Zero());

    phi_b.push_back(Eigen::Vector2d::Zero());
    phi_a.push_back(Eigen::Vector2d::Zero());

    q_s_star.push_back(Eigen::Vector2d::Zero());
    q_s.push_back(params_.init_angle);
    q_s_hat.push_back(Eigen::Vector2d::Zero());

    e_q.push_back(Eigen::Vector2d::Zero());
    e_r.push_back(Eigen::Vector2d::Zero());

    a.push_back(Eigen::Vector2d::Zero());
    integral_a.push_back(Eigen::Vector2d::Zero());

    f.push_back(Eigen::Vector2d::Zero());
    
    frame = 1;

    for(int i=0; i< 4000; i++)
    {
        
        f_d.push_back(params_.f_d_0);
        
    }
    for(int i = 4000; i < 8000; i++)
    {
        f_d.push_back(params_.f_d_1);

    }
    for(int i = 8000; i < 12000; i++)
    {
        f_d.push_back(params_.f_d_2);

    }

}

void ControlState::plot_tau()
{
    plt::figure_size(1200, 780);

    vector<vector<double>> tau_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < tau.size(); i++)
        {
            tem.push_back(tau[i][j]);
        }
        tau_plot.push_back(tem);
    }

    for (int i = 0; i < 2; i++)
    {
        string tau_string;
        tau_string = "tau" + to_string(i+1);
        plt::named_plot(tau_string, tau_plot[i]);
    }


    plt::xlabel("time (us)");
    plt::ylabel("torque (Nm)");
    plt::title("Joint Torque");
    plt::legend();
    plt::save("result_tao.pdf");
}

void ControlState::plot_real_tau()
{
    plt::figure_size(1200, 780);

    vector<vector<double>> tau_star_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < tau_star.size(); i++)
        {
            tem.push_back(tau_star[i][j]);
        }
        tau_star_plot.push_back(tem);
    }

    for (int i = 0; i < 2; i++)
    {
        string tau_string;
        tau_string = "tau" + to_string(i+1);
        plt::named_plot(tau_string, tau_star_plot[i]);
    }


    plt::xlabel("time (us)");
    plt::ylabel("torque (Nm)");
    plt::title("Joint Torque");
    plt::legend();
    plt::save("real_tao.pdf");
}

void ControlState::plotExternalForce()
{
    plt::figure_size(1200, 780);

    vector<vector<double>> f_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < f.size(); i++)
        {
            tem.push_back(f[i][j]);
        }
        f_plot.push_back(tem);
    }

    for (int i = 0; i < 2; i++)
    {
        string f_string;
        f_string = "f_ext" + to_string(i+1);
        plt::named_plot(f_string, f_plot[i]);
    }


    plt::xlabel("time (us)");
    plt::ylabel("Force (N)");
    plt::title("ExternalForce");
    plt::legend();
    plt::save("ExternalForce.pdf");
}


void ControlState::plotJointAngle()
{
    plt::figure_size(1200, 780);

    vector<vector<double>> q_s_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < q_s.size(); i++)
        {
            tem.push_back(q_s[i][j]);
        }
        q_s_plot.push_back(tem);
    }

    vector<vector<double>> q_x_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < q_x.size(); i++)
        {
            tem.push_back(q_x[i][j]);
        }
        q_x_plot.push_back(tem);
    }

    vector<vector<double>> q0_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < q0.size(); i++)
        {
            tem.push_back(q0[i][j]);
        }
        q0_plot.push_back(tem);
    }

    for (int i = 0; i < 2; i++)
    {
        string q_s_string, q_x_string, q_0_string;
        q_s_string = "q_s_joint" + to_string(i+1);
        q_x_string = "q_x_joint" + to_string(i+1);
        q_0_string = "q_0_joint" + to_string(i+1);

        plt::named_plot(q_s_string, q_s_plot[i]);
        plt::named_plot(q_x_string, q_x_plot[i],"--" );
        plt::named_plot(q_0_string, q0_plot[i]);
        
    }

    plt::xlabel("Time (us)");
    plt::ylabel("Position (rad)");
    plt::legend();

    plt::title("Joint Angles");
    // plt::xlim(0, 100);
    plt::save("JointAngles.pdf");
}


void ControlState::plotJointSpeed()
{   
    plt::figure_size(1200, 780);

    vector<vector<double>> q_s_hat_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < q_s_hat.size(); i++)
        {
            tem.push_back(q_s_hat[i][j]);
        }
        q_s_hat_plot.push_back(tem);
    }
   
    for (int i = 0; i < 2; i++)
    {
        string q_s_hat_string;
        q_s_hat_string = "q_s_hat" + to_string(i+1);
        plt::named_plot(q_s_hat_string, q_s_hat_plot[i]);
    }

    plt::title("q_s_hat");
    plt::xlabel("Time (us)");
    plt::ylabel("omega (rad / s)");
    plt::legend();
    plt::save("JointSpeed.pdf");
}

void ControlState::plotCartesianSpeed()
{   
    plt::figure_size(1200, 780);

    Eigen::Matrix2d jacobian;

    for(size_t i = 0; i < q_s_hat.size(); i ++)
    {
        model.getJacobianMatrixTwoDOF(q_s_hat[i], jacobian);
        X_hat.push_back(jacobian * q_s_hat[i]);
    }
    
    vector<vector<double>> X_hat_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < X_hat.size(); i++)
        {
            tem.push_back(X_hat[i][j]);
        }
        X_hat_plot.push_back(tem);
    }

    vector<vector<double>> X0_hat_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < X0_dot.size(); i++)
        {
            tem.push_back(X0_dot[i][j]);
        }
        X0_hat_plot.push_back(tem);
    }
    
    for (int i = 0; i < 2; i++)
    {
        string X_hat_string;
        string X0_hat_string;
        X_hat_string = "X_s_hat" + to_string(i+1);
        X0_hat_string = "X0_hat" + to_string(i+1);
        plt::named_plot(X_hat_string, X_hat_plot[i]);
        plt::named_plot(X0_hat_string, X0_hat_plot[i]);
    }

    plt::title("X_s_hat");
    plt::xlabel("Time (us)");
    plt::ylabel("omega (rad / s)");
    plt::legend();
    plt::save("CartesianSpeed.pdf");
}

void ControlState::plotCartesianPosition()
{
    plt::figure_size(1200, 780);

    vector<vector<double>> X0_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < X0.size(); i++)
        {
            tem.push_back(X0[i][j]);
        }
        X0_plot.push_back(tem);
    }

    
    Eigen::Vector2d X_tem;
    for(size_t i = 0; i < q_s.size(); i++)
    {
        model.getFowardKinematicsTwoDOF(q_s[i+1], X_tem);
        X.push_back(X_tem);
    }

    vector<vector<double>> X_s_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < X.size(); i++)
        {
            tem.push_back(X[i][j]);
        }
        X_s_plot.push_back(tem);
    }

    
    Eigen::Vector2d X_d_tem;
    for(size_t i = 0; i < q_x.size(); i++)
    {
        model.getFowardKinematicsTwoDOF(q_x[i+1], X_d_tem);
        X_x.push_back(X_d_tem);
    }

    vector<vector<double>> X_x_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < X_x.size(); i++)
        {
            tem.push_back(X_x[i][j]);
            // std::cout << tem[i] << std::endl;
        }
        X_x_plot.push_back(tem);
    }

    for (int i = 0; i < 2; i++)
    {
        string X0_string, X_s_string, X_x_string;
        X0_string = "X0" + to_string(i+1);
        X_s_string = "X" + to_string(i+1);
        X_x_string = "X_d" + to_string(i+1);
        
        plt::named_plot(X0_string, X0_plot[i], "--");
        plt::named_plot(X_s_string, X_s_plot[i]);
        plt::named_plot(X_x_string, X_x_plot[i]);
    }
    plt::xlabel("Time (us)");
    plt::ylabel("Position (m)");
    plt::title("Cartesian Position");
    plt::legend();
    plt::save("CartesianPosition.pdf");
}

void ControlState::saveData()
{
    int count_total = params_.SimTime / params_.dt;
.
    ofstream outfile_q0("1-q0.txt", ios::out);
    ofstream outfile_q("2-q_s.txt", ios::out);
    ofstream outfile_q_x("3-q_x.txt", ios::out);

    ofstream outfile_tau("4-tau.txt", ios::out);
    ofstream outfile_tau_star("5-tau_star.txt", ios::out);
    ofstream outfile_f_ext("6-f_ext.txt", ios::out);

    ofstream outfile_X0("7-X0.txt", ios::out);
    ofstream outfile_X("8-X.txt", ios::out);
    ofstream outfile_X_d("9-X_d.txt", ios::out);

    ofstream outfile_JointForce("10-ExternalJointForce.txt", ios::out);
    ofstream outfile_q_s_hat("11-q_s_hat.txt", ios::out);
    ofstream outfile_q_x_hat("12-q_x_hat.txt", ios::out);

    for (int i = 0; i < count_total; i++)
    {
        outfile_JointForce << tau_ext[i][0] << " "  << tau_ext[i][1] << '\n';
    }

    for (int i = 0; i < count_total; i++)
    {
        outfile_q_s_hat << q_s_hat[i][0] << " " << q_s_hat[i][1] << '\n';
    }

    for (int i = 0; i < count_total; i++)
    {
        outfile_q_x_hat << q_x_hat[i][0] << " " << q_x_hat[i][1] << '\n';
    }

    for (int i = 0; i < count_total; i++)
    {
        outfile_q0 << q0[i][0] << " "  << q0[i][1] << '\n';
    }

    for (int i = 0; i < count_total; i++)
    {
        outfile_q << q[i][0] << " "  << q[i][1] << '\n';
    }

    for (int i = 0; i < count_total; i++)
    {
        outfile_q_x << q_x[i][0] << " "  << q_x[i][1] << '\n';
    }

    for (int i = 0; i < count_total; i++)
    {
        outfile_tau << tau[i][0] << " "  << tau[i][1] << '\n';
    }

    for (int i = 0; i < count_total; i++)
    {
        outfile_tau_star << tau_star[i][0] << " "  << tau_star[i][1] << '\n';
    }

    for (int i = 0; i < count_total; i++)
    {
        outfile_f_ext << f[i][0] << " "  << f_ext[i][1] << '\n';
    }

    for (int i = 0; i < count_total; i++)
    {
        outfile_X0 << X0[i][0] << " "  << X0[i][1] << '\n';
    }

    for (int i = 0; i < count_total; i++)
    {
        outfile_X << X[i][0] << " "  << X[i][1] << '\n';
    }

    for (int i = 0; i < count_total; i++)
    {
        outfile_X_d << X_d[i][0] << " "  << X_d[i][1] << '\n';
    }

    outfile_q0.close();
    outfile_q.close();
    outfile_q_x.close();
    outfile_tau.close();
    outfile_tau_star.close();
    outfile_f_ext.close();
    outfile_X0.close();
    outfile_X.close();
    outfile_X_d.close();
    outfile_JointForce.close();
    outfile_q_s_hat.close();
    outfile_q_x_hat.close();


}