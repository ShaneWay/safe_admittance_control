#include <BaseController.h>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

BaseController::BaseController(const ConfigLoader& loader)
{   
    YAML::Node ctrl = loader.getNode("controller");
    YAML::Node robot = loader.getNode("robot");

    M_x = loader.getMatrix2d(ctrl["M_x"]);
    B_x = loader.getMatrix2d(ctrl["B_x"]);
    K_x = loader.getMatrix2d(ctrl["K_x"]);
    K = loader.getMatrix2d(ctrl["K"]);
    B = loader.getMatrix2d(ctrl["B"]);
    L = loader.getMatrix2d(ctrl["L"]);
    M = loader.getMatrix2d(ctrl["M"]);
    F_max = loader.getVector2d(ctrl["F_max"]);
    Q_max = loader.getVector2d(ctrl["Q_max"]);
    f_d_tem = loader.getVector2d(ctrl["f_d_tem"]);
    dt = ctrl["dt"].as<double>();
    SimTime = ctrl["SimTime"].as<double>();
    controller_name = ctrl["controller_name"].as<std::string>();
    control_target = ctrl["control_target"].as<std::string>();

    parseTarget(ctrl);

    init_angle = loader.getVector2d(robot["init_angle"]);
    init_pos = loader.getVector2d(robot["init_pos"]);
    sin_T = robot["sin_T"].as<double>();;
    amplitude = robot["amplitude"].as<double>();;
    x_amplitude = robot["x_amplitude"].as<double>();;

    init_angle[0] = init_angle[0] / 180. * M_PI;
    init_angle[1] = init_angle[1] / 180. * M_PI;
    omega = 2 * M_PI / sin_T;

    q0.push_back(init_angle);
    q0_dot.push_back(Eigen::Vector2d::Zero());
    q0_ddot.push_back(Eigen::Vector2d::Zero());
    X.push_back(init_pos);
    X_x.push_back(init_pos);

    X0.push_back(init_pos); 
    X0_dot.push_back(Eigen::Vector2d::Zero());
    X0_ddot.push_back(Eigen::Vector2d::Zero());

    tau.push_back(Eigen::Vector2d::Zero());
    tau_star.push_back(Eigen::Vector2d::Zero());
    tau_ext.push_back(Eigen::Vector2d::Zero());

    u_x.push_back(Eigen::Vector2d::Zero());
    u_x_star.push_back(Eigen::Vector2d::Zero());
    
    q_x.push_back(init_angle);
    q_x_star.push_back(Eigen::Vector2d::Zero());
    q_x_hat.push_back(Eigen::Vector2d::Zero());

    phi_b.push_back(Eigen::Vector2d::Zero());
    phi_a.push_back(Eigen::Vector2d::Zero());

    q_s_star.push_back(Eigen::Vector2d::Zero());
    q_s.push_back(init_angle);
    q_s_hat.push_back(Eigen::Vector2d::Zero());

    a.push_back(Eigen::Vector2d::Zero());
    integral_a.push_back(Eigen::Vector2d::Zero());

    f.push_back(Eigen::Vector2d::Zero());
    f_d.push_back(f_d_tem);


    frame = 1;
    for(int i=0; i< 10000; i++)
    {
        
        f_d.push_back(f_d_tem);
        
    }
    
    this->printParams();
    if (target_ == ControlTarget::PositionControl)
    {
        std::cout << "generate===============================" << std::endl;
        generateJointTrajectory();
        plotGeneratedTrajectory();
        std::cout << "generate===end============================" << std::endl;
    }
}

void BaseController::plotGeneratedTrajectory()
{

     // plot tau_star
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

    vector<vector<double>> q0_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i = 0; i < q0.size(); i++)
        {
            tem.push_back(q0[i][j]);
        }
        q0_plot.push_back(tem);
    }

    plt::figure_size(1200, 780);
    for (int i = 0; i < 2; i++)
    {
        string X0_string, q0_string;
        X0_string = "X0" + to_string(i+1);
        q0_string = "q0" + to_string(i+1);
        plt::named_plot(X0_string, X0_plot[i]);
        plt::named_plot(q0_string, q0_plot[i]);

    }
    
    plt::xlabel("Time (us)");
    plt::ylabel("Position (m)");
    plt::title("Cartesian Position");
    plt::legend();
    plt::save("GeneratedTrajectory.pdf");

}

void BaseController::parseTarget(YAML::Node &ctrl)
{
    if (control_target == "position")
    {
        target_ = ControlTarget::PositionControl;
    }else{
        target_ = ControlTarget::ForceControl;
    }
}

void BaseController::generateJointTrajectory()
{
    sleep(2);
    unsigned int time_count = 0;

    Eigen::Vector2d X_0_tem , X_0_dot_tem, X_0_ddot_tem;
    
    
    double t;
    int count_total = SimTime / dt + 1;
    int flat_count = count_total - sin_T / 4 / dt;
    // std::cout << "count_total" << count_total << std::endl;
    for (time_count = 0; time_count < count_total; time_count++ )
    {

        t = time_count * dt;
        if(time_count < flat_count)
        {
            X_0_tem[0] = x_amplitude * t + init_pos[0];
            X_0_tem[1] = amplitude * sin(omega * t) + init_pos[1];

            X_0_dot_tem[0] = x_amplitude;
            X_0_dot_tem[1] = 1.0 * amplitude * omega * cos(omega * t);

            X_0_ddot_tem[0] = 0.;
            X_0_ddot_tem[1] = - 1.0 * amplitude * omega * omega * sin(omega * t);

            // X_0_tem[0] = 0.1 * sin(omega * t) + init_pos[0];
            // X_0_tem[1] = 0 * amplitude * sin(omega * t) + init_pos[1];
        
            // X_0_dot_tem[0] = 0.1 * omega * cos(omega * t);
            // X_0_dot_tem[1] = 0.;

            // X_0_ddot_tem[0] = -0.1 * omega * omega * sin(omega * t);
            // X_0_ddot_tem[1] = 0.;
        }
        else
        {
            X_0_tem[0] = X0[flat_count - 1][0];
            X_0_tem[1] = X0[flat_count - 1][1];
        }

        // update x_0, x_0_dot, x_0_ddot

        X0.push_back(X_0_tem);
        X0_dot.push_back(X_0_dot_tem);
        X0_ddot.push_back(X_0_ddot_tem);
        // std::cout << "jacobian_now" << jacobian_now.transpose() << std::endl;

        model.getJacobianMatrixTwoDOF(q0[time_count], jacobian_now);
        // std::cout << "jacobian_now" << jacobian_now.transpose() << std::endl;

        if ( frame == 0)
        {
            jacobian_last = jacobian_now;
        }

        /// make sure that k, k+1, k-1
        /// update q0, q0_dot, q0_ddot

        q0_dot.push_back(jacobian_now.inverse() * X0_dot[time_count + 1]);
        q0.push_back(q0[time_count] + dt * q0_dot[time_count]);
        q0_ddot.push_back(jacobian_now.inverse() * (X0_ddot[time_count + 1] - (jacobian_now - jacobian_last) / dt * q0_dot[time_count]));

        jacobian_last = jacobian_now;
    }
    q0_dot[0] = q0_ddot[1];
    q0_dot[0] = q0_ddot[1];
}

Eigen::Vector2d BaseController::getTorque(Eigen::Vector2d & f_ext_from_sensor, Eigen::Vector2d& q_frome_sensor)
{
    switch (target_)
    {
    case ControlTarget::ForceControl:
        return getTorqueOnForceControl(f_ext_from_sensor, q_frome_sensor);
        break;
    
    case ControlTarget::PositionControl:
        return getTorqueOnPositionControl(f_ext_from_sensor, q_frome_sensor);
        break;
    }
}
void BaseController::refresh()
{
    switch (target_)
    {
    case ControlTarget::ForceControl:
        refreshOnForceControl();
        break;
    
    case ControlTarget::PositionControl:
        refreshOnPositionControl();
        break;
    }
}

void BaseController::printParams() const
{
    std::cout << "====Base Controller Parameters====\n"
              << "M_x: " << M_x << "\n"
              << "B_x: " << B_x << "\n"
              << "K: " << K << "\n"
              << "B: " << B << "\n"
              << "L: " << L << "\n"
              << "M: " << M << "\n"
              << "F_max: " << F_max.transpose() << "\n"
              << "Q_max: " << Q_max.transpose() << "\n"
              << "f_d_tem: " << f_d_tem.transpose() << "\n"
              << "SimeTime: " << SimTime << "\n"
              << "controller_name <<: " << controller_name << "\n"
              << "Control_target <<: " << control_target << "\n"
              << "================================\n";
}

void BaseController::plot_tau()
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

void BaseController::plot_real_tau()
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

void BaseController::plotExternalForce()
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


void BaseController::plotJointAngle()
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


void BaseController::plotJointSpeed()
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

void BaseController::plotCartesianSpeed()
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

void BaseController::plotCartesianPosition()
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

Eigen::Vector2d BaseController::proj(Eigen::Vector2d& tau_star_input)
{
    Eigen::Vector2d tau_projected;
    for (int i = 0; i < 2; i++)
    {
        if (tau_star_input[i] < -F_max[i])
        {
            tau_projected[i] =  -F_max[i];
        }
        else if (tau_star_input[i] > F_max[i])
        {
            tau_projected[i] = F_max[i];
        }
        else
        {
            tau_projected[i] = tau_star_input[i];
        }
    }
    return tau_projected;
}