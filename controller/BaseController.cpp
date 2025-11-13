#include <BaseController.h>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

BaseController::BaseController(const ConfigLoader& loader)
{   
    YAML::Node ctrl = loader.getNode("controller");
    YAML::Node robot = loader.getNode("robot");

    M_x = loader.getMatrix4d("controller.M_x");
    B_x = loader.getMatrix4d("controller.B_x");
    K_x = loader.getMatrix4d("controller.K_x");
    K = loader.getMatrix4d("controller.K");
    B = loader.getMatrix4d("controller.B");
    L = loader.getMatrix4d("controller.L");
    M = loader.getMatrix4d("controller.M");
    F_max = loader.getVector2d("controller.F_max")
    Q_max = loader.getVector2d("controller.Q_max")
    f_d_tem = loader.getVector2d("controller.f_d_tem")
    dt = ctrl["dt"].as<double>();
    SimTime = ctrl["SimTime"].as<double>();
    control_mode = ctrl["control_mode"].as<std::string>();
    control_target = ctrl["control_target"].as<std::string>();

    parseTarget(ctrl);

    init_angle = loader.getVector2d("init_angle");
    init_pos = loader.getVector2d("init_pos");
    omiga = robot["omiga"].as<double>();;
    amplitude = robot["amplitude"].as<double>();;
    x_amplitude = robot["x_amplitude"].as<double>();;

    init_angle[0] = init_angle[0] / 180. * M_PI;
    init_angle[1] = init_angle[1] / 180. * M_PI;

    tau.push_back(Eigen::Vector2d::Zero());
    tau_star.push_back(Eigen::Vector2d::Zero());

    u_x.push_back(Eigen::Vector2d::Zero());
    u_x_star.push_back(Eigen::Vector2d::Zero());
    
    q_x.push_back(Eigen::Vector2d::Zero());
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
    }
}

void BaseController::parseTarget(YAML::Node &ctrl)
{
    if (ctrl["control_target"] == "position")
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
    Eigen::Matrix2d jacobian_last;
    Eigen::Matrix2d jacobian_now;
    
    double t;
    int count_total = SimTime / dt + 1;
    for (time_count = 0; time_count < count_total; time_count++ )
    {

        t = time_count * dt;
        if(time_count < 10501)
        {
            X_0_tem[0] = x_amplitude * t + init_pos[0];
            X_0_tem[1] = amplitude * sin(omiga * t) + init_pos[1];

            X_0_dot_tem[0] = x_amplitude;
            X_0_dot_tem[1] = 1.0 * amplitude * omiga * cos(omiga * t);

            X_0_ddot_tem[0] = 0.;
            X_0_ddot_tem[1] = - 1.0 * amplitude * omiga * omiga * sin(omiga * t);

            // X_0_tem[0] = 0.1 * sin(omiga * t) + init_pos[0];
            // X_0_tem[1] = 0 * amplitude * sin(omiga * t) + init_pos[1];
        
            // X_0_dot_tem[0] = 0.1 * omiga * cos(omiga * t);
            // X_0_dot_tem[1] = 0.;

            // X_0_ddot_tem[0] = -0.1 * omiga * omiga * sin(omiga * t);
            // X_0_ddot_tem[1] = 0.;
        }
        else
        {
            X_0_tem[0] = X0[10500][0];
            X_0_tem[1] = X0[10500][1];
        }

        // update x_0, x_0_dot, x_0_ddot

        X0.push_back(X_0_tem);
        X0_dot.push_back(X_0_dot_tem);
        X0_ddot.push_back(X_0_ddot_tem);

        model.getJacobianMatrixTwoDOF(q0[time_count], jacobian_now);

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
}

double BaseController::getTorque(double & f_ext_from_sensor, double& q_frome_sensor)
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
              << "Control_mode <<: " << control_mode << "\n"
              << "Control_target <<: " << control_target << "\n"
              << "================================\n";
}

void BaseController::plot_tau()
{
    plt::figure_size(1200, 780);

    for (int i = 0; i < 2; i++)
    {
        string tau_string;
        tau_string = "tau" + to_string(i+1);
        plt::named_plot(tau_string, tau[i]);
    }


    plt::xlabel("time (us)");
    plt::ylabel("torque (Nm)");
    plt::title("Joint Torque");
    plt::legend();
    plt::save("result_tao.pdf");
}

void BaseController::plotExternalForce()
{
    plt::figure_size(1200, 780);

    for (int i = 0; i < 2; i++)
    {
        string f_string;
        f_string = "f_ext" + to_string(i+1);
        plt::named_plot(f_string, f[i]);
    }


    plt::xlabel("time (us)");
    plt::ylabel("Force (N)");
    plt::title("ExternalForce");
    plt::legend();
    plt::save("ExternalForce.pdf");
}


void BaseController::plot_q()
{
    plt::figure_size(1200, 780);

    for (int i = 0; i < 2; i++)
    {
        string q_s_string, q_x_string, q_0_string;
        q_s_string = "q_s_joint" + to_string(i+1);
        q_x_string = "q_x_joint" + to_string(i+1);
        q_0_string = "q_0_joint" + to_string(i+1);

        plt::named_plot(q_s_string, q_s[i]);
        plt::named_plot(q_x_string, q_x[i],"--" );
        plt::named_plot(q_0_string, q0[i]);
        
    }

    plt::legend();
    plt::xlabel("Time (us)");
    plt::ylabel("Position (rad)");
    plt::title("Joint Angles");
    // plt::xlim(0, 100);
    plt::save("JointAngles.pdf");
}


void BaseController::plot_q_hat()
{   
   
    for (int i = 0; i < 2; i++)
    {
        string q_s_hat_string;
        q_s_hat_string = "q_s_hat" + to_string(i+1);
        plt::named_plot(q_s_hat_string, q_s_hat[i]);
    }

    plt::figure_size(1200, 780);
    plt::title("q_s_hat");
    plt::xlabel("Time (us)");
    plt::ylabel("omiga (rad / s)");
    plt::save("q_s_hat.pdf");
}

void BaseController::plotCartesianSpeed()
{   
    Eigen::Vector2d X_hat_tem;
    X_hat_tem.push_back(Eigen::Vector2d(0., 0.));
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < X.size(); j++)
        {
            X_hat_tem[j][i].push_back((X[j + 1][i] - X[j][i]) / dt)
        }
    } 
    
    for (int i = 0; i < 2; i++)
    {
        string X_s_hat_string;
        string X0_hat_string;
        X_hat_string = "X_s_hat" + to_string(i+1);
        X0_hat_string = "X0_hat" + to_string(i+1);
        plt::named_plot(X_s_hat_string, X_hat_tem[i]);
        plt::named_plot(X0_hat_string, X0_dot[i]);
    }

    plt::figure_size(1200, 780);
    plt::title("X_s_hat");
    plt::xlabel("Time (us)");
    plt::ylabel("omiga (rad / s)");
    plt::save("X_s_hat.pdf");
}

void BaseController::plotCartesianPosition()
{
    KortexKinematics model;
    vv2d X_x;
    vv2d X_s;

    Eigen::Vector2d X_s_tem;
    for(size_t i = 0; i < q_s.size(); i ++)
    {
        model.getFowardKinematicsTwoDOF(q_s[i], X_s_tem);
        X_s.push_back(X_s_tem);
    }

    
    Eigen::Vector2d X_x_tem;
    for(size_t i = 0; i < q_x.size(); i ++)
    {
        model.getFowardKinematicsTwoDOF(q_x[i], X_x_tem);
        X_x.push_back(X_x_tem);
    }

    plt::figure_size(1200, 780);
    for (int i = 0; i < 2; i++)
    {
        string X0_string, X_s_string, X_s_string;
        X0_string = "X0" + to_string(i+1);
        X_s_string = "X" + to_string(i+1);
        X_x_string = "X_d" + to_string(i+1);
        
        plt::named_plot(X0_string, X0[i], "--");
        plt::named_plot(X_s_string, X_s[i]);
        plt::named_plot(X_x_string, X_x[i]);
    }
}

double BaseController::proj(Eigen::Vector2d& tau_star_input)
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