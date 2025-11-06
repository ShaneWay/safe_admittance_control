#include <BaseController.h>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

BaseController::BaseController(const Config& config)
{   
    M_x = config.controller.M_x;
    B_x = config.controller.B_x;
    K = config.controller.K;
    B = config.controller.B;
    L = config.controller.L;
    M = config.controller.M;
    F_max = config.controller.F_max;
    Q_max = config.controller.Q_max;
    f_d_tem = config.controller.f_d_tem;
    TimeUnit = config.controller.TimeUnit;
    SimTime = config.controller.SimTime;
    control_mode = config.controller.control_mode;
    control_target = config.controller.control_target;

    parseTarget(config);

    init_angle = config.robot.init_angle;
    init_angle = init_angle / 180. * M_PI;

    tau.push_back(0);
    tau_star.push_back(0);

    u_x.push_back(0);
    u_x_star.push_back(0);
    
    q_x.push_back(init_angle);
    q_x_star.push_back(init_angle);
    q_x_hat.push_back(0);

    phi_b.push_back(0);
    phi_a.push_back(0);

    q_s_star.push_back(0);
    q_s.push_back(init_angle);
    q_s_hat.push_back(0);

    a.push_back(0);
    integral_a.push_back(0);

    f.push_back(0);
    f_d.push_back(f_d_tem);

    frame = 1;
    for(int i=0; i< 10000; i++)
    {
        
        f_d.push_back(f_d_tem);
        
    }
    
    this->printParams();
    if (target_ = ControlTarget::PositionControl)
    {
        generateJointTrajectory();
    }
}

void BaseController::parseTarget(const Config& config)
{
    if (config.controller.control_target == "position")
    {
        target_ = ControlTarget::PositionControl;
    }else{
        target_ = ControlTarget::ForceControl;
    }
}

void BaseController::generateJointTrajectory()
{
    unsigned int time_count = 0;
    double t = 0;
    double end_angle = 170 / 180 * M_PI;
    int count_total = SimTime / TimeUnit;
    double step =  (end_angle - init_angle) / count_total;
    for (time_count = 0; time_count < count_total; time_count++ )
    {
        q0.pushback(init_angle + time_count * step )
        q0_dot.pushback(step);
        q0_ddot.pushback(0.);
    }
}

double BaseController::getTorque(const double & T, double & f_ext_from_sensor, double& q_frome_sensor)
{
    switch (target_)
    {
    case ControlTarget::ForceControl:
        return getTorqueOnForceControl(T, f_ext_from_sensor, q_frome_sensor);
        break;
    
    case ControlTarget::PositionControl:
        return getTorqueOnPositionControl(T, f_ext_from_sensor, q_frome_sensor);
        break;
    }
}
void BaseController::refresh(const double & T)
{
    switch (target_)
    {
    case ControlTarget::ForceControl:
        refreshOnForceControl(T);
        break;
    
    case ControlTarget::PositionControl:
        refreshOnPositionControl(T);
        break;
    }
}

BaseController::printParams()
{
    std::cout << "====Base Controller Parameters====\n"
              << "M_x: " << M_x << "\n"
              << "B_x: " << B_x << "\n"
              << "K: " << K << "\n"
              << "B: " << B << "\n"
              << "L: " << L << "\n"
              << "M: " << M << "\n"
              << "F_max: " << F_max << "\n"
              << "Q_max: " << Q_max << "\n"
              << "f_d_tem: " << f_d_tem << "\n"
              << "SimeTime: " << SimTime << "\n"
              << "Control_mode <<: " << control_mode << "\n"
              << "Control_target <<: " << control_target << "\n"
              << "================================\n";
}

void BaseController::plot_tau()
{
    plt::figure_size(1200, 780);
    plt::named_plot("real_tao", tau_star);
    plt::named_plot("projected_tao", tau);
    plt::named_plot("input_f", f);
    plt::xlabel("time (us)");
    plt::ylabel("torque (Nm)");
    plt::legend();
    plt::save("result_tao.pdf");
}


void BaseController::plot_q()
{
    plt::figure_size(1200, 780);
    plt::named_plot("computed_qx", q_x,  "r--");
    plt::named_plot("q_s", q_s);
    plt::legend();
    plt::title("q");
    // plt::xlim(0, 100);
    plt::save("result_qs.pdf");
}

void BaseController::plot_q_hat()
{
    plt::figure_size(1200, 780);
    plt::named_plot("q_x_hat", q_x_hat,  "r--");
    // plt::named_plot("q_s_hat", q_s_hat);
    plt::legend();
    plt::title("q_hat");
    // plt::xlim(0, 100);
    plt::save("q_hat.pdf");
}

double BaseController::proj(double tau_star_input)
{
    if (tau_star_input < -F_max)
    {
        return -F_max;
    }
    else if (tau_star_input > F_max)
    {
        return F_max;
    }
    else{
        return tau_star_input;
    }
}