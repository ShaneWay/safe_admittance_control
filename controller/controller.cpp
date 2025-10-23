#include <controller.h>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

controller::controller(const Config& config)
{

    M_x = config.controller.M_x;
    B_x = config.controller.B_x;
    K = config.controller.K;
    B = config.controller.B;
    L = config.controller.L;
    M = config.controller.M;
    F_max = config.controller.F_max;
    f_d_tem = config.controller.f_d_tem;
    Q_max = config.controller.Q_max;
    TimeUnit = config.controller.TimeUnit;
    control_mode = config.controller.control_mode;
    // std::cout << control_mode << std::endl;
    parseMode(control_mode);

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
    for(int i=0; i< 5001; i++)
    {
        if(i >1000 && i < 1500)
        {
            f_d.push_back(10);
        }
        else{
            f_d.push_back(f_d_tem);
        }
    }
}

void controller::printParams() const {
    std::cout << "====Controller Parameters====\n"
              << "M_x: " << M_x << "\n"
              << "B_x: " << B_x << "\n"
              << "K: " << K << "\n"
              << "B: " << B << "\n"
              << "L: " << L << "\n"
              << "M: " << M << "\n"
              << "F_max: " << F_max << "\n"
              << "f_d_tem: " << f_d_tem << "\n"
              << "Q_max: " << Q_max << "\n"
              << "control_mode <<: " << control_mode << "\n"
              << "================================\n";
}

void controller::parseMode(const std::string& modeStr) {
    if (modeStr == "normal") std::cout << "1111" << std::endl ; mode_ = ControlMode::NORMAL;
    if (modeStr == "sfc") mode_ = ControlMode::SFC;
    if (modeStr == "smc") {
        std::cout << "3333" << std::endl ;
        mode_ = ControlMode::SMC;
    }
}

double controller::getTorque(const double & T, double & f_ext_from_sensor, double & q_frome_sensor)
{
    double tor;
    switch (mode_)
    {
    case ControlMode::NORMAL:
        tor = getTorqueNormal(T, f_ext_from_sensor, q_frome_sensor);
        return tor;
        break;

    case ControlMode::SFC:
        tor = getTorqueSFC(T, f_ext_from_sensor, q_frome_sensor);
        return tor;
        break;

    case ControlMode::SMC:
        tor = getTorqueSMC(T, f_ext_from_sensor, q_frome_sensor);
        return tor;
        break;

    default:
        cout << "please choose a control mode for the robot!" << endl;
        break;
    }
}

double controller::getTorqueNormal(const double & T, double & f_ext_from_sensor, double & q_frome_sensor)
{
    f.push_back(f_ext_from_sensor);
    q_s.push_back(q_frome_sensor);
    q_s_hat.push_back((q_s[frame] - q_s[frame - 1])/T);
    cout << "f[frame " << frame << "] = " << f[frame] << endl;
    cout << "q[frame " << frame << "] = " << q_s[frame] << endl;

    double K_hat = K + B / T + L * T;
    double u_x_star_tem = ((M_x * q_x_hat[frame - 1]) + T * (f[frame] + f_d[frame - 1]))
    / (M_x + B_x * T);
    u_x_star.push_back(u_x_star_tem);

    double phi_b_tem = (B * (q_x[frame - 1] - q_s[frame - 1])) / T - L * a[frame - 1];
    phi_b.push_back(phi_b_tem);

    double phi_a_tem = M * (q_s[frame] - q_x[frame - 1] - T * q_x_hat[frame - 1]) / (T * T);
    phi_a.push_back(phi_a_tem);

    double q_s_star_tem = q_s[frame] + (phi_b[frame] - phi_a[frame]) / (K_hat + M / (T * T));

    double Mat = K_hat + M / (T * T);
    double tau_tem = Mat * (q_x[frame] - q_s_star_tem);
    tau.push_back(tau_tem);

    return tau[frame];
}

double controller::getTorqueSFC(const double & T, double & f_ext_from_sensor, double & q_frome_sensor)
{
    f.push_back(f_ext_from_sensor);
    q_s.push_back(q_frome_sensor);
    q_s_hat.push_back((q_s[frame] - q_s[frame - 1])/T);
    cout << "f[frame " << frame << "] = " << f[frame] << endl;
    cout << "q[frame " << frame << "] = " << q_s[frame] << endl;
    
    double a_tem = f[frame] - B_x * std::abs(q_x_hat[frame - 1]) * std::abs(q_x_hat[frame - 1]) * q_x_hat[frame -1] / M_x;
    a.push_back(a_tem);
    double q_x_hat_tem = q_x_hat[frame - 1] + a[frame] * T;
    q_x_hat.push_back(q_x_hat_tem);
    double integral_a_tem = integral_a[frame - 1] + T * (q_x[frame] - q_s[frame]);
    integral_a.push_back(integral_a_tem);
    double tau_star_tem = M * a[frame] + K * (q_x[frame] - q_s[frame]) * B * (q_x_hat[frame] - q_s_hat[frame]) + L * integral_a[frame];
    tau_star.push_back(tau_star_tem);
    
    tau.push_back(proj(tau_star_tem));

    return tau[frame];
}


double controller::getTorqueSMC(const double & T, double & f_ext_from_sensor, double & q_frome_sensor)
{ 

    f.push_back(f_ext_from_sensor);
    q_s.push_back(q_frome_sensor);
    q_s_hat.push_back((q_s[frame] - q_s[frame - 1])/T);
    cout << "f[frame " << frame << "] = " << f[frame] << endl;
    cout << "q[frame " << frame << "] = " << q_s[frame] << endl;

    // f_d.push_back(f_d_tem);

    double K_hat = K + B / T + L * T;

    double u_x_star_tem = ((M_x * q_x_hat[frame - 1]) + T * (f[frame] + f_d[frame - 1]))
    / (M_x + B_x * T);
    u_x_star.push_back(u_x_star_tem);
    // std::cout << "f_d: " << f_d[frame] << std::endl;

    double q_x_star_hat = proj_Q(u_x_star_tem);
    std::cout << "q_x_star_hat: " << q_x_star_hat << std::endl;

    double lamda = f[frame] + f_d[frame] - M_x * (q_x_star_hat - q_x_hat[frame - 1])/ T - B_x * q_x_star_hat;
    // std::cout << "lamda: " << lamda << std::endl;

    double q_x_star_tem = q_x[frame - 1] + T * u_x_star[frame];
    // std::cout << "q_x_star: " << q_x_star_tem << std::endl;
    q_x_star.push_back(q_x_star_tem);

    double phi_b_tem = (B * (q_x[frame - 1] - q_s[frame - 1])) / T - L * a[frame - 1];
    phi_b.push_back(phi_b_tem);
    // std::cout << "phi_b_tem: " << phi_b_tem << std::endl;


    double phi_a_tem = M * (q_s[frame] - q_x[frame - 1] - T * q_x_hat[frame - 1]) / (T * T);
    phi_a.push_back(phi_a_tem);
    // std::cout << "phi_a_tem: " << phi_a_tem << std::endl;


    double q_s_star_tem = q_s[frame] + (phi_b[frame] - phi_a[frame]) / (K_hat + M / (T * T));
    q_s_star.push_back(q_s_star_tem);
    // std::cout << "q_s_star: " << q_s_star_tem << std::endl;


    double Mat = K_hat + M / (T * T);

    double tao_star_tem = Mat * ((q_x_star[frame] - q_s_star[frame]) - T * T * lamda / (M_x + T * B_x));
    
    tau_star.push_back(tao_star_tem);

    std::cout << "tao_star: " << tao_star_tem << endl;
    double tau_tem = proj(tau_star[frame]);
    tau.push_back(tau_tem);

    return tau[frame];
}

void controller::refresh(const double & T)
{
    switch (mode_)
    {
    case ControlMode::NORMAL:
        refreshNormal(T);
        break;
    case ControlMode::SFC:
        refreshSFC(T);
        break;
    case ControlMode::SMC:
        refreshSMC(T);
        break;
    default:
        cout << "please choose a control mode for the robot!" << endl;
        break;
    }
}

void controller::refreshNormal(const double & T)
{
    double K_hat = K + B / T + L * T;
    double q_x_tem = q_x[frame - 1] + T * u_x_star[frame];
    q_x.push_back(q_x_tem);

    double q_x_hat_tem = (q_x[frame] - q_x[frame - 1]) / T;
    q_x_hat.push_back(q_x_hat_tem);

    double a_tem = a[frame - 1]  + T * (q_x[frame] - q_s[frame]);
    a.push_back(a_tem);

}

void controller::refreshSFC(const double & T)
{

}

void controller::refreshSMC(const double & T)
{
    double K_hat = K + B / T + L * T;
    double q_x_tem = q_s_star[frame] + tau[frame] / (K_hat + M / (T * T));
    q_x.push_back(q_x_tem);

    double q_x_hat_tem = (q_x[frame] - q_x[frame - 1]) / T;
    q_x_hat.push_back(q_x_hat_tem);
    std::cout << "q_x_hat_tem: " << q_x_hat_tem << std::endl;
    
    double a_tem = a[frame - 1]  + T * (q_x[frame] - q_s[frame]);
    a.push_back(a_tem);

    

    frame++;
}


double controller::proj(double tau_star_input)
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

double controller::proj_Q(double u_x_star)
{
    if (u_x_star < -Q_max)
    {
        return -Q_max;
    }
    else if (u_x_star > Q_max)
    {
        return Q_max;
    }
    else{
        return u_x_star;
    }
}


void controller::plot_tau()
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


void controller::plot_q()
{
    plt::figure_size(1200, 780);
    plt::named_plot("computed_qx", q_x,  "r--");
    plt::named_plot("q_s", q_s);
    plt::legend();
    plt::title("q");
    // plt::xlim(0, 100);
    plt::save("result_qs.pdf");
}

void controller::plot_q_hat()
{
    plt::figure_size(1200, 780);
    plt::named_plot("q_x_hat", q_x_hat,  "r--");
    // plt::named_plot("q_s_hat", q_s_hat);
    plt::legend();
    plt::title("q_hat");
    // plt::xlim(0, 100);
    plt::save("q_hat.pdf");
}



controller::~controller()
{

}
