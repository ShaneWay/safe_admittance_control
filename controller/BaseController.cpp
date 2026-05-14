#include <BaseController.h>
#include "matplotlibcpp.h"

#include <algorithm>
#include <cmath>

namespace plt = matplotlibcpp;

namespace {

Eigen::Vector2d catmullRomPosition(const Eigen::Vector2d& p0,
                                   const Eigen::Vector2d& p1,
                                   const Eigen::Vector2d& p2,
                                   const Eigen::Vector2d& p3,
                                   double u)
{
    const double u2 = u * u;
    const double u3 = u2 * u;

    return 0.5 * ((2.0 * p1)
        + (-p0 + p2) * u
        + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * u2
        + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * u3);
}

Eigen::Vector2d catmullRomVelocity(const Eigen::Vector2d& p0,
                                   const Eigen::Vector2d& p1,
                                   const Eigen::Vector2d& p2,
                                   const Eigen::Vector2d& p3,
                                   double u,
                                   double segment_time)
{
    const double u2 = u * u;

    Eigen::Vector2d dPdu = 0.5 * ((-p0 + p2)
        + 2.0 * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * u
        + 3.0 * (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * u2);

    return dPdu / segment_time;
}

Eigen::Vector2d catmullRomAcceleration(const Eigen::Vector2d& p0,
                                       const Eigen::Vector2d& p1,
                                       const Eigen::Vector2d& p2,
                                       const Eigen::Vector2d& p3,
                                       double u,
                                       double segment_time)
{
    Eigen::Vector2d d2Pdu2 = 0.5 * (
        2.0 * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3)
        + 6.0 * (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * u);

    return d2Pdu2 / (segment_time * segment_time);
}

}


BaseController::BaseController(BaseParams& params, ControlState& state): params_(params), state_(state),
M_x(params.M_x),
B_x(params.B_x),
K_x(params.K_x),
B_x_sfc(params.B_x_sfc),

K(params.K),
B(params.B),
L(params.L),
M(params.M),
F_max(params.F_max),
Q_max(params.Q_max),
controller_name(params.controller_name),
control_target(params.control_target),
dt(params.dt),
SimTime(params.SimTime),
f_d_0(params.f_d_0),

tau(state.tau),
tau_ext(state.tau_ext),
tau_star(state.tau_star),
u_x(state.u_x),
u_x_star(state.u_x_star),
q_x(state.q_x),
q_x_hat(state.q_x_hat),
q_x_star(state.q_x_star),
phi_b(state.phi_b),
phi_a(state.phi_a),
q_s(state.q_s),
q_s_star(state.q_s_star),
a(state.a),
integral_a(state.integral_a),
frame(state.frame),
f(state.f),
f_d(state.f_d),
q_s_hat(state.q_s_hat),
q0(state.q0),
q0_dot(state.q0_dot),
q0_ddot(state.q0_ddot),
X0(state.X0),
X0_dot(state.X0_dot),
X0_ddot(state.X0_ddot),
X(state.X),
X_hat(state.X_hat),
X_x(state.X_x),
jacobian_last(state.jacobian_last),
jacobian_now(state.jacobian_now),
e_r(state.e_r),
e_q(state.e_q)
{
    parseTarget();

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

void BaseController::parseTarget()
{
    if (control_target == "position" || isImpactExperimentTarget())
    {
        target_ = ControlTarget::PositionControl;
    }
    else
    {
        target_ = ControlTarget::ForceControl;
    }
}

bool BaseController::isImpactExperimentTarget() const
{
    return control_target == "impact";
}

void BaseController::generateJointTrajectory()
{
    if (isImpactExperimentTarget())
    {
        std::cout << "Generate impact experiment trajectory." << std::endl;
        generateImpactExperimentTrajectory();
    }
    else
    {
        std::cout << "Generate original trajectory." << std::endl;
        generateOriginalJointTrajectory();
    }
}

void BaseController::generateImpactExperimentTrajectory()
{
    sleep(2);

    // 轨迹路点顺序：05 -> 04 -> 03 -> 02 -> 01
    // 当前控制器的 X0/q0 是 Eigen::Vector2d，所以这里只生成 XY 轨迹；
    // z 姿态不参与该 2 自由度控制器的轨迹生成。
    std::vector<Eigen::Vector2d> waypoints;
    waypoints.emplace_back(-0.471, 0.470);  // traj_05
    waypoints.emplace_back(-0.354, 0.524);  // traj_04
    waypoints.emplace_back(-0.274, 0.461);  // traj_03
    waypoints.emplace_back(-0.123, 0.463);  // traj_02
    waypoints.emplace_back(-0.040, 0.453);  // traj_01

    const int count_total = static_cast<int>(params_.SimTime / params_.dt) + 1;
    const int segment_num = static_cast<int>(waypoints.size()) - 1;
    const double total_time = params_.SimTime;
    const double segment_time = total_time / static_cast<double>(segment_num);

    for (int time_count = 0; time_count < count_total; ++time_count)
    {
        const double t = time_count * params_.dt;

        Eigen::Vector2d X_0_tem = Eigen::Vector2d::Zero();
        Eigen::Vector2d X_0_dot_tem = Eigen::Vector2d::Zero();
        Eigen::Vector2d X_0_ddot_tem = Eigen::Vector2d::Zero();

        if (t >= total_time)
        {
            X_0_tem = waypoints.back();
        }
        else
        {
            const double s = t / segment_time;
            const int seg = std::min(
                static_cast<int>(std::floor(s)),
                segment_num - 1
            );

            const double u = s - static_cast<double>(seg);

            const Eigen::Vector2d& p0 = waypoints[std::max(seg - 1, 0)];
            const Eigen::Vector2d& p1 = waypoints[seg];
            const Eigen::Vector2d& p2 = waypoints[seg + 1];
            const Eigen::Vector2d& p3 = waypoints[std::min(seg + 2, segment_num)];

            X_0_tem      = catmullRomPosition(p0, p1, p2, p3, u);
            X_0_dot_tem  = catmullRomVelocity(p0, p1, p2, p3, u, segment_time);
            X_0_ddot_tem = catmullRomAcceleration(p0, p1, p2, p3, u, segment_time);
        }

        state_.X0.push_back(X_0_tem);
        state_.X0_dot.push_back(X_0_dot_tem);
        state_.X0_ddot.push_back(X_0_ddot_tem);

        model.getJacobianMatrixTwoDOF(q0[time_count], state_.jacobian_now);

        state_.q0_dot.push_back(
            state_.jacobian_now.inverse() * state_.X0_dot[time_count + 1]
        );

        state_.q0.push_back(
            state_.q0[time_count] + params_.dt * state_.q0_dot[time_count]
        );

        state_.q0_ddot.push_back(
            state_.jacobian_now.inverse() *
            (
                state_.X0_ddot[time_count + 1]
                - (state_.jacobian_now - state_.jacobian_last)
                  / params_.dt * state_.q0_dot[time_count]
            )
        );

        state_.jacobian_last = state_.jacobian_now;
    }
}

void BaseController::generateOriginalJointTrajectory()
{
    sleep(2);
    unsigned int time_count = 0;

    Eigen::Vector2d X_0_tem , X_0_dot_tem, X_0_ddot_tem;
    
    
    double t;
    int count_total = params_.SimTime / params_.dt + 1;
    int flat_count = count_total - params_.sin_T / 4 / params_.dt;
    // std::cout << "count_total" << count_total << std::endl;
    for (time_count = 0; time_count < count_total; time_count++ )
    {

        t = time_count * params_.dt;
        if(time_count < flat_count)
        {
            X_0_tem[0] = params_.x_amplitude * t + params_.init_pos[0];
            X_0_tem[1] = params_.amplitude * sin(params_.omega * t) + params_.init_pos[1];

            X_0_dot_tem[0] = params_.x_amplitude;
            X_0_dot_tem[1] = 1.0 * params_.amplitude * params_.omega * cos(params_.omega * t);

            X_0_ddot_tem[0] = 0.;
            X_0_ddot_tem[1] = - 1.0 * params_.amplitude * params_.omega * params_.omega * sin(params_.omega * t);
        }
        else
        {
            X_0_tem[0] = X0[flat_count - 1][0];
            X_0_tem[1] = X0[flat_count - 1][1];
        }

        // update x_0, x_0_dot, x_0_ddot
        state_.X0.push_back(X_0_tem);
        state_.X0_dot.push_back(X_0_dot_tem);
        state_.X0_ddot.push_back(X_0_ddot_tem);

        model.getJacobianMatrixTwoDOF(q0[time_count], state_.jacobian_now);

        /// make sure that k, k+1, k-1
        /// update q0, q0_dot, q0_ddot
        state_.q0_dot.push_back(state_.jacobian_now.inverse() * state_.X0_dot[time_count + 1]);
        state_.q0.push_back(state_.q0[time_count] + params_.dt * state_.q0_dot[time_count]);
        state_.q0_ddot.push_back(state_.jacobian_now.inverse() * (state_.X0_ddot[time_count + 1] - (state_.jacobian_now - state_.jacobian_last) / params_.dt * state_.q0_dot[time_count]));

        state_.jacobian_last = state_.jacobian_now;
    }
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
              << "f_d_0: " << f_d_0.transpose() << "\n"
              << "SimeTime: " << SimTime << "\n"
              << "controller_name <<: " << controller_name << "\n"
              << "Control_target <<: " << control_target << "\n"
              << "================================\n";
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