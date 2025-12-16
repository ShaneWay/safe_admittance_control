#include <BaseController.h>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

BaseController::BaseController(BaseParams& params, ControlState& state): params_(params), state_(state),
M_x(params.M_x),
B_x(params.B_x),
K_x(params.K_x),
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
jacobian_now(state.jacobian_now)
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