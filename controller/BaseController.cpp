#include "BaseController.h"
#include "matplotlibcpp.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <unistd.h>

namespace plt = matplotlibcpp;

BaseController::BaseController(const std::string& config_file, DataLogger& logger) : logger_(logger)
{
    loadConfig(config_file);
    initializeState();
    parseTarget();

    printParams();

    if (target_ == ControlTarget::PositionControl) {
        std::cout << "generate===============================" << std::endl;
        generateJointTrajectory();
        plotGeneratedTrajectory();
        std::cout << "generate===end============================" << std::endl;
    }
}

void BaseController::loadConfig(const std::string& config_file)
{
    ConfigLoader loader(config_file);
    YAML::Node ctrl = loader.getNode("controller");
    YAML::Node robot = loader.getNode("robot");

    M_x = loader.getMatrix2d(ctrl["M_x"]);
    B_x = loader.getMatrix2d(ctrl["B_x"]);
    K_x = loader.getMatrix2d(ctrl["K_x"]);
    B_x_sfc = loader.getMatrix2d(ctrl["B_x_sfc"]);

    K = loader.getMatrix2d(ctrl["K"]);
    B = loader.getMatrix2d(ctrl["B"]);
    L = loader.getMatrix2d(ctrl["L"]);
    M = loader.getMatrix2d(ctrl["M"]);

    F_max = loader.getVector2d(ctrl["F_max"]);
    Q_max = loader.getVector2d(ctrl["Q_max"]);
    f_d_0 = loader.getVector2d(ctrl["f_d_0"]);
    f_d_1 = loader.getVector2d(ctrl["f_d_1"]);
    f_d_2 = loader.getVector2d(ctrl["f_d_2"]);

    dt = ctrl["dt"].as<double>();
    SimTime = ctrl["SimTime"].as<double>();
    controller_name = ctrl["controller_name"].as<std::string>();
    control_target = ctrl["control_target"].as<std::string>();

    init_angle = loader.getVector2d(robot["init_angle"]);
    init_pos = loader.getVector2d(robot["init_pos"]);
    sin_T = robot["sin_T"].as<double>();
    amplitude = robot["amplitude"].as<double>();
    x_amplitude = robot["x_amplitude"].as<double>();

    impact_init_angle = loader.getVector2d(robot["impact_init_angle"]);
    impact_init_pos = loader.getVector2d(robot["impact_init_pos"]);
    if (robot["impact_mid_pos"]) {
        impact_mid_pos = loader.getVector2d(robot["impact_mid_pos"]);
    } else {
        impact_mid_pos << -0.107, 0.442;
    }

    if (robot["impact_end_pos"]) {
        impact_end_pos = loader.getVector2d(robot["impact_end_pos"]);
    } else {
        impact_end_pos << 0.034, 0.338;
    }

    impact_init_angle[0] = impact_init_angle[0] * M_PI / 180.0;
    impact_init_angle[1] = impact_init_angle[1] * M_PI / 180.0;

    init_angle[0] = init_angle[0] * M_PI / 180.0;
    init_angle[1] = init_angle[1] * M_PI / 180.0;
    omega = 2.0 * M_PI / sin_T;
}

void BaseController::initializeState()
{
    frame = 1;

    f.setZero();

    Eigen::Vector2d selected_init_angle = init_angle;
    Eigen::Vector2d selected_init_pos = init_pos;

    if (isImpactExperimentTarget()) {
        selected_init_angle = impact_init_angle;
        selected_init_pos = impact_init_pos;
    }

    q_s = selected_init_angle;
    q_s_last = selected_init_angle;
    q_s_hat.setZero();
    q_s_hat_last.setZero();

    q_x = selected_init_angle;
    q_x_last = selected_init_angle;
    q_x_hat.setZero();
    q_x_hat_last.setZero();
    q_x_star.setZero();
    q_s_star.setZero();

    q0 = selected_init_angle;
    q0_dot.setZero();
    q0_ddot.setZero();

    tau.setZero();
    tau_star.setZero();
    tau_ext.setZero();

    u_x.setZero();
    u_x_star.setZero();

    phi_a.setZero();
    phi_a_last.setZero();
    phi_b.setZero();
    phi_b_last.setZero();

    a.setZero();
    a_last.setZero();

    integral_a.setZero();
    integral_a_last.setZero();

    e_q.setZero();
    e_q_last.setZero();
    e_r.setZero();
    e_r_last.setZero();

    model.getFowardKinematicsTwoDOF(selected_init_angle, X);

    X0 = X;
    X0_dot.setZero();
    X0_ddot.setZero();

    X_d = X;

    jacobian_now.setZero();
    jacobian_last.setZero();

    const size_t reserve_size = static_cast<size_t>(SimTime / dt) + 10;

    q0_traj.clear();
    q0_dot_traj.clear();
    q0_ddot_traj.clear();
    X0_traj.clear();
    X0_dot_traj.clear();
    X0_ddot_traj.clear();

    q0_traj.reserve(reserve_size);
    q0_dot_traj.reserve(reserve_size);
    q0_ddot_traj.reserve(reserve_size);
    X0_traj.reserve(reserve_size);
    X0_dot_traj.reserve(reserve_size);
    X0_ddot_traj.reserve(reserve_size);

    q0_traj.push_back(q0);
    q0_dot_traj.push_back(q0_dot);
    q0_ddot_traj.push_back(q0_ddot);
    X0_traj.push_back(X0);
    X0_dot_traj.push_back(X0_dot);
    X0_ddot_traj.push_back(X0_ddot);
}

void BaseController::printParams() const
{
    std::cout << "================ controller params ================" << std::endl;

    std::cout << "controller_name: " << controller_name << std::endl;
    std::cout << "control_target:  " << control_target << std::endl;
    std::cout << "dt:              " << dt << std::endl;
    std::cout << "SimTime:         " << SimTime << std::endl;

    std::cout << "init_angle:      " << init_angle.transpose() << " rad" << std::endl;
    std::cout << "init_pos:        " << init_pos.transpose() << " m" << std::endl;

    std::cout << "M_x:\n" << M_x << std::endl;
    std::cout << "B_x:\n" << B_x << std::endl;
    std::cout << "K_x:\n" << K_x << std::endl;
    std::cout << "B_x_sfc:\n" << B_x_sfc << std::endl;

    std::cout << "M:\n" << M << std::endl;
    std::cout << "B:\n" << B << std::endl;
    std::cout << "K:\n" << K << std::endl;
    std::cout << "L:\n" << L << std::endl;

    std::cout << "F_max:           " << F_max.transpose() << std::endl;
    std::cout << "Q_max:           " << Q_max.transpose() << std::endl;

    std::cout << "f_d_0:           " << f_d_0.transpose() << std::endl;
    std::cout << "f_d_1:           " << f_d_1.transpose() << std::endl;
    std::cout << "f_d_2:           " << f_d_2.transpose() << std::endl;

    std::cout << "sin_T:           " << sin_T << std::endl;
    std::cout << "omega:           " << omega << std::endl;
    std::cout << "amplitude:       " << amplitude << std::endl;
    std::cout << "x_amplitude:     " << x_amplitude << std::endl;

    std::cout << "===================================================" << std::endl;
}

void BaseController::parseTarget()
{
    if (control_target == "position" || isImpactExperimentTarget()) {
        target_ = ControlTarget::PositionControl;
    } else {
        target_ = ControlTarget::ForceControl;
    }
}
bool BaseController::isImpactExperimentTarget() const
{
    return control_target == "impact";
}

Eigen::Vector2d BaseController::getTorque(Eigen::Vector2d& f_ext_from_sensor, Eigen::Vector2d& q_from_sensor)
{
    updateMeasuredState(f_ext_from_sensor, q_from_sensor);

    if (target_ == ControlTarget::PositionControl) {
        tau = getTorqueOnPositionControl(f_ext_from_sensor, q_from_sensor);
    } else {
        tau = getTorqueOnForceControl(f_ext_from_sensor, q_from_sensor);
    }

    updateCartesianState();
    logCurrentFrame();

    return tau;
}

void BaseController::refresh()
{
    q_s_last = q_s;
    q_x_last = q_x;
    q_x_hat_last = q_x_hat;
    q_s_hat_last = q_s_hat;

    a_last = a;
    integral_a_last = integral_a;

    phi_a_last = phi_a;
    phi_b_last = phi_b;

    e_q_last = e_q;
    e_r_last = e_r;

    jacobian_last = jacobian_now;

    if (target_ == ControlTarget::PositionControl) {
        refreshOnPositionControl();
    } else {
        refreshOnForceControl();
    }

    frame++;
}

void BaseController::updateMeasuredState(const Eigen::Vector2d& f_ext, const Eigen::Vector2d& q_s_measured)
{
    f = f_ext;
    q_s = q_s_measured;
    q_s_hat = (q_s - q_s_last) / dt;

    model.getJacobianMatrixTwoDOF(q_s, jacobian_now);
    tau_ext = jacobian_now.transpose() * f;
}

void BaseController::updateCartesianState()
{
    model.getFowardKinematicsTwoDOF(q_s, X);
    model.getFowardKinematicsTwoDOF(q_x, X_d);

    if (!X0_traj.empty()) {
        const size_t idx = std::min<size_t>(static_cast<size_t>(frame), X0_traj.size() - 1);

        X0 = X0_traj[idx];
        X0_dot = X0_dot_traj[idx];
        X0_ddot = X0_ddot_traj[idx];

        q0 = q0_traj[idx];
        q0_dot = q0_dot_traj[idx];
        q0_ddot = q0_ddot_traj[idx];
    }
}

void BaseController::logCurrentFrame()
{
    ControlLogSample s;

    s.time = frame * dt;
    s.frame = frame;

    s.f_ext = f;

    s.q_s = q_s;
    s.q_s_hat = q_s_hat;

    s.q_x = q_x;
    s.q_x_hat = q_x_hat;

    s.q0 = q0;

    s.tau = tau;
    s.tau_star = tau_star;
    s.tau_ext = tau_ext;

    s.X0 = X0;
    s.X = X;
    s.X_d = X_d;

    logger_.log(s);
}

Eigen::Vector2d BaseController::proj(const Eigen::Vector2d& input) const
{
    Eigen::Vector2d out = input;

    for (int i = 0; i < 2; ++i) {
        if (out[i] > F_max[i]) {
            out[i] = F_max[i];
        }

        if (out[i] < -F_max[i]) {
            out[i] = -F_max[i];
        }
    }

    return out;
}

Eigen::Vector2d BaseController::proj_Q(const Eigen::Vector2d& input) const
{
    Eigen::Vector2d out = input;

    for (int i = 0; i < 2; ++i) {
        if (out[i] > Q_max[i]) {
            out[i] = Q_max[i];
        }

        if (out[i] < -Q_max[i]) {
            out[i] = -Q_max[i];
        }
    }

    return out;
}

Eigen::Vector2d BaseController::getDesiredForceJoint() const
{
    if (frame <= 4000) {
        return jacobian_now.transpose() * f_d_0;
    }

    if (frame <= 8000) {
        return jacobian_now.transpose() * f_d_1;
    }

    return jacobian_now.transpose() * f_d_2;
}

void BaseController::generateJointTrajectory()
{
    if (isImpactExperimentTarget()) {
        std::cout << "Generate impact experiment trajectory." << std::endl;
        generateImpactExperimentTrajectory();
    } else {
        std::cout << "Generate original trajectory." << std::endl;
        generateOriginalJointTrajectory();
    }
}

void BaseController::generateOriginalJointTrajectory()
{
    const int count = static_cast<int>(SimTime / dt);

    q0_traj.clear();
    q0_dot_traj.clear();
    q0_ddot_traj.clear();
    X0_traj.clear();
    X0_dot_traj.clear();
    X0_ddot_traj.clear();

    q0_traj.reserve(count + 1);
    q0_dot_traj.reserve(count + 1);
    q0_ddot_traj.reserve(count + 1);
    X0_traj.reserve(count + 1);
    X0_dot_traj.reserve(count + 1);
    X0_ddot_traj.reserve(count + 1);

    Eigen::Vector2d x_last = init_pos;
    Eigen::Vector2d x_dot_last = Eigen::Vector2d::Zero();

    for (int i = 0; i <= count; ++i) {
        const double t = i * dt;

        Eigen::Vector2d q = init_angle;
        Eigen::Vector2d q_dot = Eigen::Vector2d::Zero();
        Eigen::Vector2d q_ddot = Eigen::Vector2d::Zero();

        q[0] = init_angle[0] + amplitude * std::sin(omega * t);
        q_dot[0] = amplitude * omega * std::cos(omega * t);
        q_ddot[0] = -amplitude * omega * omega * std::sin(omega * t);

        Eigen::Vector2d x;
        model.getFowardKinematicsTwoDOF(q, x);

        Eigen::Vector2d x_dot;
        Eigen::Vector2d x_ddot;

        if (i == 0) {
            x_dot = Eigen::Vector2d::Zero();
        } else {
            x_dot = (x - x_last) / dt;
        }

        if (i <= 1) {
            x_ddot = Eigen::Vector2d::Zero();
        } else {
            x_ddot = (x_dot - x_dot_last) / dt;
        }

        q0_traj.push_back(q);
        q0_dot_traj.push_back(q_dot);
        q0_ddot_traj.push_back(q_ddot);

        X0_traj.push_back(x);
        X0_dot_traj.push_back(x_dot);
        X0_ddot_traj.push_back(x_ddot);

        x_last = x;
        x_dot_last = x_dot;
    }

    if (!q0_traj.empty()) {
        q0 = q0_traj.front();
        q0_dot = q0_dot_traj.front();
        q0_ddot = q0_ddot_traj.front();

        X0 = X0_traj.front();
        X0_dot = X0_dot_traj.front();
        X0_ddot = X0_ddot_traj.front();
    }
}

void BaseController::generateImpactExperimentTrajectory()
{
    sleep(2);

    const int count = static_cast<int>(SimTime / dt);
    Eigen::Vector2d p0;
    model.getFowardKinematicsTwoDOF(impact_init_angle, p0);
    // const Eigen::Vector2d p0(-0.247, 0.299);
    const Eigen::Vector2d p1 = impact_mid_pos;
    const Eigen::Vector2d p2 = impact_end_pos;

    q0_traj.clear();
    q0_dot_traj.clear();
    q0_ddot_traj.clear();
    X0_traj.clear();
    X0_dot_traj.clear();
    X0_ddot_traj.clear();

    q0_traj.reserve(count + 1);
    q0_dot_traj.reserve(count + 1);
    q0_ddot_traj.reserve(count + 1);
    X0_traj.reserve(count + 1);
    X0_dot_traj.reserve(count + 1);
    X0_ddot_traj.reserve(count + 1);

    Eigen::Vector2d q = impact_init_angle;
    Eigen::Vector2d q_dot = Eigen::Vector2d::Zero();
    Eigen::Vector2d q_dot_last = Eigen::Vector2d::Zero();

    Eigen::Vector2d x_last = p0;
    Eigen::Vector2d x_dot_last = Eigen::Vector2d::Zero();

    for (int i = 0; i <= count; ++i) {
        const double r = static_cast<double>(i) / static_cast<double>(count);

        const double s = 10.0 * std::pow(r, 3) - 15.0 * std::pow(r, 4) + 6.0 * std::pow(r, 5);

        const double s_dot = 30.0 * r * r * (1.0 - r) * (1.0 - r) / SimTime;

        const double s_ddot = 60.0 * r * (1.0 - r) * (1.0 - 2.0 * r) / (SimTime * SimTime);

        Eigen::Vector2d x = 2.0 * (s - 0.5) * (s - 1.0) * p0 - 4.0 * s * (s - 1.0) * p1 + 2.0 * s * (s - 0.5) * p2;

        Eigen::Vector2d dx_ds = (4.0 * s - 3.0) * p0 + (-8.0 * s + 4.0) * p1 + (4.0 * s - 1.0) * p2;

        Eigen::Vector2d d2x_ds2 = 4.0 * p0 - 8.0 * p1 + 4.0 * p2;

        Eigen::Vector2d x_dot = dx_ds * s_dot;

        Eigen::Vector2d x_ddot = d2x_ds2 * s_dot * s_dot + dx_ds * s_ddot;

        Eigen::Matrix2d J;
        model.getJacobianMatrixTwoDOF(q, J);

        const double lambda = 1e-4;
        Eigen::Matrix2d J_inv = J.transpose() * (J * J.transpose() + lambda * Eigen::Matrix2d::Identity()).inverse();

        if (i == 0) {
            q_dot.setZero();
        } else {
            q_dot = J_inv * x_dot;
        }

        Eigen::Vector2d q_ddot;

        if (i <= 1) {
            q_ddot.setZero();
        } else {
            q_ddot = (q_dot - q_dot_last) / dt;
        }

        if (i > 0) {
            q = q + q_dot * dt;
        }

        q0_traj.push_back(q);
        q0_dot_traj.push_back(q_dot);
        q0_ddot_traj.push_back(q_ddot);

        X0_traj.push_back(x);
        X0_dot_traj.push_back(x_dot);
        X0_ddot_traj.push_back(x_ddot);

        q_dot_last = q_dot;
    }

    if (!q0_traj.empty()) {
        q0 = q0_traj.front();
        q0_dot = q0_dot_traj.front();
        q0_ddot = q0_ddot_traj.front();

        X0 = X0_traj.front();
        X0_dot = X0_dot_traj.front();
        X0_ddot = X0_ddot_traj.front();
    }

    std::cout << "Impact parabolic Cartesian trajectory generated." << std::endl;
    std::cout << "Start X0: " << X0_traj.front().transpose() << " m" << std::endl;
    std::cout << "End X0:   " << X0_traj.back().transpose() << " m" << std::endl;
    std::cout << "Start q0: " << q0_traj.front().transpose() << " rad" << std::endl;
    std::cout << "End q0:   " << q0_traj.back().transpose() << " rad" << std::endl;
}

void BaseController::plotGeneratedTrajectory()
{
    if (X0_traj.empty()) {
        std::cout << "[plotGeneratedTrajectory] X0_traj is empty, skip plot." << std::endl;
        return;
    }

    std::vector<double> x0_data;
    std::vector<double> y0_data;

    x0_data.reserve(X0_traj.size());
    y0_data.reserve(X0_traj.size());

    for (const auto& x : X0_traj) {
        x0_data.push_back(x[0]);
        y0_data.push_back(x[1]);
    }

    plt::clf();
    plt::figure_size(1200, 780);

    plt::named_plot("X0 trajectory", x0_data, y0_data, "b-");

    // 实际生成轨迹的起点和终点
    const Eigen::Vector2d start_pos = X0_traj.front();
    const Eigen::Vector2d end_pos = X0_traj.back();

    plt::plot(std::vector<double>{start_pos[0]}, std::vector<double>{start_pos[1]}, "go");
    plt::annotate("start", start_pos[0], start_pos[1]);

    plt::plot(std::vector<double>{end_pos[0]}, std::vector<double>{end_pos[1]}, "ro");
    plt::annotate("end", end_pos[0], end_pos[1]);

    if (isImpactExperimentTarget()) {
        std::vector<double> wp_x;
        std::vector<double> wp_y;

        wp_x.push_back(start_pos[0]);
        wp_y.push_back(start_pos[1]);

        wp_x.push_back(impact_mid_pos[0]);
        wp_y.push_back(impact_mid_pos[1]);

        wp_x.push_back(impact_end_pos[0]);
        wp_y.push_back(impact_end_pos[1]);

        plt::named_plot("waypoints", wp_x, wp_y, "ko");

        plt::annotate("mid", impact_mid_pos[0], impact_mid_pos[1]);

        // 如果实际轨迹终点和 impact_end_pos 有细微误差，也把 impact end 标出来
        plt::plot(std::vector<double>{impact_end_pos[0]}, std::vector<double>{impact_end_pos[1]}, "kx");
    }

    plt::xlabel("x (m)");
    plt::ylabel("y (m)");
    plt::title("Generated X0 Trajectory in Cartesian Space");
    plt::legend();
    plt::grid(true);
    plt::axis("equal");

    plt::save("Generated_X0_Trajectory_XY.pdf");
    plt::clf();
    plt::close();

    std::cout << "Generated_X0_Trajectory_XY.pdf saved." << std::endl;
}