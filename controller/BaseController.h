#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "ConfigLoader.h"
#include "DataLogger.h"
#include <kinematics_model.h>

using Vector2dTrajectory = std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;

enum class ControlTarget
{
    ForceControl,
    PositionControl
};

class BaseController
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit BaseController(const std::string& config_file, DataLogger& logger);
    virtual ~BaseController() = default;

    Eigen::Vector2d getTorque(Eigen::Vector2d& f_ext_from_sensor, Eigen::Vector2d& q_from_sensor);

    void refresh();

    void printParams() const;

    double getDt() const
    {
        return dt;
    }
    double getSimTime() const
    {
        return SimTime;
    }
    const std::string& getControllerName() const
    {
        return controller_name;
    }
    const std::string& getControlTarget() const
    {
        return control_target;
    }

  protected:
    virtual Eigen::Vector2d getTorqueOnForceControl(Eigen::Vector2d& f_ext_from_sensor, Eigen::Vector2d& q_from_sensor) = 0;
    virtual Eigen::Vector2d getTorqueOnPositionControl(Eigen::Vector2d& f_ext_from_sensor, Eigen::Vector2d& q_from_sensor) = 0;
    virtual void refreshOnForceControl() {}
    virtual void refreshOnPositionControl() {}

    void loadConfig(const std::string& config_file);
    void initializeState();
    void parseTarget();
    bool isImpactExperimentTarget() const;

    void generateJointTrajectory();
    void generateOriginalJointTrajectory();
    void generateImpactExperimentTrajectory();
    void plotGeneratedTrajectory();

    void updateMeasuredState(const Eigen::Vector2d& f_ext, const Eigen::Vector2d& q_s_measured);
    void updateCartesianState();
    void logCurrentFrame();

    Eigen::Vector2d proj(const Eigen::Vector2d& input) const;
    Eigen::Vector2d proj_Q(const Eigen::Vector2d& input) const;

    Eigen::Vector2d getDesiredForceJoint() const;

  protected:
    DataLogger& logger_;
    KortexKinematics model;
    ControlTarget target_ = ControlTarget::ForceControl;

    Eigen::Matrix2d M_x = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d B_x = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d K_x = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d B_x_sfc = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d K = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d B = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d L = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d M = Eigen::Matrix2d::Identity();

    Eigen::Vector2d F_max = Eigen::Vector2d::Zero();
    Eigen::Vector2d Q_max = Eigen::Vector2d::Zero();
    Eigen::Vector2d f_d_0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d f_d_1 = Eigen::Vector2d::Zero();
    Eigen::Vector2d f_d_2 = Eigen::Vector2d::Zero();

    double dt = 0.002;
    double SimTime = 6.0;
    double omega = 0.0;
    double sin_T = 6.0;
    double amplitude = 0.0;
    double x_amplitude = 0.0;

    std::string controller_name = "normal";
    std::string control_target = "position";

    Eigen::Vector2d init_angle = Eigen::Vector2d::Zero();
    Eigen::Vector2d init_pos = Eigen::Vector2d::Zero();

    Eigen::Vector2d impact_init_angle = Eigen::Vector2d::Zero();
    Eigen::Vector2d impact_init_pos = Eigen::Vector2d::Zero();
    Eigen::Vector2d impact_mid_pos = Eigen::Vector2d::Zero();
    Eigen::Vector2d impact_end_pos = Eigen::Vector2d::Zero();

    int32_t frame = 1;

    Eigen::Vector2d f = Eigen::Vector2d::Zero();
    Eigen::Vector2d q_s = Eigen::Vector2d::Zero();
    Eigen::Vector2d q_s_last = Eigen::Vector2d::Zero();
    Eigen::Vector2d q_s_hat = Eigen::Vector2d::Zero();
    Eigen::Vector2d q_s_hat_last = Eigen::Vector2d::Zero();

    Eigen::Vector2d q_x = Eigen::Vector2d::Zero();
    Eigen::Vector2d q_x_last = Eigen::Vector2d::Zero();
    Eigen::Vector2d q_x_hat = Eigen::Vector2d::Zero();
    Eigen::Vector2d q_x_hat_last = Eigen::Vector2d::Zero();
    Eigen::Vector2d q_x_star = Eigen::Vector2d::Zero();
    Eigen::Vector2d q_s_star = Eigen::Vector2d::Zero();

    Eigen::Vector2d q0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d q0_dot = Eigen::Vector2d::Zero();
    Eigen::Vector2d q0_ddot = Eigen::Vector2d::Zero();

    Eigen::Vector2d tau = Eigen::Vector2d::Zero();
    Eigen::Vector2d tau_star = Eigen::Vector2d::Zero();
    Eigen::Vector2d tau_ext = Eigen::Vector2d::Zero();

    Eigen::Vector2d u_x = Eigen::Vector2d::Zero();
    Eigen::Vector2d u_x_star = Eigen::Vector2d::Zero();

    Eigen::Vector2d phi_a = Eigen::Vector2d::Zero();
    Eigen::Vector2d phi_a_last = Eigen::Vector2d::Zero();
    Eigen::Vector2d phi_b = Eigen::Vector2d::Zero();
    Eigen::Vector2d phi_b_last = Eigen::Vector2d::Zero();

    Eigen::Vector2d a = Eigen::Vector2d::Zero();
    Eigen::Vector2d a_last = Eigen::Vector2d::Zero();
    Eigen::Vector2d integral_a = Eigen::Vector2d::Zero();
    Eigen::Vector2d integral_a_last = Eigen::Vector2d::Zero();

    Eigen::Vector2d e_q = Eigen::Vector2d::Zero();
    Eigen::Vector2d e_q_last = Eigen::Vector2d::Zero();
    Eigen::Vector2d e_r = Eigen::Vector2d::Zero();
    Eigen::Vector2d e_r_last = Eigen::Vector2d::Zero();

    Eigen::Vector2d X0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d X0_dot = Eigen::Vector2d::Zero();
    Eigen::Vector2d X0_ddot = Eigen::Vector2d::Zero();
    Eigen::Vector2d X = Eigen::Vector2d::Zero();
    Eigen::Vector2d X_d = Eigen::Vector2d::Zero();

    Eigen::Matrix2d jacobian_now = Eigen::Matrix2d::Zero();
    Eigen::Matrix2d jacobian_last = Eigen::Matrix2d::Zero();

    Vector2dTrajectory q0_traj;
    Vector2dTrajectory q0_dot_traj;
    Vector2dTrajectory q0_ddot_traj;
    Vector2dTrajectory X0_traj;
    Vector2dTrajectory X0_dot_traj;
    Vector2dTrajectory X0_ddot_traj;
};