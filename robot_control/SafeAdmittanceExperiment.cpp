#include "SafeAdmittanceExperiment.h"

#include <experimental/filesystem>
#include <limits>

namespace fs = std::experimental::filesystem;

SafeAdmittanceExperiment::SafeAdmittanceExperiment(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                                                   k_api::ActuatorConfig::ActuatorConfigClient* actuator_config)
    : base_(base)
    , base_cyclic_(base_cyclic)
    , actuator_config_(actuator_config)
    , config_file_("../controller/controller.yaml")
    , loader_(config_file_)
    , logger_()
    , controller_factory_(config_file_, logger_)
    , force_sensor_connected_(false)
    , actuator_count_(0)
    , f_init_(Eigen::Vector2d::Zero())
    , f_filtered_(Eigen::Vector2d::Zero())
    , force_filter_initialized_(false)
    , q1_last_unwrapped_deg_(0.0)
    , q4_last_unwrapped_deg_(0.0)
{
    const std::string controller_type = loader_.getNode("controller")["controller_name"].as<std::string>();
    control_ = controller_factory_.create(controller_type);

    const size_t reserve_size = static_cast<size_t>(control_->getSimTime() / control_->getDt()) + 10;
    logger_.reserve(reserve_size);
    logger_.loadWaypointsFromYaml(config_file_);
}

int64_t SafeAdmittanceExperiment::getTickUs() const
{
    timespec t{};
    clock_gettime(CLOCK_MONOTONIC, &t);
    return static_cast<int64_t>(t.tv_sec * 1000000LLU + t.tv_nsec / 1000);
}

double SafeAdmittanceExperiment::unwrapAngleDeg(double current_deg, double last_unwrapped_deg) const
{
    double candidate = current_deg;

    while (current_deg - last_unwrapped_deg > 180.0) candidate -= 360.0;
    while (current_deg - last_unwrapped_deg < -180.0) candidate += 360.0;

    return candidate;
}

Eigen::Vector2d SafeAdmittanceExperiment::readJointPositionInput()
{
    const double q1_raw_deg = base_feedback_.actuators(0).position();
    const double q4_raw_deg = base_feedback_.actuators(3).position();

    double q1_unwrapped_deg = unwrapAngleDeg(q1_raw_deg, q1_last_unwrapped_deg_);
    double q4_unwrapped_deg = unwrapAngleDeg(q4_raw_deg, q4_last_unwrapped_deg_);

    q1_last_unwrapped_deg_ = q1_unwrapped_deg;
    q4_last_unwrapped_deg_ = q4_unwrapped_deg;

    Eigen::Vector2d q_input;
    q_input << q1_unwrapped_deg * M_PI / 180.0, q4_unwrapped_deg * M_PI / 180.0;

    return q_input;
}

bool SafeAdmittanceExperiment::connectForceSensor()
{
    std::cout << "##################################\nSRI TCP Client connect.\n##################################\n" << std::endl;

    if (commManager_.Init() && commManager_.Run()) {
        force_sensor_connected_ = true;
        std::cout << "[OK] Force sensor connected." << std::endl;
    } else {
        std::cout << "[WARNING] Force sensor connection failed. Use zero external force." << std::endl;
    }

    return true;
}

Eigen::Vector2d SafeAdmittanceExperiment::estimateForceBias(const Eigen::Vector3d& eulerAngle)
{
    if (!force_sensor_connected_) return Eigen::Vector2d::Zero();

    constexpr int sample_count = 200;
    constexpr int sleep_ms = 5;

    Eigen::Vector2d bias = Eigen::Vector2d::Zero();

    std::cout << "Estimating force bias..." << std::endl;

    for (int i = 0; i < sample_count; ++i) {
        bias += commManager_.getBaseForce(eulerAngle);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }

    bias /= static_cast<double>(sample_count);

    std::cout << "Initial force bias: " << bias.transpose() << std::endl;

    return bias;
}

Eigen::Vector2d SafeAdmittanceExperiment::filterForce(const Eigen::Vector2d& f_raw)
{
    constexpr double alpha = 0.90;
    constexpr double deadband = 0.3;

    if (!force_filter_initialized_) {
        f_filtered_ = f_raw;
        force_filter_initialized_ = true;
    } else {
        f_filtered_ = alpha * f_filtered_ + (1.0 - alpha) * f_raw;
    }

    Eigen::Vector2d out = f_filtered_;

    for (int i = 0; i < 2; ++i) {
        if (std::abs(out[i]) < deadband) {
            out[i] = 0.0;
        } else {
            out[i] += (out[i] > 0.0) ? -deadband : deadband;
        }
    }

    return out;
}

Eigen::Vector2d SafeAdmittanceExperiment::readForceInput(const Eigen::Vector3d& eulerAngle)
{
    if (!force_sensor_connected_) return Eigen::Vector2d::Zero();

    const Eigen::Vector2d f_raw = commManager_.getBaseForce(eulerAngle) - f_init_;
    return filterForce(f_raw);
}

bool SafeAdmittanceExperiment::initializeRobot()
{
    actuator_count_ = base_->GetActuatorCount().count();

    auto servoing_mode = k_api::Base::ServoingModeInformation();
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    base_->SetServoingMode(servoing_mode);

    base_feedback_ = base_cyclic_->RefreshFeedback();

    base_command_.clear_actuators();
    for (unsigned int i = 0; i < actuator_count_; ++i) {
        base_command_.add_actuators()->set_position(base_feedback_.actuators(i).position());
    }

    base_command_.set_frame_id(0);
    base_feedback_ = base_cyclic_->Refresh(base_command_);

    std::cout << "Robot initialized in low level servoing mode." << std::endl;
    return true;
}

bool SafeAdmittanceExperiment::enableTorqueMode()
{
    auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
    control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);

    actuator_config_->SetControlMode(control_mode_message, 1);
    actuator_config_->SetControlMode(control_mode_message, 4);

    std::cout << "Joint 1 and joint 4 are set to torque mode." << std::endl;
    return true;
}

bool SafeAdmittanceExperiment::runControlLoop()
{
    const int sim_count = static_cast<int>(control_->getSimTime() / control_->getDt());
    const int64_t period_us = static_cast<int64_t>(control_->getDt() * 1000000.0);

    int timer_count = 0;
    int64_t begin = 0;
    double aver_time = 0.0;

    Eigen::Vector2d f_input = Eigen::Vector2d::Zero();
    Eigen::Vector2d q_input = Eigen::Vector2d::Zero();
    Eigen::Vector2d tau = Eigen::Vector2d::Zero();
    Eigen::Vector3d euler = Eigen::Vector3d::Zero();

    std::cout << "Running torque control for " << control_->getSimTime() << " seconds." << std::endl;

    while (timer_count < sim_count) {
        const int64_t now = getTickUs();
        if (now - begin <= period_us) continue;

        begin = getTickUs();

        base_command_.mutable_actuators(0)->set_position(base_feedback_.actuators(0).position());
        base_command_.mutable_actuators(3)->set_position(base_feedback_.actuators(3).position());

        q_input = readJointPositionInput();

        euler[0] = base_feedback_.base().tool_pose_theta_x() * M_PI / 180.0;
        euler[1] = base_feedback_.base().tool_pose_theta_y() * M_PI / 180.0;
        euler[2] = base_feedback_.base().tool_pose_theta_z() * M_PI / 180.0;

        f_input = readForceInput(euler);
        tau = control_->getTorque(f_input, q_input);

        std::cout << "real tau: " << tau.transpose() << std::endl;
        std::cout << "==========================================!" << std::endl;

        base_command_.mutable_actuators(0)->set_torque_joint(tau[0]);
        base_command_.mutable_actuators(3)->set_torque_joint(tau[1]);

        base_command_.set_frame_id(base_command_.frame_id() + 1);
        if (base_command_.frame_id() > 65535) base_command_.set_frame_id(0);

        for (unsigned int i = 0; i < actuator_count_; ++i) {
            base_command_.mutable_actuators(i)->set_command_id(base_command_.frame_id());
        }

        try {
            base_feedback_ = base_cyclic_->Refresh(base_command_, 0);
        } catch (k_api::KDetailedException& ex) {
            std::cout << "Kortex exception: " << ex.what() << std::endl;
            std::cout << "Error sub-code: "
                      << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())) << std::endl;
        } catch (std::runtime_error& ex) {
            std::cout << "runtime error: " << ex.what() << std::endl;
        } catch (...) {
            std::cout << "Unknown error." << std::endl;
        }

        /*
         * DataLogger is called inside BaseController::getTorque().
         * Therefore refresh() must be called after command calculation/logging.
         */
        control_->refresh();
        ++timer_count;

        const int64_t last = getTickUs();
        aver_time += static_cast<double>(last - begin) / static_cast<double>(sim_count);

        std::cout << "current loop time: " << last - begin << " us" << std::endl;
        std::cout << "accumulated average time: " << aver_time << " us" << std::endl;
    }

    return true;
}

void SafeAdmittanceExperiment::plotAndSave()
{
    std::cout << "##################################\nplot data\n##################################\n" << std::endl;

    const std::string save_dir = "data/" + control_->getControllerName() + "_" + control_->getControlTarget();
    fs::create_directories(save_dir);

    logger_.saveCsv(save_dir + "/experiment_log.csv");
    logger_.plotAll(save_dir + "/plots");

    std::cout << "[DataLogger] plot and save finished" << std::endl;
}

void SafeAdmittanceExperiment::stopForceSensor()
{
    if (force_sensor_connected_) {
        commManager_.Stop();
        force_sensor_connected_ = false;
    }
}

bool SafeAdmittanceExperiment::run()
{
    try {
        connectForceSensor();
        initializeRobot();

        Eigen::Vector3d euler = Eigen::Vector3d::Zero();
        euler[0] = base_feedback_.base().tool_pose_theta_x() * M_PI / 180.0;
        euler[1] = base_feedback_.base().tool_pose_theta_y() * M_PI / 180.0;
        euler[2] = base_feedback_.base().tool_pose_theta_z() * M_PI / 180.0;

        if (force_sensor_connected_) {
            commManager_.SendGODCommand("GSD", "");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            f_init_ = estimateForceBias(euler);
        }

        std::cout << "\n==========================================" << std::endl;
        std::cout << "Robot is ready for torque control." << std::endl;
        std::cout << "Press ENTER to start the control loop..." << std::endl;
        std::cout << "==========================================\n" << std::endl;
        std::cin.get();

        enableTorqueMode();
        runControlLoop();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        plotAndSave();
        stopForceSensor();

        std::cout << "Torque control example clean exit." << std::endl;
        return true;
    } catch (k_api::KDetailedException& ex) {
        std::cout << "API error: " << ex.what() << std::endl;
    } catch (std::runtime_error& ex) {
        std::cout << "Error: " << ex.what() << std::endl;
    } catch (...) {
        std::cout << "Unknown error in SafeAdmittanceExperiment." << std::endl;
    }

    stopForceSensor();
    return false;
}
