#pragma once

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include <ActuatorConfigClientRpc.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <KDetailedException.h>

#include <ConfigLoader.h>
#include <ControllerFactory.h>
#include <DataLogger.h>

#include "sriCommDefine.h"
#include "sriCommManager.h"

namespace k_api = Kinova::Api;

class SafeAdmittanceExperiment
{
  public:
    SafeAdmittanceExperiment(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                             k_api::ActuatorConfig::ActuatorConfigClient* actuator_config);

    bool run();

  private:
    k_api::Base::BaseClient* base_;
    k_api::BaseCyclic::BaseCyclicClient* base_cyclic_;
    k_api::ActuatorConfig::ActuatorConfigClient* actuator_config_;

    std::string config_file_;
    ConfigLoader loader_;
    DataLogger logger_;
    ControllerFactory controller_factory_;
    std::unique_ptr<BaseController> control_;

    CSRICommManager commManager_;
    bool force_sensor_connected_;

    k_api::BaseCyclic::Feedback base_feedback_;
    k_api::BaseCyclic::Command base_command_;

    unsigned int actuator_count_;

    Eigen::Vector2d f_init_;
    Eigen::Vector2d f_filtered_;
    bool force_filter_initialized_;

    bool angle_initialized_;
    double q1_last_unwrapped_deg_;
    double q4_last_unwrapped_deg_;

  private:
    bool connectForceSensor();
    bool initializeRobot();
    bool enableTorqueMode();
    bool runControlLoop();
    void plotAndSave();
    void stopForceSensor();

    Eigen::Vector2d estimateForceBias(const Eigen::Vector3d& eulerAngle);
    Eigen::Vector2d readForceInput(const Eigen::Vector3d& eulerAngle);
    Eigen::Vector2d filterForce(const Eigen::Vector2d& f_raw);

    Eigen::Vector2d readJointPositionInput();

    double normalizeToPiDeg(double angle_deg) const;
    double unwrapAngleDeg(double current_deg, double last_unwrapped_deg) const;

    int64_t getTickUs() const;
};
