#pragma once

#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <string>
#include <thread>

#include <ActuatorConfigClientRpc.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <KDetailedException.h>
#include <RouterClient.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#include "utilities.h"

namespace k_api = Kinova::Api;

class KortexRobotInterface
{
  public:
    KortexRobotInterface(int argc, char** argv);
    ~KortexRobotInterface();

    bool connect();
    bool moveToExperimentStart();
    void disconnect();

    k_api::Base::BaseClient* base();
    k_api::BaseCyclic::BaseCyclicClient* baseCyclic();
    k_api::ActuatorConfig::ActuatorConfigClient* actuatorConfig();

  private:
    static constexpr int PORT = 10000;
    static constexpr int PORT_REAL_TIME = 10001;

    const std::chrono::seconds TIMEOUT_PROMISE_DURATION = std::chrono::seconds{10};

    ExampleArgs parsed_args_;

    k_api::TransportClientTcp* transport_;
    k_api::RouterClient* router_;

    k_api::TransportClientUdp* transport_real_time_;
    k_api::RouterClient* router_real_time_;

    k_api::SessionManager* session_manager_;
    k_api::SessionManager* session_manager_real_time_;

    k_api::Base::BaseClient* base_;
    k_api::BaseCyclic::BaseCyclicClient* base_cyclic_;
    k_api::ActuatorConfig::ActuatorConfigClient* actuator_config_;

    std::function<void(k_api::Base::ActionNotification)> createActionListener(std::promise<k_api::Base::ActionEvent>& finish_promise);

    void createSessions();
    void createServices();
};
