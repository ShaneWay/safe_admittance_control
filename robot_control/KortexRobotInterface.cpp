#include "KortexRobotInterface.h"

KortexRobotInterface::KortexRobotInterface(int argc, char** argv)
    : parsed_args_(ParseExampleArguments(argc, argv))
    , transport_(nullptr)
    , router_(nullptr)
    , transport_real_time_(nullptr)
    , router_real_time_(nullptr)
    , session_manager_(nullptr)
    , session_manager_real_time_(nullptr)
    , base_(nullptr)
    , base_cyclic_(nullptr)
    , actuator_config_(nullptr)
{
}

KortexRobotInterface::~KortexRobotInterface()
{
    disconnect();
}

bool KortexRobotInterface::connect()
{
    try {
        auto error_callback = [](k_api::KError err) {
            std::cout << "_________ callback error _________" << err.toString() << std::endl;
        };

        std::cout << "Creating transport objects" << std::endl;
        transport_ = new k_api::TransportClientTcp();
        router_ = new k_api::RouterClient(transport_, error_callback);
        transport_->connect(parsed_args_.ip_address, PORT);

        std::cout << "Creating transport real time objects" << std::endl;
        transport_real_time_ = new k_api::TransportClientUdp();
        router_real_time_ = new k_api::RouterClient(transport_real_time_, error_callback);
        transport_real_time_->connect(parsed_args_.ip_address, PORT_REAL_TIME);

        createSessions();
        createServices();

        std::cout << "Kortex robot connected." << std::endl;
        return true;
    } catch (k_api::KDetailedException& ex) {
        std::cout << "Kortex API error during connect: " << ex.what() << std::endl;
        disconnect();
        return false;
    } catch (std::runtime_error& ex) {
        std::cout << "Runtime error during connect: " << ex.what() << std::endl;
        disconnect();
        return false;
    } catch (...) {
        std::cout << "Unknown error during connect." << std::endl;
        disconnect();
        return false;
    }
}

void KortexRobotInterface::createSessions()
{
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(60000);
    create_session_info.set_connection_inactivity_timeout(2000);

    std::cout << "Creating sessions for communication" << std::endl;

    session_manager_ = new k_api::SessionManager(router_);
    session_manager_->CreateSession(create_session_info);

    session_manager_real_time_ = new k_api::SessionManager(router_real_time_);
    session_manager_real_time_->CreateSession(create_session_info);

    std::cout << "Sessions created." << std::endl;
}

void KortexRobotInterface::createServices()
{
    base_ = new k_api::Base::BaseClient(router_);
    base_cyclic_ = new k_api::BaseCyclic::BaseCyclicClient(router_real_time_);
    actuator_config_ = new k_api::ActuatorConfig::ActuatorConfigClient(router_);
}

std::function<void(k_api::Base::ActionNotification)>
KortexRobotInterface::createActionListener(std::promise<k_api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise](k_api::Base::ActionNotification notification) {
        const auto action_event = notification.action_event();

        switch (action_event) {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            finish_promise.set_value(action_event);
            break;
        default:
            break;
        }
    };
}

bool KortexRobotInterface::moveToExperimentStart()
{
    if (base_ == nullptr) {
        std::cout << "Base client is null." << std::endl;
        return false;
    }

    try {
        std::cout << "Starting angular action movement ..." << std::endl;

        auto action = k_api::Base::Action();
        action.set_name("Move to experiment start position");
        action.set_application_data("");

        auto reach_joint_angles = action.mutable_reach_joint_angles();
        auto joint_angles = reach_joint_angles->mutable_joint_angles();

        auto actuator_count = base_->GetActuatorCount();

        float exp_start_angle[] = {155.161f, 90.0f, 90.0f, 131.619f, 0.0f, 0.0f, 0.0f};

        for (size_t i = 0; i < actuator_count.count(); ++i) {
            auto joint_angle = joint_angles->add_joint_angles();
            joint_angle->set_joint_identifier(i);
            joint_angle->set_value(exp_start_angle[i]);
        }

        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();

        auto notification_handle =
            base_->OnNotificationActionTopic(createActionListener(finish_promise), k_api::Common::NotificationOptions());

        std::cout << "Executing action" << std::endl;
        base_->ExecuteAction(action);

        std::cout << "Waiting for movement to finish ..." << std::endl;
        const auto status = finish_future.wait_for(TIMEOUT_PROMISE_DURATION);

        base_->Unsubscribe(notification_handle);

        if (status != std::future_status::ready) {
            std::cout << "Timeout on action notification wait." << std::endl;
            return false;
        }

        const auto promise_event = finish_future.get();

        std::cout << "Angular movement completed." << std::endl;
        std::cout << "Promise value: " << k_api::Base::ActionEvent_Name(promise_event) << std::endl;

        return promise_event == k_api::Base::ActionEvent::ACTION_END;
    } catch (k_api::KDetailedException& ex) {
        std::cout << "Kortex API error during angular movement: " << ex.what() << std::endl;
        return false;
    } catch (std::runtime_error& ex) {
        std::cout << "Runtime error during angular movement: " << ex.what() << std::endl;
        return false;
    } catch (...) {
        std::cout << "Unknown error during angular movement." << std::endl;
        return false;
    }
}

void KortexRobotInterface::disconnect()
{
    try {
        if (session_manager_ != nullptr) {
            session_manager_->CloseSession();
        }

        if (session_manager_real_time_ != nullptr) {
            session_manager_real_time_->CloseSession();
        }
    } catch (...) {
        std::cout << "Warning: failed to close Kortex sessions." << std::endl;
    }

    delete actuator_config_;
    actuator_config_ = nullptr;

    delete base_cyclic_;
    base_cyclic_ = nullptr;

    delete base_;
    base_ = nullptr;

    delete session_manager_real_time_;
    session_manager_real_time_ = nullptr;

    delete session_manager_;
    session_manager_ = nullptr;

    delete router_real_time_;
    router_real_time_ = nullptr;

    delete transport_real_time_;
    transport_real_time_ = nullptr;

    delete router_;
    router_ = nullptr;

    delete transport_;
    transport_ = nullptr;
}

k_api::Base::BaseClient* KortexRobotInterface::base()
{
    return base_;
}

k_api::BaseCyclic::BaseCyclicClient* KortexRobotInterface::baseCyclic()
{
    return base_cyclic_;
}

k_api::ActuatorConfig::ActuatorConfigClient* KortexRobotInterface::actuatorConfig()
{
    return actuator_config_;
}
