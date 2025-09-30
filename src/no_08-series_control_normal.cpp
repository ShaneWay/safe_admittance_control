

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <KDetailedException.h>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>

#include "utilities.h"

#include <google/protobuf/util/json_util.h>

#include <unistd.h>
#include <dynamics_model.h>

#include <controller.h>


#include <time.h>

namespace k_api = Kinova::Api;

#define PORT 10000
#define PORT_REAL_TIME 10001

float TIME_DURATION = 10.0f; // Duration of the example (seconds)

// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_PROMISE_DURATION = std::chrono::seconds{10};


/*****************************
 * Example related function *
 *****************************/
int64_t GetTickUs()
{
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
}

// Create an event listener that will set the promise action event to the exit value
// Will set promise to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(k_api::Base::ActionNotification)> 
    create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            finish_promise.set_value(action_event);
            break;
        default:
            break;
        }
    };
}

bool example_angular_action_movement_exp(k_api::Base::BaseClient* base) 
{
    std::cout << "Starting angular action movement ..." << std::endl;

    auto action = k_api::Base::Action();
    action.set_name("Example angular action movement");
    action.set_application_data("");

    auto reach_joint_angles = action.mutable_reach_joint_angles();
    auto joint_angles = reach_joint_angles->mutable_joint_angles();

    auto actuator_count = base->GetActuatorCount();

    float exp_start_angle[] = {0., 10., 0., 120, 0., 0., 90.};

    // Arm straight up
    for (size_t i = 0; i < actuator_count.count(); ++i) 
    {
        auto joint_angle = joint_angles->add_joint_angles();
        joint_angle->set_joint_identifier(i);
        joint_angle->set_value(exp_start_angle[i]);
    }

    // Connect to notification action topic
    // (Promise alternative)
    // See cartesian examples for Reference alternative
    std::promise<k_api::Base::ActionEvent> finish_promise;
    auto finish_future = finish_promise.get_future();
    auto promise_notification_handle = base->OnNotificationActionTopic(
        create_event_listener_by_promise(finish_promise),
        k_api::Common::NotificationOptions()
    );

    std::cout << "Executing action" << std::endl;
    base->ExecuteAction(action);

    std::cout << "Waiting for movement to finish ..." << std::endl;

    // Wait for future value from promise
    // (Promise alternative)
    // See cartesian examples for Reference alternative
    const auto status = finish_future.wait_for(TIMEOUT_PROMISE_DURATION);
    base->Unsubscribe(promise_notification_handle);

    if(status != std::future_status::ready)
    {
        std::cout << "Timeout on action notification wait" << std::endl;
        return false;
    }
    const auto promise_event = finish_future.get();

    std::cout << "Angular movement completed" << std::endl;
    std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl; 

    return true;
}


bool example_angular_action_movement(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic) 
{
    std::cout << "Starting angular action movement ..." << std::endl;

    auto action = k_api::Base::Action();
    auto feedback = base_cyclic->RefreshFeedback();
    action.set_name("Example angular action movement");
    action.set_application_data("");

    auto reach_joint_angles = action.mutable_reach_joint_angles();
    auto joint_angles = reach_joint_angles->mutable_joint_angles();

    auto actuator_count = base->GetActuatorCount();

    auto joint_angle = joint_angles->add_joint_angles();
    joint_angle->set_joint_identifier(0);
    joint_angle->set_value(70);
    for (size_t i = 1; i < actuator_count.count() - 1; ++i) 
    {
        auto joint_angle = joint_angles->add_joint_angles();
        joint_angle->set_joint_identifier(i);
        joint_angle->set_value(feedback.actuators(i).position());
    }

    auto joint_angle2 = joint_angles->add_joint_angles();
    joint_angle2->set_joint_identifier(6);
    joint_angle2->set_value(90);

    // Connect to notification action topic
    // (Promise alternative)
    // See cartesian examples for Reference alternative
    std::promise<k_api::Base::ActionEvent> finish_promise;
    auto finish_future = finish_promise.get_future();
    auto promise_notification_handle = base->OnNotificationActionTopic(
        create_event_listener_by_promise(finish_promise),
        k_api::Common::NotificationOptions()
    );

    std::cout << "Executing action" << std::endl;
    base->ExecuteAction(action);

    std::cout << "Waiting for movement to finish ..." << std::endl;

    // Wait for future value from promise
    // (Promise alternative)
    // See cartesian examples for Reference alternative
    const auto status = finish_future.wait_for(TIMEOUT_PROMISE_DURATION);
    base->Unsubscribe(promise_notification_handle);

    if(status != std::future_status::ready)
    {
        std::cout << "Timeout on action notification wait" << std::endl;
        return false;
    }
    const auto promise_event = finish_future.get();

    std::cout << "Angular movement completed" << std::endl;
    std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl; 

    return true;
}


bool move_to_home_position(k_api::Base::BaseClient* base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list()) 
    {
        if (action.name() == "Home") 
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0) 
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
        return false;
    } 
    else 
    {
        // Connect to notification action topic
        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise),
            k_api::Common::NotificationOptions()
        );
        
        // Execute action
        base->ExecuteActionFromReference(action_handle);

        // Wait for future value from promise
        const auto status = finish_future.wait_for(TIMEOUT_PROMISE_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if(status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
            return false;
        }
        const auto promise_event = finish_future.get();

        std::cout << "Move to Home completed" << std::endl;
        std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl;

        return true;
    }
}




bool cyclic_torque_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, k_api::ActuatorConfig::ActuatorConfigClient* actuator_config)
{
    bool return_status = true;

    // Get actuator count
    unsigned int actuator_count = base->GetActuatorCount().count();
    
    // Clearing faults
    try
    {
        base->ClearFaults();
    }
    catch(...)
    {
        std::cout << "Unable to clear robot faults" << std::endl;
        return false;
    }

    
    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command  base_command;

    std::vector<float> commands;

    auto servoing_mode = k_api::Base::ServoingModeInformation();

    int timer_count = 0;
    int64_t now = 0;
    int64_t last = 0;

    std::cout << "Initializing the arm for torque control example" << std::endl;
    try
    {
        // Set the base in low-level servoing mode
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        base_feedback = base_cyclic->RefreshFeedback();

        // Initialize each actuator to their current position
        for (unsigned int i = 0; i < actuator_count; i++)
        {
            commands.push_back(base_feedback.actuators(i).position());

            // Save the current actuator position, to avoid a following error
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        // Send a first frame
        base_feedback = base_cyclic->Refresh(base_command);
        
        // Set last actuator in torque mode now that the command is equal to measure
        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);

        // actuator_config->SetControlMode(control_mode_message, 2);
        actuator_config->SetControlMode(control_mode_message, 4);

        std::cout << "Running torque control example for " << TIME_DURATION << " seconds" << std::endl;

        double f_input;
        double q_input;
        double tau;

        KortexDynamics model;
        controller control;

        vector<double> q(7);
        vector<double> g(7);
        // Real-time loop
        while (timer_count < (TIME_DURATION * 1000))
        {
            now = GetTickUs();

            if (now - last > 1000)
            {
                double T = 0.001;
                std::cout << "time: " << now - last <<  " us" << endl;
                // Position command to first actuator is set to measured one to avoid following error to trigger
                // Bonus: When doing this instead of disabling the following error, if communication is lost and first
                //        actuator continues to move under torque command, resulting position error with command will
                //        trigger a following error and switch back the actuator in position command to hold its position
                
                // base_command.mutable_actuators(1)->set_position(base_feedback.actuators(1).position());

                base_command.mutable_actuators(3)->set_position(base_feedback.actuators(3).position());

                for(int i =0; i < 7; i++){
                    q[i] = base_feedback.actuators(i).position() / 180.0 * M_PI;
                    // cout << q[i] << endl;
                }
                for(int j =0; j < 7; j++){
                    g[j] = 0;
                }

                model.getGravity(q, g);
                
                if (base_feedback.actuators(3).position() > 200)
                {
                    q_input = (base_feedback.actuators(3).position() - 360. ) / 180.0 * M_PI;
                }else{
                    q_input = (base_feedback.actuators(3).position() ) / 180.0 * M_PI;
                }
                
                f_input =  base_feedback.actuators(3).torque() + g[3];

                tau = control.get_tau_normal(T, f_input, q_input);

                cout << "real tau: " << tau << endl;

                cout << "==========================================!" << endl;

                base_command.mutable_actuators(3)->set_torque_joint(tau + g[3]);

                // base_command.mutable_actuators(1)->set_torque_joint(g[1]);
                // base_command.mutable_actuators(3)->set_torque_joint(g[3]);
                
                base_command.set_frame_id(base_command.frame_id() + 1);
                if (base_command.frame_id() > 65535)
                    base_command.set_frame_id(0);

                for (int idx = 0; idx < actuator_count; idx++)
                {
                    base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());
                }
                try
                {
                    base_feedback = base_cyclic->Refresh(base_command, 0);
                }
                catch (k_api::KDetailedException& ex)
                {
                    std::cout << "Kortex exception: " << ex.what() << std::endl;

                    std::cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
                }
                catch (std::runtime_error& ex2)
                {
                    std::cout << "runtime error: " << ex2.what() << std::endl;
                }
                catch(...)
                {
                    std::cout << "Unknown error." << std::endl;
                }

                timer_count++;
                last = GetTickUs();
                control.refresh_normal(T);
            }
        }


        std::cout << "Torque control example completed" << std::endl;

        // Set first actuator back in position 
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        actuator_config->SetControlMode(control_mode_message, 4);
        // actuator_config->SetControlMode(control_mode_message, 6);

        std::cout << "Torque control example clean exit" << std::endl;

    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "API error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error& ex2)
    {
        std::cout << "Error: " << ex2.what() << std::endl;
        return_status = false;
    }
    
    // Set the servoing mode back to Single Level
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return return_status;
}

int main(int argc, char **argv)
{
    auto parsed_args = ParseExampleArguments(argc, argv);

    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    
    std::cout << "Creating transport objects" << std::endl;
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(parsed_args.ip_address, PORT);

    std::cout << "Creating transport real time objects" << std::endl;
    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(parsed_args.ip_address, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Sessions created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
    auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);

    // Example core
    bool success = true;
    // success &= move_to_home_position(base);
    success &= example_angular_action_movement_exp(base);
    success &= cyclic_torque_control(base, base_cyclic, actuator_config);
    if (!success)
    {
        std::cout << "There has been an unexpected error." << endl;
    }

    // Close API session
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    // Destroy the API
    delete base;
    delete base_cyclic;
    delete actuator_config;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;

    return success ? 0 : 1;
}
