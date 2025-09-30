/* ***************************************************************************
 * Kinova inc.
 * Copyright (c) 2006-2017 Kinova Incorporated. All rights reserved.
 ****************************************************************************/

#ifndef __KEAPIROUTER_H__
#define __KEAPIROUTER_H__


#include <functional>
#include <map>
#include <ctime>

#include "Frame.pb.h"
#include "Frame.h"

#include "ITransportServer.h"
#include "KBasicException.h"
#include "KError.h"

#include <chrono>
#include <atomic>

using ChronoClock = typename std::conditional<
                                                std::chrono::high_resolution_clock::is_steady,
                                                std::chrono::high_resolution_clock,
                                                std::chrono::steady_clock
                                             >::type;

using namespace Kinova::Api;

struct ReturnStatus
{
    Kinova::Api::Error  m_error;

    ReturnStatus(Kinova::Api::Error error)
    {
        m_error = error;
    }
    ReturnStatus(Kinova::Api::ErrorCodes errorCode, Kinova::Api::SubErrorCodes errorSubCode, std::string errorSubString = std::string(""))
    {
        setError(errorCode, errorSubCode, errorSubString);
    }
    ReturnStatus()
    {
        setError(ErrorCodes::ERROR_NONE, Kinova::Api::SubErrorCodes::SUB_ERROR_NONE);
    }
    ~ReturnStatus(){}

    Kinova::Api::Error&& getError()
    {
        return std::move(m_error);
    }

    void setError(Kinova::Api::ErrorCodes errorCode, uint16_t errorSubCode, std::string errorSubString = std::string(""))
    {
        m_error.set_error_code(errorCode);
        m_error.set_error_sub_code(errorSubCode);
        m_error.set_error_sub_string(errorSubString);
    }

    bool isSuccessful()
    {
        return (m_error.error_code() == ErrorCodes::ERROR_NONE);
    }

    uint16_t getErrorStatus()
    {
        return (m_error.error_sub_code()  & 0x0FFF) | ((m_error.error_code()  << 12) & 0xF000);
    }
    
    Kinova::Api::Frame createErrorMsgFrame( const Kinova::Api::Header& refHeader )
    {
        Kinova::Api::Frame errorMsgFrame;

        std::string serializedError;
        m_error.SerializeToString(&serializedError);
    
        auto header = errorMsgFrame.mutable_header();
        header->CopyFrom( refHeader );
    
        uint32_t DeviceIdMask = refHeader.frame_info() & 0x00FF0000;
        uint32_t frameInfo = ((Kinova::Api::HeaderVersion::CURRENT_VERSION << 28) & 0xF0000000) |
                             ((Kinova::Api::FrameTypes::MSG_FRAME_RESPONSE << 24) & 0x0F000000) |
                             DeviceIdMask | (getErrorStatus() & 0x0000FFFF);
        header->set_frame_info(frameInfo);
    
        uint32_t payloadInfo = ((0 << 24)           & 0xFF000000) |
                               ((serializedError.size()) & 0x00FFFFFF);
        header->set_payload_info(payloadInfo);
    
        errorMsgFrame.set_payload(serializedError);
    
        return errorMsgFrame;
    }
};

class KeApiRouter
{
public:
    // KeApiRouter(bool isSessionIdToManage = false, bool isRouterBridgingOutOfSessionAllowed = false);
    KeApiRouter(bool isRouterBridgingOutOfSessionAllowed = false);
    virtual ~KeApiRouter();

    bool Init(ITransportServer* pTransportServer, std::function<void(KError)> callback);
    void Exit();

    struct tSessionInfo
    {
        //
        // Owned by the Server App (read-write)
        //
        uint32_t    userId; // todo: replace by userProfileHandle and move definition of UserProfileHandle 
                            // from Base.proto to Frame.proto ... but will be implemented during the refactoring of session
        bool        isLoggedIn;
        bool        active; // true until a client connection is considered inactive (inactivity timeout)
        int         inactivityTimeout_ms;           // session inactivity time before the session times out and closes on its own
        uint32_t    connectionInactivityTimeout_ms; // todo could be based on tcp sanity or associated to a specific connection type (udp, rs-485, etc.)
        
        //
        // Owned by the API (read-only outside)
        //
        int         sessionId;
        
        // should be encapsulated and not accessible via the sessionInfo expose outside api
        ClientConnectionHandle  connectionHandle;
        ChronoClock::time_point lastActivityTimestamp;
        ChronoClock::time_point lastConnectionInactivityCallbackTimestamp;

        // TODO sfforget 2019-05-02 Consider adding events in tSessionInfo directly:
        // created, deleted, connectionInactivityTimeout, userHandleChange(old, current)
    };
    
    void ProcessReceive();
    void ProcessConnectionsSanity();
    void registerBridgingCallback(std::function<void (uint8_t, Frame &)> bridgingCallback);
    bool RegisterService(uint32_t serviceId, std::function<int(const tSessionInfo&,int,int,int,std::string*)> onMessageCb);
    void Send(const tSessionInfo &sessionInfo, Kinova::Api::FrameTypes frameType, ReturnStatus& returnStatus, std::string* pStrPayload, int deviceId, int serviceVersion, int serviceId, int functionId, int messageId );
    void RegisterCleanupNotification(uint32_t serviceId, std::function<void(const ClientConnectionHandle &connectionHandle)> functionCb);
    void RegisterErrorCallback(std::function<void(KError)> callback);

    void RegisterConnectionsInactivityCallback(std::function<void(const tSessionInfo& sessionInfo)> inactivityCallback);
    void RegisterSessionInactivityCallback(std::function<void(const tSessionInfo& sessionInfo)> sessionInactivityCallback);

    void UpdateUserSessionInfo(const tSessionInfo &sessionInfo);
    void RemoveSessionInfo( const ClientConnectionHandle& connectionHandle);

    const std::unordered_map<uint32_t, KeApiRouter::tSessionInfo>& GetConnectionsMap() const;

    int GetNextNotificationId();
    
    //void Notify(int notifId, const Kinova::Api::Header& header, std::string* pStrOut);
    void Notify(ClientConnectionHandle &connectionHandle, int notifId, std::string* pStrOut, int serviceVersion, int serviceId, int functionId);

    void sendBridgedMsgFrame(const Kinova::Api::Frame &msgFrame);

private: // Methods
    void CheckConnectionsInactivityTimeout();
    void OnMessage(const ClientConnectionHandle &connectionHandle, uint8_t* pBuffer, int rcvSize);
    
    void SendMsgFrame(const tSessionInfo &sessionInfo, const Kinova::Api::Frame& frame, bool isErrorReportingAllowed=true);

public: // Variables
    // None

private: // Variables
    ITransportServer* m_pTransportServer;

    std::unordered_map<int,std::function<int(const tSessionInfo &, int, int, int, std::string*)>> m_mapServiceRouting;
    std::unordered_map<uint32_t, tSessionInfo> m_mapConnectionSessions;
    std::atomic<int> m_nLastNotificationId;
    
    std::function<void(const tSessionInfo &)>   m_inactivityCallback;
    std::function<void(const tSessionInfo &)>   m_sessionsInactivityCallback;
    
    std::unordered_map<uint16_t, std::function<void(const ClientConnectionHandle&)>> m_cbCleanupNotification;
    std::function<void(KError)>  m_errorCallback;

    std::function<void(uint8_t, Frame&)>  m_bridgingCallback;
    std::function<bool(const tSessionInfo &sessionInfo)>  m_isBridgingAllowedCallback;
    
    std::clock_t m_timeLastCheckSessions;
};

#endif // __KEAPIROUTER_H__