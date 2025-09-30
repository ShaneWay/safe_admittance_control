#ifndef _ROUTER_BRIDGE_H_
#define _ROUTER_BRIDGE_H_

#include <string>
#include <map>
#include <functional>
#include <exception>
#include <DeviceManager.pb.h>
#include <Common.pb.h>

#include "Frame.pb.h"

#include "IRouterClient.h"

#include "KeApiRouter.h"    // router server
#include "KeApiService.h"    // router server

using namespace std;
using namespace Kinova::Api;

    // typedef map< uint32_t, function<Error (Frame&)> > NotificationPerSession;
    
    struct BridgingInfo
    {
        uint8_t                     bridgedDeviceId;
        Common::DeviceTypes         bridgedDeviceType;
        uint8_t                     bridgedDeviceOrder;
        string                      bridgedDeviceTagId;
        
        IRouterClient*      pClientRouter;
    };
    
    class RouterBridge
    {
    private:

    public:
        RouterBridge(KeApiRouter* serverRouter, BridgingInfo serverInfo);
        ~RouterBridge();
    
        void BridgingFromServer(uint32_t bridgedDeviceId, const Frame &msgFrame);
        void BridgingFromClient(const Frame &msgFrame);
    
        void RegisterClientBridgingInfo(BridgingInfo bridgingInfo);
        void UnregisterClientBridgingInfo(uint8_t bridgedDeviceId);
        void UnregisterAllClientBridgingInfo();
    
        const unordered_map<uint8_t, BridgingInfo>& GetAvailableBridgingTable();
        unordered_map<uint8_t, BridgingInfo> GetAllDevicesInfo();
        
    private:
        KeApiRouter*                m_pServerRouter;
    
        unordered_map<uint8_t, BridgingInfo>    m_bridgeTable;
        BridgingInfo                            m_serverInfo;
    };


#endif // _ROUTER_BRIDGE_H_
