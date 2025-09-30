
/* ***************************************************************************
 * Kinova inc.
 *
 * Copyright (c) 2006-2017 Kinova Incorporated. All rights reserved.
 ****************************************************************************/

#ifndef CPPAPI_KEAPI_DEVICE_MNG_H
#define CPPAPI_KEAPI_DEVICE_MNG_H

#include <map>
#include <functional>

#include "Frame.pb.h"
#include "DeviceManager.pb.h"
#include "DeviceManagerServerRpc.h"
#include "RouterBridge.h"

class KeApiDeviceManager : public DeviceManagerServer
{
    RouterBridge*   m_pRouterBridge;
    
public:
    KeApiDeviceManager(RouterBridge *pRouterBridge, bool isServiceOutOfSessionAllowed = false);
    virtual ~KeApiDeviceManager();
    
    virtual Kinova::Api::Error ReadAllDevices(const KeApiRouter::tSessionInfo &sessionInfo, Common::Empty* pInEmpty, Kinova::Api::DeviceManager::DeviceHandles* pOutDeviceHandles) override;
};

#endif // CPPAPI_KEAPI_DEVICE_MNG_H
