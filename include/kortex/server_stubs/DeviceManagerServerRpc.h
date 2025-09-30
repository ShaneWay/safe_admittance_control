#ifndef __APIDEVICEMANAGERSERVER_H__
#define __APIDEVICEMANAGERSERVER_H__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <functional>
#include "KeApiRouter.h"
#include "KeApiService.h"
#include "DeviceManager.pb.h"
#include <map>

class DeviceManagerServer : public KeApiService
{
	public:
		DeviceManagerServer(bool isServiceOutOfSessionAllowed);
		virtual ~DeviceManagerServer();

		bool Init(KeApiRouter* pApiRouter);

		virtual Kinova::Api::Error ReadAllDevices(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::DeviceManager::DeviceHandles* pOutDeviceHandles);



	private:
		void cbCleanupNotification( const ClientConnectionHandle &connectionHandle );

		ReturnStatus cbReadAllDevices(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		
		
};

#endif