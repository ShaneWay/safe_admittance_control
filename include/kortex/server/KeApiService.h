/* ***************************************************************************
 * Kinova inc.
 * Copyright (c) 2006-2017 Kinova Incorporated. All rights reserved.
 ****************************************************************************/

#ifndef __KEAPISERVICE_H__
#define __KEAPISERVICE_H__

#include <map>
#include <functional>

#include <google/protobuf/message.h>
#include "KeApiRouter.h"

typedef std::map<uint16_t, ClientConnectionHandle> MapNotifs; // key: notifId
typedef std::function<ReturnStatus( const KeApiRouter::tSessionInfo &sessionInfo, std::string* pStrPayload )> CallbackServiceFunction;

class KeApiService
{
public: // Methods
	KeApiService(bool isServiceOutOfSessionAllowed);
	virtual ~KeApiService();

	bool Init(KeApiRouter* pApiRouter, int serviceId);
	void Exit();

	int OnMessage(const KeApiRouter::tSessionInfo &sessionInfo, int deviceId, int functionId, int messageId, std::string* pInOutPayload);
    
private:  // Methods
	// None

public: // Variables
	KeApiRouter* m_pApiRouter;

protected:
    void RegisterFunction(uint16_t functionId, CallbackServiceFunction functionCb);
    void removeNotification(MapNotifs &mapNotifs, const ClientConnectionHandle &connectionHandle);
	
	int	m_nApiServiceId;
	int m_nApiServiceVersion;
	
	std::unordered_map<uint16_t, CallbackServiceFunction> m_mapFunction;
	std::function<bool(const KeApiRouter::tSessionInfo &sessionInfo)>  m_outOfSessionValidationCallbackFct;
};

#endif // __KEAPISERVICE_H__