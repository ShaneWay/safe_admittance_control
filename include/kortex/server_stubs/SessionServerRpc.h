#ifndef __APISESSIONSERVER_H__
#define __APISESSIONSERVER_H__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <functional>
#include "KeApiRouter.h"
#include "KeApiService.h"
#include "Session.pb.h"
#include <map>

class SessionServer : public KeApiService
{
	public:
		SessionServer(bool isServiceOutOfSessionAllowed);
		virtual ~SessionServer();

		bool Init(KeApiRouter* pApiRouter);

		virtual Kinova::Api::Error CreateSession(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Session::CreateSessionInfo* pInCreateSessionInfo, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error CloseSession(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error KeepAlive(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetConnections(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::Session::ConnectionList* pOutConnectionList);



	private:
		void cbCleanupNotification( const ClientConnectionHandle &connectionHandle );

		ReturnStatus cbCreateSession(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbCloseSession(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbKeepAlive(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetConnections(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		
		
};

#endif