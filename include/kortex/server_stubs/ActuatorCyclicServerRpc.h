#ifndef __APIACTUATORCYCLICSERVER_H__
#define __APIACTUATORCYCLICSERVER_H__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <functional>
#include "KeApiRouter.h"
#include "KeApiService.h"
#include "ActuatorCyclic.pb.h"
#include <map>

class ActuatorCyclicServer : public KeApiService
{
	public:
		ActuatorCyclicServer(bool isServiceOutOfSessionAllowed);
		virtual ~ActuatorCyclicServer();

		bool Init(KeApiRouter* pApiRouter);

		virtual Kinova::Api::Error Refresh(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ActuatorCyclic::Command* pInCommand, Kinova::Api::ActuatorCyclic::Feedback* pOutFeedback);
		virtual Kinova::Api::Error RefreshCommand(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ActuatorCyclic::Command* pInCommand, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error RefreshFeedback(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ActuatorCyclic::MessageId* pInMessageId, Kinova::Api::ActuatorCyclic::Feedback* pOutFeedback);
		virtual Kinova::Api::Error RefreshCustomData(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ActuatorCyclic::MessageId* pInMessageId, Kinova::Api::ActuatorCyclic::CustomData* pOutCustomData);



	private:
		void cbCleanupNotification( const ClientConnectionHandle &connectionHandle );

		ReturnStatus cbRefresh(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbRefreshCommand(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbRefreshFeedback(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbRefreshCustomData(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		
		
};

#endif