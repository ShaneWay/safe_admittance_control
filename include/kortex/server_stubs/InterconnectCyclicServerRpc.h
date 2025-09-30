#ifndef __APIINTERCONNECTCYCLICSERVER_H__
#define __APIINTERCONNECTCYCLICSERVER_H__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <functional>
#include "KeApiRouter.h"
#include "KeApiService.h"
#include "InterconnectCyclic.pb.h"
#include <map>

class InterconnectCyclicServer : public KeApiService
{
	public:
		InterconnectCyclicServer(bool isServiceOutOfSessionAllowed);
		virtual ~InterconnectCyclicServer();

		bool Init(KeApiRouter* pApiRouter);

		virtual Kinova::Api::Error Refresh(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::InterconnectCyclic::Command* pInCommand, Kinova::Api::InterconnectCyclic::Feedback* pOutFeedback);
		virtual Kinova::Api::Error RefreshCommand(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::InterconnectCyclic::Command* pInCommand, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error RefreshFeedback(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::InterconnectCyclic::MessageId* pInMessageId, Kinova::Api::InterconnectCyclic::Feedback* pOutFeedback);
		virtual Kinova::Api::Error RefreshCustomData(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::InterconnectCyclic::MessageId* pInMessageId, Kinova::Api::InterconnectCyclic::CustomData* pOutCustomData);



	private:
		void cbCleanupNotification( const ClientConnectionHandle &connectionHandle );

		ReturnStatus cbRefresh(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbRefreshCommand(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbRefreshFeedback(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbRefreshCustomData(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		
		
};

#endif