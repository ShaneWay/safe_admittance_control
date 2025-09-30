#ifndef __APIBASECYCLICSERVER_H__
#define __APIBASECYCLICSERVER_H__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <functional>
#include "KeApiRouter.h"
#include "KeApiService.h"
#include "BaseCyclic.pb.h"
#include <map>

class BaseCyclicServer : public KeApiService
{
	public:
		BaseCyclicServer(bool isServiceOutOfSessionAllowed);
		virtual ~BaseCyclicServer();

		bool Init(KeApiRouter* pApiRouter);

		virtual Kinova::Api::Error Refresh(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::BaseCyclic::Command* pInCommand, Kinova::Api::BaseCyclic::Feedback* pOutFeedback);
		virtual Kinova::Api::Error RefreshCommand(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::BaseCyclic::Command* pInCommand, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error RefreshFeedback(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::BaseCyclic::Feedback* pOutFeedback);
		virtual Kinova::Api::Error RefreshCustomData(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::BaseCyclic::CustomData* pInCustomData, Kinova::Api::BaseCyclic::CustomData* pOutCustomData);



	private:
		void cbCleanupNotification( const ClientConnectionHandle &connectionHandle );

		ReturnStatus cbRefresh(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbRefreshCommand(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbRefreshFeedback(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbRefreshCustomData(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		
		
};

#endif