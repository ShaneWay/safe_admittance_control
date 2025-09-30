#ifndef __APIGRIPPERCYCLICSERVER_H__
#define __APIGRIPPERCYCLICSERVER_H__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <functional>
#include "KeApiRouter.h"
#include "KeApiService.h"
#include "GripperCyclic.pb.h"
#include <map>

class GripperCyclicServer : public KeApiService
{
	public:
		GripperCyclicServer(bool isServiceOutOfSessionAllowed);
		virtual ~GripperCyclicServer();

		bool Init(KeApiRouter* pApiRouter);

		virtual Kinova::Api::Error Refresh(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::GripperCyclic::Command* pInCommand, Kinova::Api::GripperCyclic::Feedback* pOutFeedback);
		virtual Kinova::Api::Error RefreshCommand(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::GripperCyclic::Command* pInCommand, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error RefreshFeedback(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::GripperCyclic::MessageId* pInMessageId, Kinova::Api::GripperCyclic::Feedback* pOutFeedback);
		virtual Kinova::Api::Error RefreshCustomData(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::GripperCyclic::MessageId* pInMessageId, Kinova::Api::GripperCyclic::CustomData* pOutCustomData);



	private:
		void cbCleanupNotification( const ClientConnectionHandle &connectionHandle );

		ReturnStatus cbRefresh(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbRefreshCommand(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbRefreshFeedback(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbRefreshCustomData(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		
		
};

#endif