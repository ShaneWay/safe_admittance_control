#ifndef __APITESTSERVER_H__
#define __APITESTSERVER_H__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <functional>
#include "KeApiRouter.h"
#include "KeApiService.h"
#include "Test.pb.h"
#include <map>

class TestServer : public KeApiService
{
	public:
		TestServer(bool isServiceOutOfSessionAllowed);
		virtual ~TestServer();

		bool Init(KeApiRouter* pApiRouter);

		virtual Kinova::Api::Error SetMockValidationStruct(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Test::validateStruct* pInvalidateStruct, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error TestParamAndReturn(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Test::SendStruct* pInSendStruct, Kinova::Api::Test::RcvStruct* pOutRcvStruct);
		virtual Kinova::Api::Error TestParamOnly(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Test::SendStruct* pInSendStruct, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error TestReturnOnly(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::Test::RcvStruct* pOutRcvStruct);
		virtual Kinova::Api::Error TestTimeout(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error TestAsync(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Test::timeToResponse* pIntimeToResponse, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error TestConcurrence(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error TestTriggerNotif(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error TestNotImplemented(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error ServerError(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Test::TestError* pInTestError, Kinova::Api::Test::TestError* pOutTestError);
		virtual Kinova::Api::Error TriggerSomethingChangeTopic(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error Wait(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Test::Delay* pInDelay, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error Throw(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Error* pInError, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error Disconnect(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error Forget(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error NotImplemented(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error Deprecated(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error DeprecatedWithMessage(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::Common::Empty* pOutEmpty);

		bool IsThereTestNotifSubscribedNotif();
		bool IsThereSomethingChangeTopicSubscribedNotif();

		void NotifyTestNotif(Kinova::Api::Test::TestNotification& testNotification);
		void NotifySomethingChangeTopic(Kinova::Api::Test::SomethingChanged& somethingChanged);

	private:
		void cbCleanupNotification( const ClientConnectionHandle &connectionHandle );

		ReturnStatus cbSetMockValidationStruct(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbTestParamAndReturn(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbTestParamOnly(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbTestReturnOnly(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbTestTimeout(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbOnNotificationTestNotif(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbTestNotifUnsubscribe(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbTestAsync(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbTestConcurrence(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbTestTriggerNotif(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbTestNotImplemented(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbServerError(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbUnsubscribe(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbOnNotificationSomethingChangeTopic(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbTriggerSomethingChangeTopic(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbWait(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbThrow(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbDisconnect(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbForget(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbNotImplemented(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbDeprecated(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbDeprecatedWithMessage(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		
		ReturnStatus TestNotifUnsubscribe(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::NotificationHandle* pInNotificationHandle, Kinova::Api::Common::Empty* pOutEmpty);
		ReturnStatus Unsubscribe(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::NotificationHandle* pInNotificationHandle, Kinova::Api::Common::Empty* pOutEmpty);
		
		MapNotifs m_mapNotifTestNotif;
		MapNotifs m_mapNotifSomethingChangeTopic;
};

#endif