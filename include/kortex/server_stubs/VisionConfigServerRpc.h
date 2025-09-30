#ifndef __APIVISIONCONFIGSERVER_H__
#define __APIVISIONCONFIGSERVER_H__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <functional>
#include "KeApiRouter.h"
#include "KeApiService.h"
#include "VisionConfig.pb.h"
#include <map>

class VisionConfigServer : public KeApiService
{
	public:
		VisionConfigServer(bool isServiceOutOfSessionAllowed);
		virtual ~VisionConfigServer();

		bool Init(KeApiRouter* pApiRouter);

		virtual Kinova::Api::Error SetSensorSettings(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::VisionConfig::SensorSettings* pInSensorSettings, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetSensorSettings(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::VisionConfig::SensorIdentifier* pInSensorIdentifier, Kinova::Api::VisionConfig::SensorSettings* pOutSensorSettings);
		virtual Kinova::Api::Error GetOptionValue(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::VisionConfig::OptionIdentifier* pInOptionIdentifier, Kinova::Api::VisionConfig::OptionValue* pOutOptionValue);
		virtual Kinova::Api::Error SetOptionValue(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::VisionConfig::OptionValue* pInOptionValue, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetOptionInformation(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::VisionConfig::OptionIdentifier* pInOptionIdentifier, Kinova::Api::VisionConfig::OptionInformation* pOutOptionInformation);
		virtual Kinova::Api::Error DoSensorFocusAction(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::VisionConfig::SensorFocusAction* pInSensorFocusAction, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetIntrinsicParameters(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::VisionConfig::SensorIdentifier* pInSensorIdentifier, Kinova::Api::VisionConfig::IntrinsicParameters* pOutIntrinsicParameters);
		virtual Kinova::Api::Error GetIntrinsicParametersProfile(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::VisionConfig::IntrinsicProfileIdentifier* pInIntrinsicProfileIdentifier, Kinova::Api::VisionConfig::IntrinsicParameters* pOutIntrinsicParameters);
		virtual Kinova::Api::Error SetIntrinsicParameters(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::VisionConfig::IntrinsicParameters* pInIntrinsicParameters, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetExtrinsicParameters(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::VisionConfig::ExtrinsicParameters* pOutExtrinsicParameters);
		virtual Kinova::Api::Error SetExtrinsicParameters(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::VisionConfig::ExtrinsicParameters* pInExtrinsicParameters, Kinova::Api::Common::Empty* pOutEmpty);

		bool IsThereVisionTopicSubscribedNotif();

		void NotifyVisionTopic(Kinova::Api::VisionConfig::VisionNotification& visionNotification);

	private:
		void cbCleanupNotification( const ClientConnectionHandle &connectionHandle );

		ReturnStatus cbSetSensorSettings(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetSensorSettings(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetOptionValue(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetOptionValue(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetOptionInformation(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbOnNotificationVisionTopic(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbDoSensorFocusAction(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetIntrinsicParameters(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetIntrinsicParametersProfile(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetIntrinsicParameters(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetExtrinsicParameters(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetExtrinsicParameters(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		
		
		MapNotifs m_mapNotifVisionTopic;
};

#endif