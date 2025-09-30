#ifndef __APICONTROLCONFIGSERVER_H__
#define __APICONTROLCONFIGSERVER_H__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <functional>
#include "KeApiRouter.h"
#include "KeApiService.h"
#include "ControlConfig.pb.h"
#include <map>

class ControlConfigServer : public KeApiService
{
	public:
		ControlConfigServer(bool isServiceOutOfSessionAllowed);
		virtual ~ControlConfigServer();

		bool Init(KeApiRouter* pApiRouter);

		virtual Kinova::Api::Error SetGravityVector(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ControlConfig::GravityVector* pInGravityVector, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetGravityVector(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::ControlConfig::GravityVector* pOutGravityVector);
		virtual Kinova::Api::Error SetPayloadInformation(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ControlConfig::PayloadInformation* pInPayloadInformation, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetPayloadInformation(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::ControlConfig::PayloadInformation* pOutPayloadInformation);
		virtual Kinova::Api::Error SetToolConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ControlConfig::ToolConfiguration* pInToolConfiguration, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetToolConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::ControlConfig::ToolConfiguration* pOutToolConfiguration);
		virtual Kinova::Api::Error SetCartesianReferenceFrame(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ControlConfig::CartesianReferenceFrameInfo* pInCartesianReferenceFrameInfo, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetCartesianReferenceFrame(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::ControlConfig::CartesianReferenceFrameInfo* pOutCartesianReferenceFrameInfo);
		virtual Kinova::Api::Error GetControlMode(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::ControlConfig::ControlModeInformation* pOutControlModeInformation);
		virtual Kinova::Api::Error SetJointSpeedSoftLimits(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ControlConfig::JointSpeedSoftLimits* pInJointSpeedSoftLimits, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error SetTwistLinearSoftLimit(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ControlConfig::TwistLinearSoftLimit* pInTwistLinearSoftLimit, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error SetTwistAngularSoftLimit(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ControlConfig::TwistAngularSoftLimit* pInTwistAngularSoftLimit, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error SetJointAccelerationSoftLimits(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ControlConfig::JointAccelerationSoftLimits* pInJointAccelerationSoftLimits, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetKinematicHardLimits(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::ControlConfig::KinematicLimits* pOutKinematicLimits);
		virtual Kinova::Api::Error GetKinematicSoftLimits(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ControlConfig::ControlModeInformation* pInControlModeInformation, Kinova::Api::ControlConfig::KinematicLimits* pOutKinematicLimits);
		virtual Kinova::Api::Error GetAllKinematicSoftLimits(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::ControlConfig::KinematicLimitsList* pOutKinematicLimitsList);
		virtual Kinova::Api::Error SetDesiredLinearTwist(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ControlConfig::LinearTwist* pInLinearTwist, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error SetDesiredAngularTwist(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ControlConfig::AngularTwist* pInAngularTwist, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error SetDesiredJointSpeeds(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ControlConfig::JointSpeeds* pInJointSpeeds, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetDesiredSpeeds(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::ControlConfig::DesiredSpeeds* pOutDesiredSpeeds);
		virtual Kinova::Api::Error ResetGravityVector(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::ControlConfig::GravityVector* pOutGravityVector);
		virtual Kinova::Api::Error ResetPayloadInformation(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::ControlConfig::PayloadInformation* pOutPayloadInformation);
		virtual Kinova::Api::Error ResetToolConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::ControlConfig::ToolConfiguration* pOutToolConfiguration);
		virtual Kinova::Api::Error ResetJointSpeedSoftLimits(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ControlConfig::ControlModeInformation* pInControlModeInformation, Kinova::Api::ControlConfig::JointSpeedSoftLimits* pOutJointSpeedSoftLimits);
		virtual Kinova::Api::Error ResetTwistLinearSoftLimit(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ControlConfig::ControlModeInformation* pInControlModeInformation, Kinova::Api::ControlConfig::TwistLinearSoftLimit* pOutTwistLinearSoftLimit);
		virtual Kinova::Api::Error ResetTwistAngularSoftLimit(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ControlConfig::ControlModeInformation* pInControlModeInformation, Kinova::Api::ControlConfig::TwistAngularSoftLimit* pOutTwistAngularSoftLimit);
		virtual Kinova::Api::Error ResetJointAccelerationSoftLimits(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ControlConfig::ControlModeInformation* pInControlModeInformation, Kinova::Api::ControlConfig::JointAccelerationSoftLimits* pOutJointAccelerationSoftLimits);

		bool IsThereControlConfigurationTopicSubscribedNotif();
		bool IsThereControlModeTopicSubscribedNotif();

		void NotifyControlConfigurationTopic(Kinova::Api::ControlConfig::ControlConfigurationNotification& controlConfigurationNotification);
		void NotifyControlModeTopic(Kinova::Api::ControlConfig::ControlModeNotification& controlModeNotification);

	private:
		void cbCleanupNotification( const ClientConnectionHandle &connectionHandle );

		ReturnStatus cbSetGravityVector(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetGravityVector(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetPayloadInformation(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetPayloadInformation(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetToolConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetToolConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbOnNotificationControlConfigurationTopic(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbUnsubscribe(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetCartesianReferenceFrame(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetCartesianReferenceFrame(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetControlMode(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetJointSpeedSoftLimits(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetTwistLinearSoftLimit(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetTwistAngularSoftLimit(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetJointAccelerationSoftLimits(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetKinematicHardLimits(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetKinematicSoftLimits(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetAllKinematicSoftLimits(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetDesiredLinearTwist(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetDesiredAngularTwist(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetDesiredJointSpeeds(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetDesiredSpeeds(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbResetGravityVector(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbResetPayloadInformation(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbResetToolConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbResetJointSpeedSoftLimits(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbResetTwistLinearSoftLimit(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbResetTwistAngularSoftLimit(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbResetJointAccelerationSoftLimits(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbOnNotificationControlModeTopic(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		
		ReturnStatus Unsubscribe(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::NotificationHandle* pInNotificationHandle, Kinova::Api::Common::Empty* pOutEmpty);
		
		MapNotifs m_mapNotifControlConfigurationTopic;
		MapNotifs m_mapNotifControlModeTopic;
};

#endif