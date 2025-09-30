#ifndef __APIACTUATORCONFIGSERVER_H__
#define __APIACTUATORCONFIGSERVER_H__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <functional>
#include "KeApiRouter.h"
#include "KeApiService.h"
#include "ActuatorConfig.pb.h"
#include <map>

class ActuatorConfigServer : public KeApiService
{
	public:
		ActuatorConfigServer(bool isServiceOutOfSessionAllowed);
		virtual ~ActuatorConfigServer();

		bool Init(KeApiRouter* pApiRouter);

		virtual Kinova::Api::Error GetAxisOffsets(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::ActuatorConfig::AxisOffsets* pOutAxisOffsets);
		virtual Kinova::Api::Error SetAxisOffsets(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ActuatorConfig::AxisPosition* pInAxisPosition, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error ReadTorqueCalibration(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::ActuatorConfig::TorqueCalibration* pOutTorqueCalibration);
		virtual Kinova::Api::Error WriteTorqueCalibration(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ActuatorConfig::TorqueCalibration* pInTorqueCalibration, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error SetTorqueOffset(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ActuatorConfig::TorqueOffset* pInTorqueOffset, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetControlMode(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::ActuatorConfig::ControlModeInformation* pOutControlModeInformation);
		virtual Kinova::Api::Error SetControlMode(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ActuatorConfig::ControlModeInformation* pInControlModeInformation, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetActivatedControlLoop(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::ActuatorConfig::ControlLoop* pOutControlLoop);
		virtual Kinova::Api::Error SetActivatedControlLoop(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ActuatorConfig::ControlLoop* pInControlLoop, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetVectorDriveParameters(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::ActuatorConfig::VectorDriveParameters* pOutVectorDriveParameters);
		virtual Kinova::Api::Error SetVectorDriveParameters(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ActuatorConfig::VectorDriveParameters* pInVectorDriveParameters, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetEncoderDerivativeParameters(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::ActuatorConfig::EncoderDerivativeParameters* pOutEncoderDerivativeParameters);
		virtual Kinova::Api::Error SetEncoderDerivativeParameters(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ActuatorConfig::EncoderDerivativeParameters* pInEncoderDerivativeParameters, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetControlLoopParameters(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ActuatorConfig::LoopSelection* pInLoopSelection, Kinova::Api::ActuatorConfig::ControlLoopParameters* pOutControlLoopParameters);
		virtual Kinova::Api::Error SetControlLoopParameters(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ActuatorConfig::ControlLoopParameters* pInControlLoopParameters, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error StartFrequencyResponse(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ActuatorConfig::FrequencyResponse* pInFrequencyResponse, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error StopFrequencyResponse(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error StartStepResponse(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ActuatorConfig::StepResponse* pInStepResponse, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error StopStepResponse(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error StartRampResponse(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ActuatorConfig::RampResponse* pInRampResponse, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error StopRampResponse(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error SelectCustomData(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ActuatorConfig::CustomDataSelection* pInCustomDataSelection, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetSelectedCustomData(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::ActuatorConfig::CustomDataSelection* pOutCustomDataSelection);
		virtual Kinova::Api::Error SetCommandMode(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ActuatorConfig::CommandModeInformation* pInCommandModeInformation, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error ClearFaults(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error SetServoing(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ActuatorConfig::Servoing* pInServoing, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error MoveToPosition(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ActuatorConfig::PositionCommand* pInPositionCommand, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetCommandMode(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::ActuatorConfig::CommandModeInformation* pOutCommandModeInformation);
		virtual Kinova::Api::Error GetServoing(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::ActuatorConfig::Servoing* pOutServoing);
		virtual Kinova::Api::Error GetTorqueOffset(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::ActuatorConfig::TorqueOffset* pOutTorqueOffset);
		virtual Kinova::Api::Error SetCoggingFeedforwardMode(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::ActuatorConfig::CoggingFeedforwardModeInformation* pInCoggingFeedforwardModeInformation, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetCoggingFeedforwardMode(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::ActuatorConfig::CoggingFeedforwardModeInformation* pOutCoggingFeedforwardModeInformation);



	private:
		void cbCleanupNotification( const ClientConnectionHandle &connectionHandle );

		ReturnStatus cbGetAxisOffsets(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetAxisOffsets(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbReadTorqueCalibration(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbWriteTorqueCalibration(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetTorqueOffset(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetControlMode(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetControlMode(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetActivatedControlLoop(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetActivatedControlLoop(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetVectorDriveParameters(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetVectorDriveParameters(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetEncoderDerivativeParameters(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetEncoderDerivativeParameters(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetControlLoopParameters(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetControlLoopParameters(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbStartFrequencyResponse(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbStopFrequencyResponse(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbStartStepResponse(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbStopStepResponse(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbStartRampResponse(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbStopRampResponse(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSelectCustomData(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetSelectedCustomData(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetCommandMode(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbClearFaults(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetServoing(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbMoveToPosition(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetCommandMode(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetServoing(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetTorqueOffset(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetCoggingFeedforwardMode(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetCoggingFeedforwardMode(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		
		
};

#endif