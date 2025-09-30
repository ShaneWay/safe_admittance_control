#ifndef __APIDEVICECONFIGSERVER_H__
#define __APIDEVICECONFIGSERVER_H__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <functional>
#include "KeApiRouter.h"
#include "KeApiService.h"
#include "DeviceConfig.pb.h"
#include <map>

class DeviceConfigServer : public KeApiService
{
	public:
		DeviceConfigServer(bool isServiceOutOfSessionAllowed);
		virtual ~DeviceConfigServer();

		bool Init(KeApiRouter* pApiRouter);

		virtual Kinova::Api::Error GetRunMode(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::DeviceConfig::RunMode* pOutRunMode);
		virtual Kinova::Api::Error SetRunMode(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::DeviceConfig::RunMode* pInRunMode, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetDeviceType(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::DeviceConfig::DeviceType* pOutDeviceType);
		virtual Kinova::Api::Error GetFirmwareVersion(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::DeviceConfig::FirmwareVersion* pOutFirmwareVersion);
		virtual Kinova::Api::Error GetBootloaderVersion(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::DeviceConfig::BootloaderVersion* pOutBootloaderVersion);
		virtual Kinova::Api::Error GetModelNumber(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::DeviceConfig::ModelNumber* pOutModelNumber);
		virtual Kinova::Api::Error GetPartNumber(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::DeviceConfig::PartNumber* pOutPartNumber);
		virtual Kinova::Api::Error GetSerialNumber(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::DeviceConfig::SerialNumber* pOutSerialNumber);
		virtual Kinova::Api::Error GetMACAddress(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::DeviceConfig::MACAddress* pOutMACAddress);
		virtual Kinova::Api::Error GetIPv4Settings(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::DeviceConfig::IPv4Settings* pOutIPv4Settings);
		virtual Kinova::Api::Error SetIPv4Settings(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::DeviceConfig::IPv4Settings* pInIPv4Settings, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetPartNumberRevision(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::DeviceConfig::PartNumberRevision* pOutPartNumberRevision);
		virtual Kinova::Api::Error RebootRequest(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::DeviceConfig::RebootRqst* pInRebootRqst, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error SetSafetyEnable(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::DeviceConfig::SafetyEnable* pInSafetyEnable, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error SetSafetyErrorThreshold(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::DeviceConfig::SafetyThreshold* pInSafetyThreshold, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error SetSafetyWarningThreshold(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::DeviceConfig::SafetyThreshold* pInSafetyThreshold, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error SetSafetyConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::DeviceConfig::SafetyConfiguration* pInSafetyConfiguration, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetSafetyConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::SafetyHandle* pInSafetyHandle, Kinova::Api::DeviceConfig::SafetyConfiguration* pOutSafetyConfiguration);
		virtual Kinova::Api::Error GetSafetyInformation(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::SafetyHandle* pInSafetyHandle, Kinova::Api::DeviceConfig::SafetyInformation* pOutSafetyInformation);
		virtual Kinova::Api::Error GetSafetyEnable(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::SafetyHandle* pInSafetyHandle, Kinova::Api::DeviceConfig::SafetyEnable* pOutSafetyEnable);
		virtual Kinova::Api::Error GetSafetyStatus(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::SafetyHandle* pInSafetyHandle, Kinova::Api::DeviceConfig::SafetyStatus* pOutSafetyStatus);
		virtual Kinova::Api::Error ClearAllSafetyStatus(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error ClearSafetyStatus(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::SafetyHandle* pInSafetyHandle, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetAllSafetyConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::DeviceConfig::SafetyConfigurationList* pOutSafetyConfigurationList);
		virtual Kinova::Api::Error GetAllSafetyInformation(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::DeviceConfig::SafetyInformationList* pOutSafetyInformationList);
		virtual Kinova::Api::Error ResetSafetyDefaults(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error ExecuteCalibration(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::DeviceConfig::Calibration* pInCalibration, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetCalibrationResult(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::DeviceConfig::CalibrationElement* pInCalibrationElement, Kinova::Api::DeviceConfig::CalibrationResult* pOutCalibrationResult);
		virtual Kinova::Api::Error StopCalibration(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::DeviceConfig::Calibration* pInCalibration, Kinova::Api::DeviceConfig::CalibrationResult* pOutCalibrationResult);
		virtual Kinova::Api::Error SetCapSenseConfig(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::DeviceConfig::CapSenseConfig* pInCapSenseConfig, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetCapSenseConfig(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::DeviceConfig::CapSenseConfig* pOutCapSenseConfig);
		virtual Kinova::Api::Error ReadCapSenseRegister(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::DeviceConfig::CapSenseRegister* pInCapSenseRegister, Kinova::Api::DeviceConfig::CapSenseRegister* pOutCapSenseRegister);
		virtual Kinova::Api::Error WriteCapSenseRegister(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::DeviceConfig::CapSenseRegister* pInCapSenseRegister, Kinova::Api::Common::Empty* pOutEmpty);

		bool IsThereSafetyTopicSubscribedNotif();

		void NotifySafetyTopic(Kinova::Api::Common::SafetyNotification& safetyNotification);

	private:
		void cbCleanupNotification( const ClientConnectionHandle &connectionHandle );

		ReturnStatus cbGetRunMode(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetRunMode(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetDeviceType(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetFirmwareVersion(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetBootloaderVersion(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetModelNumber(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetPartNumber(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetSerialNumber(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetMACAddress(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetIPv4Settings(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetIPv4Settings(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetPartNumberRevision(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbRebootRequest(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetSafetyEnable(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetSafetyErrorThreshold(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetSafetyWarningThreshold(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetSafetyConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetSafetyConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetSafetyInformation(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetSafetyEnable(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetSafetyStatus(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbClearAllSafetyStatus(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbClearSafetyStatus(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetAllSafetyConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetAllSafetyInformation(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbResetSafetyDefaults(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbOnNotificationSafetyTopic(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbExecuteCalibration(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetCalibrationResult(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbStopCalibration(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetCapSenseConfig(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetCapSenseConfig(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbReadCapSenseRegister(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbWriteCapSenseRegister(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		
		
		MapNotifs m_mapNotifSafetyTopic;
};

#endif