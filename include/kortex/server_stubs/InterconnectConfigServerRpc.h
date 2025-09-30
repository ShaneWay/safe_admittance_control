#ifndef __APIINTERCONNECTCONFIGSERVER_H__
#define __APIINTERCONNECTCONFIGSERVER_H__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <functional>
#include "KeApiRouter.h"
#include "KeApiService.h"
#include "InterconnectConfig.pb.h"
#include <map>

class InterconnectConfigServer : public KeApiService
{
	public:
		InterconnectConfigServer(bool isServiceOutOfSessionAllowed);
		virtual ~InterconnectConfigServer();

		bool Init(KeApiRouter* pApiRouter);

		virtual Kinova::Api::Error GetUARTConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::UARTDeviceIdentification* pInUARTDeviceIdentification, Kinova::Api::Common::UARTConfiguration* pOutUARTConfiguration);
		virtual Kinova::Api::Error SetUARTConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::UARTConfiguration* pInUARTConfiguration, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetEthernetConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::InterconnectConfig::EthernetDeviceIdentification* pInEthernetDeviceIdentification, Kinova::Api::InterconnectConfig::EthernetConfiguration* pOutEthernetConfiguration);
		virtual Kinova::Api::Error SetEthernetConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::InterconnectConfig::EthernetConfiguration* pInEthernetConfiguration, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetGPIOConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::InterconnectConfig::GPIOIdentification* pInGPIOIdentification, Kinova::Api::InterconnectConfig::GPIOConfiguration* pOutGPIOConfiguration);
		virtual Kinova::Api::Error SetGPIOConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::InterconnectConfig::GPIOConfiguration* pInGPIOConfiguration, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetGPIOState(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::InterconnectConfig::GPIOIdentification* pInGPIOIdentification, Kinova::Api::InterconnectConfig::GPIOState* pOutGPIOState);
		virtual Kinova::Api::Error SetGPIOState(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::InterconnectConfig::GPIOState* pInGPIOState, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error GetI2CConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::InterconnectConfig::I2CDeviceIdentification* pInI2CDeviceIdentification, Kinova::Api::InterconnectConfig::I2CConfiguration* pOutI2CConfiguration);
		virtual Kinova::Api::Error SetI2CConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::InterconnectConfig::I2CConfiguration* pInI2CConfiguration, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error I2CRead(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::InterconnectConfig::I2CReadParameter* pInI2CReadParameter, Kinova::Api::InterconnectConfig::I2CData* pOutI2CData);
		virtual Kinova::Api::Error I2CReadRegister(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::InterconnectConfig::I2CReadRegisterParameter* pInI2CReadRegisterParameter, Kinova::Api::InterconnectConfig::I2CData* pOutI2CData);
		virtual Kinova::Api::Error I2CWrite(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::InterconnectConfig::I2CWriteParameter* pInI2CWriteParameter, Kinova::Api::Common::Empty* pOutEmpty);
		virtual Kinova::Api::Error I2CWriteRegister(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::InterconnectConfig::I2CWriteRegisterParameter* pInI2CWriteRegisterParameter, Kinova::Api::Common::Empty* pOutEmpty);



	private:
		void cbCleanupNotification( const ClientConnectionHandle &connectionHandle );

		ReturnStatus cbGetUARTConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetUARTConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetEthernetConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetEthernetConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetGPIOConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetGPIOConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetGPIOState(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetGPIOState(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbGetI2CConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbSetI2CConfiguration(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbI2CRead(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbI2CReadRegister(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbI2CWrite(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		ReturnStatus cbI2CWriteRegister(const KeApiRouter::tSessionInfo &sessionInfo, std::string* pInOutpayload);
		
		
};

#endif