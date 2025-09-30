/* ***************************************************************************
 * Kinova inc.
 * Copyright (c) 2006-2018 Kinova Incorporated. All rights reserved.
 ****************************************************************************/

#ifndef __TRANSPORTSERVERKEOS_H__
#define __TRANSPORTSERVERKEOS_H__

#if defined(_OS_WINDOWS)
// ---- win ----
#include <stdio.h>
#include <winsock2.h>
#elif defined(_OS_UNIX)
// --- linux ---
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>      // host struct
#include <sys/select.h> // use select() for multiplexing
#include <sys/fcntl.h>  // for non-blocking

#include <iostream>
#include <unistd.h>
#include <ctime>
#include <cstdio>
#include <unistd.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#else
#warning Unknown OS type!
#endif


#include "ITransportServer.h"
#include "KinovaTcpUtilities.h"
#include <Device/KeMacros.h>
#include <Kernel/NET/KeSocket.h>
#include <functional>
#include <memory>
#include <vector>
#include <atomic>
#include <mutex>


class TransportServerKeos : public ITransportServer
{

public: // struct & enums definitions

    struct tConnection
    {
        KeSocket*               pSock;       
        
        uint8_t*                RXBuffer;
        uint8_t*                TXBuffer;

        uint32_t                bytes_to_read { 0 };
        uint32_t                total_bytes_read { 0 };
        uint32_t                total_bytes_to_read { 0 };
        uint32_t                current_max_buffer_size_rx { 0 };
        uint32_t                current_max_buffer_size_tx { 0 };
        bool                    client_currently_receiving { false };

        ~tConnection()
        {
            if(pSock)
            {
                pSock->Destroy();
                delete pSock;
                pSock = nullptr;
            }

            if(RXBuffer)
            {
                delete [] RXBuffer;
                RXBuffer = nullptr;
            }

            if(TXBuffer)
            {
                delete [] TXBuffer;
                TXBuffer = nullptr;
            }
        }

    };

    typedef std::vector<std::pair<ClientConnectionHandle, std::unique_ptr<tConnection>>> ClientConnections;

    struct tConfiguration
    {
        bool				bBlockingSend;
        uint32_t			nBufferSizeTX;
        uint32_t			nBufferSizeRX;
        KeSocket::eType		nSocketType;
        uint32_t			nServerPort;
        uint32_t			nBindPort;
        tTimeRange		    nRXPollTimeOutUs;
        tTimeRange		    nTXPollTimeOutUs;
        KeosLONG			nServerIp;
        KeosLONG			nBindIp;
        uint32_t            nMaxRecvPerCycle;

        tConfiguration() :
                bBlockingSend(true),
                nBufferSizeTX(kDefaultMaxBufSize),
                nBufferSizeRX(kDefaultMaxBufSize),
                nSocketType(KeSocket::SOCKET_UDP),
                nServerPort(defaultConnectionSettings::kApiPort),
                nBindPort(defaultConnectionSettings::kApiPort),
                nRXPollTimeOutUs(kDefaultRXPollTimeOutUs),
                nTXPollTimeOutUs(kDefaultTXPollTimeOutUs),
                nMaxRecvPerCycle(1)
        {
#ifndef __CLION_IDE__
            KEOS_STATIC_SET(nServerIp.ub, 0, 0, 0, 0);
            KEOS_STATIC_SET(nBindIp.ub, 0, 0, 0, 0);
#endif
            nBindIp.ul = INADDR_ANY;
        }
    };

public: // Methods
    TransportServerKeos();
    explicit TransportServerKeos(const tConfiguration& refConfiguration);

    ~TransportServerKeos() override;

    bool Send(const ClientConnectionHandle& connectionHandle, char* TXBuffer, int txLen) override;
    void OnMessage(std::function<void(const ClientConnectionHandle& connectionHandle, uint8_t*,int)>) override;
    
    char* GetTxBuffer(size_t* pTxLen) override;

    void Process() override;
    bool CloseConnection(ClientConnectionHandle const& handle) override;
    void RegisterOnConnectionClosedCallback(std::function<void(ClientConnectionHandle)> onCloseCallback);

private:
    void InitializeConnection();
    void ManageTcpInterface();
    void ManageUdpInterface();
    void OnClosedConnectionCallback(ClientConnectionHandle const& handle);


    const ClientConnectionHandle* FindExistingConnectionHandle(const KeSocket::tNetAddr &incomingAddress);

    uint32_t GetNextConnectionId();
    std::string GetConnectionInfoString(KeSocket::eType socketType, KeSocket::tNetAddr netAddr);

    bool                   	         m_bInited { false };
    std::atomic<unsigned int>        m_nCurrentId { 0 };
    tConfiguration			         m_Cfg;
    KinovaTcpUtilities               m_utilities_object;

    std::unique_ptr<tConnection>                                                    m_pServerConnection;

    ClientConnections    m_pClientConnections;

    // callbacks
    std::function<void(const ClientConnectionHandle& connectionHandle, uint8_t*, int)> 	m_fncOnMessageCallback;
    std::function<void(const ClientConnectionHandle& connectionHandle)>                       m_onClosedCallback;
};

#endif // __TRANSPORTSERVERKEOS_H__