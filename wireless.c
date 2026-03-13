#include "wireless.h"
#include <string.h>

#define UDP_PORT              5000
#define UDP_MAX_SIZE          64
#define UDP_START_MSG         "Roooooomba"
#define BROADCAST_INTERVAL_MS 2000

unsigned char discoveredFlag = 0;
unsigned int peerIp = 0;

unsigned int clientIp = 0;

// Send broadcast
void controller_broadcast() {
	int sock;
	int broadcast = 1;
	SlSockAddrIn_t broadcastAddr;

	sock = sl_Socket(SL_AF_INET, SL_SOCK_DGRAM, 0);
	if (sock < 0) {
		LOOP_FOREVER();
	}

	memset(&broadcastAddr, 0, sizeof(broadcastAddr));
	broadcastAddr.sin_family = SL_AF_INET;
	broadcastAddr.sin_port = sl_Htons(DISCOVERY_PORT);
	broadcastAddr.sin_addr.s_addr = sl_Htonl(0xFFFFFFFF);

	while (!discoveredFlag) {
		sl_SendTo(sock, UDP_START_MSG, strlen(UDP_START_MSG), 0,
		          (SlSockAddr_t *)&broadcastAddr, sizeof(broadcastAddr));
		sl_TaskSleep(BROADCAST_INTERVAL_MS);
	}

	sl_Close(sock);
}

void listen_for_controller() {
	int sock;
	SlSockAddrIn_t addr;
	int addrSize = sizeof(addr);
	char buffer[256];

	// Create UDP socket
	sock = sl_Socket(SL_AF_INET, SL_SOCK_DGRAM, 0);
	if (sock < 0) {
		LOOP_FOREVER();
	}

	// Bind to all interfaces
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = SL_AF_INET;
	addr.sin_port = sl_Htons(UDP_PORT);
	addr.sin_addr.s_addr = 0; // Listen on all interfaces

	if (sl_Bind(sock, (SlSockAddr_t *)&addr, sizeof(addr)) < 0) {
		sl_Close(sock);
		LOOP_FOREVER();
	}

	while (!discoveredFlag) {
		int bytes = sl_RecvFrom(sock, buffer, sizeof(buffer) - 1, 0,
		                        (SlSockAddr_t *)&addr, &addrSize);
		if (bytes > 0) {
			buffer[bytes] = '\0';

			// Ignore own broadcast
			if (sender.sin_addr.s_addr != sl_Htonl(sl_NetCfgGet(SL_NETCFG_IPV4_ADDR))) {
				peerIp = sender.sin_addr.s_addr;
				discoveredFlag = 1;
			}
		}
	}

	sl_Close(sock);
	send_udp_peer(UDP_START_MSG);
}

void send_udp_peer(const char *msg) {
	if (!discoveredFlag) {
		LOOP_FOREVER();
	}

	int sock;
	SlSockAddrIn_t addr;

	sock = sl_Socket(SL_AF_INET, SL_SOCK_DGRAM, 0);
	if (sock < 0) {
		LOOP_FOREVER();
	}
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = SL_AF_INET;
	addr.sin_port = sl_Htons(UDP_PORT);
	addr.sin_addr.s_addr = peerIp;

	sl_SendTo(sock, msg, strlen(msg), 0, (SlSockAddr_t *)&addr, sizeof(addr));
	sl_Close(sock);
}

void receive_udp_peer(char *recv_buffer, unsigned int buffer_size) {
	int sock;
	SlSockAddrIn_t addr;
	int addrLen = sizeof(addr);
	char buffer[UDP_MAX_SIZE];

	sock = sl_Socket(SL_AF_INET, SL_SOCK_DGRAM, 0);
	if (sock < 0) {
		LOOP_FOREVER();
	}

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = SL_AF_INET;
	addr.sin_port = sl_Htons(MESSAGE_PORT);
	addr.sin_addr.s_addr = 0;

	if (sl_Bind(sock, (SlSockAddr_t *)&addr, sizeof(addr)) < 0) {
		sl_Close(sock);
		return;
	}

	while (1) {
		int bytes = sl_RecvFrom(sock, buffer, UDP_MAX_SIZE - 1, 0,
		                        (SlSockAddr_t *)&addr, &addrLen);
		if (bytes > 0) {
			buffer[bytes] = '\0';
			if (addr.sin_addr.s_addr == peerIp && buffer_size > bytes) {
				strcpy(recv_buffer, buffer);
			}
		}
	}
}

// Format is "[WASD][0-100]"
void parse_udp_packet(char *msg) {
	if (strlen(msg) < 2) {
		return;
	}

	switch (msg[0]) {
	case 'W':
		break;
	case 'S':
		break;
	case 'A':
		break;
	case 'D':
		break;
	default:
		return;
	}

	int dutyCycle = strtol(msg[1]);
}


//*****************************************************************************
//! \brief This function puts the device in its default state. It:
//!           - Set the mode to STATION
//!           - Configures connection policy to Auto and AutoSmartConfig
//!           - Deletes all the stored profiles
//!           - Enables DHCP
//!           - Disables Scan policy
//!           - Sets Tx power to maximum
//!           - Sets power policy to normal
//!           - Unregister mDNS services
//!           - Remove all filters
//!
//! \param   none
//! \return  On success, zero is returned. On error, negative is returned
//*****************************************************************************
static long ConfigureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode);

    // If the device is not in station-mode, try configuring it in station-mode 
    if (ROLE_STA != lMode)
    {
        if (ROLE_AP == lMode)
        {
            // If the device is in AP mode, we need to wait for this event 
            // before doing anything 
            while(!IS_IP_ACQUIRED(g_ulStatus))
            {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
            }
        }

        // Switch to STA role and restart 
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again 
        if (ROLE_STA != lRetVal)
        {
            // We don't want to proceed if the device is not coming up in STA-mode 
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }
    
    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt, 
                                &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(lRetVal);
    
    UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

    // Set connection policy to Auto + SmartConfig 
    //      (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, 
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);

    

    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore 
    // other return-codes
    //
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal)
    {
        // Wait
        while(IS_CONNECTED(g_ulStatus))
        {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
        }
    }

    // Enable DHCP client
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, 
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    InitializeAppVariables();
    
    return lRetVal; // Success
}

static long WlanConnect()
{
    SlSecParams_t secParams = {0};
    long lRetVal = 0;

    secParams.Key = (signed char*)SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

    lRetVal = sl_WlanConnect((signed char*)SSID_NAME, strlen(SSID_NAME), 0, \
                                    &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);

    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))
    {
        // Wait for WLAN Event
#ifndef SL_PLATFORM_MULTI_THREADED
        _SlNonOsMainLoopTask();
#endif
      
    }

    return SUCCESS;
}

void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(!pWlanEvent)
    {
        return;
    }

    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'-Applications
            // can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s , "
                "BSSID: %x:%x:%x:%x:%x:%x\n\r",
                      g_ucConnectionSSID,g_ucConnectionBSSID[0],
                      g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                      g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                      g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION
            if(SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
                "BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else
            {
                UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s,"
                "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        default:
        {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Event);
        }
        break;
    }
}