//*****************************************************************************
//                             WiFi -- Start
//*****************************************************************************
// WIRELESS_MODE must be either ROLE_STA or ROLE_AP
#define WIRELESS_MODE     ROLE_STA
#define WIRELESS_SSID     "Roomba"
#define WIRELESS_PASSWORD "roooooooomba"

#define UDP_PORT      5000
#define UDP_MAX_SIZE  64
#define UDP_START_MSG "Roooooomba"

void AP_INIT() {
	// Create UDP socket
	int sock;
	SlSockAddrIn_t serverAddr;

	sock = sl_Socket(SL_AF_INET, SL_SOCK_DGRAM, 0);
	if (sock < 0) {
		UART_PRINT("Failed to create UDP socket.\r\n");
		LOOP_FOREVER();
	}

	serverAddr.sin_family = SL_AF_INET;
	serverAddr.sin_port = sl_Htons(UDP_PORT);
	serverAddr.sin_addr.s_addr = 0;

	int status = bind(sock, (SlSockAddr_t *)&serverAddr, sizeof(serverAddr));
	if (status < 0) {
		UART_PRINT("Failed to bind UDP socket.\r\n");
		LOOP_FOREVER();
	}

	SlSockAddrIn_t clientAddr;
	SlSocklen_t addrSize = sizeof(clientAddr);

	// Wait for rover start signal
	while (1) {
		char buffer[UDP_MAX_SIZE];
		int len = sl_RecvFrom(sock,
		                      buffer,
		                      sizeof(buffer),
		                      0,
		                      (SlSockAddr_t *)&clientAddr,
		                      &addrSize);

		if (len > 0) {
			buffer[len] = '\0';

			if (strcmp(buffer, UDP_START_MSG)) {
				// Rover connected
				roverConnected = 1;
				break;
			}
		}
	}

	// Broadcast control signals
	while (1) {
	}
}

void AP_CONFIGURE() {
    ConfigureSimpleLinkDefaults();

	long retVal;
	retVal = sl_Start(0, 0, 0);

	if (retVal != ROLE_AP) {
		retVal = sl_WlanSetMode(ROLE_AP);
		sl_Stop(0);
		retVal = sl_Start(0, 0, 0);
	}

	sl_WlanSet(SL_WLAN_CFG_AP_ID,
	           WLAN_AP_OPT_SSID,
	           strlen(WIRELESS_SSID),
	           (unsigned char *)WIRELESS_SSID);

	sl_WlanSet(SL_WLAN_CFG_AP_ID,
	           WLAN_AP_OPT_SECURITY_TYPE,
	           1,
	           (unsigned char[]){SL_SEC_TYPE_WPA_WPA2});

	sl_WlanSet(SL_WLAN_CFG_AP_ID,
	           WLAN_AP_OPT_PASSWORD,
	           strlen(WIRELESS_PASSWORD),
	           (unsigned char *)WIRELESS_PASSWORD);

	sl_Stop(0);
	sl_Start(0, 0, 0);
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
static long ConfigureSimpleLinkDefaults() {
	SlVersionFull ver = {0};
	_WlanRxFilterOperationCommandBuff_t RxFilterIdMask = {0};

	unsigned char ucVal = 1;
	unsigned char ucConfigOpt = 0;
	unsigned char ucConfigLen = 0;
	unsigned char ucPower = 0;

	long lRetVal = -1;
	long lMode = -1;

	lMode = sl_Start(0, 0, 0);
	ASSERT_ON_ERROR(lMode);

	// If the device is not in station-mode, try configuring it in station-mode
	if (WIRELESS_MODE != lMode) {
		// we need to wait for this event before doing anything
		while (!IS_IP_ACQUIRED(g_ulStatus)) {
			_SlNonOsMainLoopTask();
		}

		lRetVal = sl_WlanSetMode(WIRELESS_MODE);
		ASSERT_ON_ERROR(lRetVal);

		lRetVal = sl_Stop(0xFF);
		ASSERT_ON_ERROR(lRetVal);

		lRetVal = sl_Start(0, 0, 0);
		ASSERT_ON_ERROR(lRetVal);

		// Check if the device is in correct mode again
		if (WIRELESS_MODE != lRetVal) {
			// We don't want to proceed if the device is not correct mode
			LOOP_FOREVER();
		}
	}

	// Get the device's version-information
	ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
	ucConfigLen = sizeof(ver);
	lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt,
	                    &ucConfigLen, (unsigned char *)(&ver));
	ASSERT_ON_ERROR(lRetVal);

	UART_PRINT("Host Driver Version: %s\n\r", SL_DRIVER_VERSION);
	UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
	           ver.NwpVersion[0], ver.NwpVersion[1], ver.NwpVersion[2], ver.NwpVersion[3],
	           ver.ChipFwAndPhyVersion.FwVersion[0], ver.ChipFwAndPhyVersion.FwVersion[1],
	           ver.ChipFwAndPhyVersion.FwVersion[2], ver.ChipFwAndPhyVersion.FwVersion[3],
	           ver.ChipFwAndPhyVersion.PhyVersion[0], ver.ChipFwAndPhyVersion.PhyVersion[1],
	           ver.ChipFwAndPhyVersion.PhyVersion[2], ver.ChipFwAndPhyVersion.PhyVersion[3]);

	// Set connection policy to Auto + SmartConfig
	//      (Device's default connection policy)
	lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION,
	                           SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
	ASSERT_ON_ERROR(lRetVal);

	// Remove all profiles
	lRetVal = sl_WlanProfileDel(0xFF);
	ASSERT_ON_ERROR(lRetVal);

	if (ROLE_STA == WIRELESS_MODE) {
		// Enable DHCP client
		lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE, 1, 1, &ucVal);
		ASSERT_ON_ERROR(lRetVal);
	} else if (ROLE_AP == WIRELESS_MODE) {
        // AP mode: enable DHCP server
        unsigned char dhcpEnable = 1;
        lRetVal = sl_NetAppSet(SL_NETAPP_DHCP_SERVER_ENABLE, 1, 1, &dhcpEnable);
        if(lRetVal < 0)
        {
            printf("Failed to enable DHCP server\n");
            return lRetVal;
        }

        // Optional: wait for IP acquisition
        while(!IS_IP_ACQUIRED(g_ulStatus))
        {
            _SlNonOsMainLoopTask();
        }
    }

	// Set PM policy to always on
	lRetVal = sl_WlanPolicySet(SL_POLICY_PM, SL_ALWAYS_ON_POLICY, NULL, 0);
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

	return SUCCESS;
}

//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent) {
	switch (pWlanEvent->Event) {
	case SL_WLAN_CONNECT_EVENT: {
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
		memcpy(g_ucConnectionSSID, pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_name,
		       pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
		memcpy(g_ucConnectionBSSID,
		       pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
		       SL_BSSID_LENGTH);

		UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s ,"
		           " BSSID: %x:%x:%x:%x:%x:%x\n\r",
		           g_ucConnectionSSID, g_ucConnectionBSSID[0],
		           g_ucConnectionBSSID[1], g_ucConnectionBSSID[2],
		           g_ucConnectionBSSID[3], g_ucConnectionBSSID[4],
		           g_ucConnectionBSSID[5]);
	} break;

	case SL_WLAN_DISCONNECT_EVENT: {
		slWlanConnectAsyncResponse_t *pEventData = NULL;

		CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
		CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

		pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

		// If the user has initiated 'Disconnect' request,
		//'reason_code' is SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION
		if (SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION == pEventData->reason_code) {
			UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
			           "BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
			           g_ucConnectionSSID, g_ucConnectionBSSID[0],
			           g_ucConnectionBSSID[1], g_ucConnectionBSSID[2],
			           g_ucConnectionBSSID[3], g_ucConnectionBSSID[4],
			           g_ucConnectionBSSID[5]);
		} else {
			UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s,"
			           "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
			           g_ucConnectionSSID, g_ucConnectionBSSID[0],
			           g_ucConnectionBSSID[1], g_ucConnectionBSSID[2],
			           g_ucConnectionBSSID[3], g_ucConnectionBSSID[4],
			           g_ucConnectionBSSID[5]);
		}
		memset(g_ucConnectionSSID, 0, sizeof(g_ucConnectionSSID));
		memset(g_ucConnectionBSSID, 0, sizeof(g_ucConnectionBSSID));
	} break;

	default: {
		UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
		           pWlanEvent->Event);
	} break;
	}
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent) {
	switch (pNetAppEvent->Event) {
	case SL_NETAPP_IPV4_IPACQUIRED_EVENT: {
		SlIpV4AcquiredAsync_t *pEventData = NULL;

		SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

		// Ip Acquired Event Data
		pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

		// Gateway IP address
		g_ulGatewayIP = pEventData->gateway;

		UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
		           "Gateway=%d.%d.%d.%d\n\r",
		           SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip, 3),
		           SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip, 2),
		           SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip, 1),
		           SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip, 0),
		           SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway, 3),
		           SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway, 2),
		           SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway, 1),
		           SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway, 0));
	} break;

	default: {
		UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
		           pNetAppEvent->Event);
	} break;
	}
}

//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse) {
	// Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent) {
	//
	// Most of the general errors are not FATAL are are to be handled
	// appropriately by the application
	//
	UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
	           pDevEvent->EventData.deviceEvent.status,
	           pDevEvent->EventData.deviceEvent.sender);
}

//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock) {
	//
	// This application doesn't work w/ socket - Events are not expected
	//
	switch (pSock->Event) {
	case SL_SOCKET_TX_FAILED_EVENT:
		switch (pSock->socketAsyncEvent.SockTxFailData.status) {
		case SL_ECLOSE:
			UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
			           "failed to transmit all queued packets\n\n",
			           pSock->socketAsyncEvent.SockAsyncData.sd);
			break;
		default:
			UART_PRINT("[SOCK ERROR] - TX FAILED : socket %d , reason"
			           "(%d) \n\n",
			           pSock->socketAsyncEvent.SockAsyncData.sd,
			           pSock->socketAsyncEvent.SockTxFailData.status);
		}
		break;

	default:
		UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n", pSock->Event);
	}
}
//*****************************************************************************
//                             WiFi -- End
//*****************************************************************************