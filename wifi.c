//*****************************************************************************
// Copyright (C) 2014 Texas Instruments Incorporated
//
// All rights reserved. Property of Texas Instruments Incorporated.
// Restricted rights to use, duplicate or disclose this code are
// granted through contract.
// The program may not be used without the written permission of
// Texas Instruments Incorporated or against the terms and conditions
// stipulated in the agreement under which this program has been supplied,
// and under no circumstances can it be used with non-TI connectivity device.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - HTTP Client Demo
// Application Overview - This sample application demonstrates how to use
//                          HTTP Client (In Minimum mode) API for HTTP based
//                          application development.
//                          This application explain user to how to:
//                          1. Connect to an access point
//                          2. Connect to a HTTP Server with and without proxy
//                          3. Do POST, GET, PUT and DELETE
//                          4. Parse JSON data using “Jasmine JSON Parser”
// Note: To use HTTP Client in minimum mode, user need to compile library (webclient)
// 			with HTTPCli_LIBTYPE_MIN option.
//
// 			HTTP Client (minimal) library supports synchronous mode, redirection
// 			handling, chunked transfer encoding support, proxy support and TLS
// 			support (for SimpleLink Only. TLS on other platforms are disabled)
//
// 			HTTP Client (Full) library supports all the features of the minimal
// 			library + asynchronous mode and content handling support +
// 			TLS support (all platforms). To use HTTP Client in full mode user need
//			to compile library (webclient) with HTTPCli_LIBTYPE_MIN option. For full
//			mode RTOS is needed.
//
//*****************************************************************************

#include <string.h>

// SimpleLink includes
#include "simplelink.h"

// driverlib includes
#include "hw_ints.h"
#include "hw_types.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "utils.h"
#include "interrupt.h"

// common interface includes
#include "uart_if.h"
#include "common.h"
#include "pinmux.h"

// HTTP Client lib
#include <http/client/httpcli.h>
#include <http/client/common.h>

// JSON Parser
#include "jsmn.h"

#define APPLICATION_VERSION "1.4.0"
#define APP_NAME            "HTTP Client"

#define POST_REQUEST_URI "/post"
#define POST_DATA        "{\n\"name\":\"xyz\",\n\"address\":\n{\n\"plot#\":12,\n\"street\":\"abc\",\n\"city\":\"ijk\"\n},\n\"age\":30\n}"

#define DELETE_REQUEST_URI "/delete"

#define PUT_REQUEST_URI "/put"
#define PUT_DATA        "PUT request."

#define GET_REQUEST_URI "/get"

#define HOST_NAME "httpbin.org" //"<host name>"
#define HOST_PORT 80

#define PROXY_IP   <proxy_ip>
#define PROXY_PORT <proxy_port>

#define READ_SIZE     1450
#define MAX_BUFF_SIZE 1460

// Application specific status/error codes
typedef enum {
	/* Choosing this number to avoid overlap with host-driver's error codes */
	DEVICE_NOT_IN_STATION_MODE = -0x7D0,
	DEVICE_START_FAILED = DEVICE_NOT_IN_STATION_MODE - 1,
	INVALID_HEX_STRING = DEVICE_START_FAILED - 1,
	TCP_RECV_ERROR = INVALID_HEX_STRING - 1,
	TCP_SEND_ERROR = TCP_RECV_ERROR - 1,
	FILE_NOT_FOUND_ERROR = TCP_SEND_ERROR - 1,
	INVALID_SERVER_RESPONSE = FILE_NOT_FOUND_ERROR - 1,
	FORMAT_NOT_SUPPORTED = INVALID_SERVER_RESPONSE - 1,
	FILE_OPEN_FAILED = FORMAT_NOT_SUPPORTED - 1,
	FILE_WRITE_ERROR = FILE_OPEN_FAILED - 1,
	INVALID_FILE = FILE_WRITE_ERROR - 1,
	SERVER_CONNECTION_FAILED = INVALID_FILE - 1,
	GET_HOST_IP_FAILED = SERVER_CONNECTION_FAILED - 1,

	STATUS_CODE_MAX = -0xBB8
} e_AppStatusCodes;

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile unsigned long g_ulStatus = 0;              // SimpleLink Status
unsigned long g_ulDestinationIP;                    // IP address of destination server
unsigned long g_ulGatewayIP = 0;                    // Network Gateway IP address
unsigned char g_ucConnectionSSID[SSID_LEN_MAX + 1]; // Connection SSID
unsigned char g_ucConnectionBSSID[BSSID_LEN_MAX];   // Connection BSSID
unsigned char g_buff[MAX_BUFF_SIZE + 1];
long bytesReceived = 0; // variable to store the file size

#if defined(ccs) || defined(gcc)
extern void (*const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************

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
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************

//*****************************************************************************
//
//! \brief This function initializes the application variables
//!
//! \param    None
//!
//! \return None
//!
//*****************************************************************************
static void InitializeAppVariables() {
	g_ulStatus = 0;
	g_ulGatewayIP = 0;
	memset(g_ucConnectionSSID, 0, sizeof(g_ucConnectionSSID));
	memset(g_ucConnectionBSSID, 0, sizeof(g_ucConnectionBSSID));
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
static long ConfigureSimpleLinkToDefaultState() {
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
	if (ROLE_STA != lMode) {
		if (ROLE_AP == lMode) {
			// If the device is in AP mode, we need to wait for this event
			// before doing anything
			while (!IS_IP_ACQUIRED(g_ulStatus)) {
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
		if (ROLE_STA != lRetVal) {
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

	//
	// Device in station-mode. Disconnect previous connection if any
	// The function returns 0 if 'Disconnected done', negative number if already
	// disconnected Wait for 'disconnection' event if 0 is returned, Ignore
	// other return-codes
	//
	lRetVal = sl_WlanDisconnect();
	if (0 == lRetVal) {
		// Wait
		while (IS_CONNECTED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
			_SlNonOsMainLoopTask();
#endif
		}
	}

	// Enable DHCP client
	lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE, 1, 1, &ucVal);
	ASSERT_ON_ERROR(lRetVal);

	// Disable scan
	ucConfigOpt = SL_SCAN_POLICY(0);
	lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN, ucConfigOpt, NULL, 0);
	ASSERT_ON_ERROR(lRetVal);

	// Set Tx power level for station mode
	// Number between 0-15, as dB offset from max power - 0 will set max power
	ucPower = 0;
	lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,
	                     WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
	ASSERT_ON_ERROR(lRetVal);

	// Set PM policy to normal
	lRetVal = sl_WlanPolicySet(SL_POLICY_PM, SL_NORMAL_POLICY, NULL, 0);
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

	return SUCCESS;
}

void ConfigureAP() {
	long retVal;

	retVal = sl_Start(0, 0, 0);

	if (retVal != ROLE_AP) {
		retVal = sl_WlanSetMode(ROLE_AP);
		sl_Stop(0);
		retVal = sl_Start(0, 0, 0);
	}

	char ssid[] = "CC3200_AP";

	sl_WlanSet(SL_WLAN_CFG_AP_ID,
	           WLAN_AP_OPT_SSID,
	           strlen(ssid),
	           (unsigned char *)ssid);

	char password[] = "mypassword";

	sl_WlanSet(SL_WLAN_CFG_AP_ID,
	           WLAN_AP_OPT_SECURITY_TYPE,
	           1,
	           (unsigned char[]){SL_SEC_TYPE_WPA_WPA2});

	sl_WlanSet(SL_WLAN_CFG_AP_ID,
	           WLAN_AP_OPT_PASSWORD,
	           strlen(password),
	           (unsigned char *)password);

	sl_Stop(0);
	sl_Start(0, 0, 0);
}

//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void
DisplayBanner(char *AppName) {
	UART_PRINT("\n\n\n\r");
	UART_PRINT("\t\t *************************************************\n\r");
	UART_PRINT("\t\t      CC3200 %s Application       \n\r", AppName);
	UART_PRINT("\t\t *************************************************\n\r");
	UART_PRINT("\n\n\n\r");
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void) {
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
	//
	// Set vector table base
	//
#if defined(ccs) || defined(gcc)
	MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
	MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
	//
	// Enable Processor
	//
	MAP_IntMasterEnable();
	MAP_IntEnable(FAULT_SYSTICK);

	PRCMCC3200MCUInit();
}

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

int main() {
	long lRetVal = -1;
	HTTPCli_Struct httpClient;

	//
	// Board Initialization
	//
	BoardInit();

	//
	// Configure the pinmux settings for the peripherals exercised
	//
	PinMuxConfig();

	//
	// Configuring UART
	//
	InitTerm();

	//
	// Display banner
	//
	DisplayBanner(APP_NAME);

	InitializeAppVariables();

	ConfigureAP();

	// Stop the CC3200 device

	LOOP_FOREVER();
}
