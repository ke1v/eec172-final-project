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

	sl_SetSockOpt(sock, SL_SOL_SOCKET, SL_SO_BROADCAST, &broadcast, sizeof(broadcast));

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
		UART_PRINT("Socket failed\r\n");
		LOOP_FOREVER();
	}

	// Bind to all interfaces
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = SL_AF_INET;
	addr.sin_port = sl_Htons(UDP_PORT);
	addr.sin_addr.s_addr = 0; // Listen on all interfaces

	if (sl_Bind(sock, (SlSockAddr_t *)&addr, sizeof(addr)) < 0) {
		UART_PRINT("Bind failed\r\n");
		sl_Close(sock);
		LOOP_FOREVER();
	}

	UART_PRINT("Listening for UDP broadcasts on port %d...\n", UDP_PORT);

	while (!discoveredFlag) {
		int bytes = sl_RecvFrom(sock, buffer, sizeof(buffer) - 1, 0,
		                        (SlSockAddr_t *)&addr, &addrSize);
		if (bytes > 0) {
			buffer[bytes] = '\0';

			// Ignore own broadcast
			if (sender.sin_addr.s_addr != sl_Htonl(sl_NetCfgGet(SL_NETCFG_IPV4_ADDR))) {
				peerIp = sender.sin_addr.s_addr;
				discoveredFlag = 1;
				UART_PRINT("Discovered device at %d.%d.%d.%d: %s\n",
				           (sender.sin_addr.s_addr >> 24) & 0xFF,
				           (sender.sin_addr.s_addr >> 16) & 0xFF,
				           (sender.sin_addr.s_addr >> 8) & 0xFF,
				           sender.sin_addr.s_addr & 0xFF,
				           buffer);
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

void receive_udp_peer(char* recv_buffer, unsigned int buffer_size) {
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
			if (addr.sin_addr.s_addr == peerIp) {
				UART_PRINT("Message from peer: %s\n", buffer);
			}
		}
	}
}