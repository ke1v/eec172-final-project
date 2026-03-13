#ifndef __WIRELESS_H__
#define __WIRELESS_H__
#include <socket.h>

void controller_broadcast();
void listen_for_controller();
void send_udp_peer(const char *msg);
void receive_udp_peer(char *recv_buffer, unsigned int buffer_size);

#endif