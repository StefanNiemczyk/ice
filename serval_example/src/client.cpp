#include <iostream>
#include <cstdio>
#include <cstring>

#include <msp_cpp.h>

#define PAYLOAD_SIZE 16
#define PORT 8042

static int quit;

size_t
io_handler(MSP_SOCKET sock, msp_state_t state,
		const uint8_t *payload, size_t len, void *context)
{
	int ret = 0;

	/* process responses from server */
	if (payload && len) {
		std::cout << "received payload with len:" << len << std::endl;
		for (int i = 0; i < len; i++)
			std::cout << payload[i];

		std::cout << std::endl;
		ret = len;
	}
	if (ret == len && (state & MSP_STATE_SHUTDOWN_REMOTE)) {
		std::cout << "received EOF" << std::endl;
	}

	/*
	if ( outlen == 0 &&  )
		msp_shutdown(sock);
	else if (state & MSP_STATE_DATA_OUT) {
		ssize_t sent = msp_send(sock, outbuf, outlen);
	    if (sent == -1)
	        msp_shutdown(sock); // premature end
	    else {
		
	    }
	}*/

	if (state & (MSP_STATE_CLOSED | MSP_STATE_ERROR)) {
		quit = 1;
	}

	return ret;
}

int
main()
{
	int mdp_sock;
	MSP_SOCKET msp_sock;
	struct mdp_sockaddr sockaddr;
	struct timeval timeout;
	time_ms_t now;
	time_ms_t next_time = 0;

	/* message to send to server */
	char msg[60];
	strcpy(msg, "Hello World");

	if ((mdp_sock = mdp_socket()) < 0) {
		std::cerr << "client: error creating mdp socket" << std::endl;
		return -1;
	}
	msp_sock = msp_socket(mdp_sock, 0);

	/* TODO: How to get server sid? */
	sid_t sid = {0x2D, 0xF7, 0xCB, 0x17, 0x09, 0x5B, 0xEC, 0xD4, 0x4A, 0x13, 0xA1, 0xA9, 0x6F, 0x9A, 0xE5, 0x95, 0xFF, 0x9D, 0x2D, 0xEB, 0x51, 0xE8, 0xA1, 0x72, 0x3C, 0x1A, 0xD6, 0x08, 0x82, 0x6A, 0xF4, 0x18};

	sockaddr.sid = sid;
	sockaddr.port = PORT;

	msp_connect(msp_sock, &sockaddr);
	if (!msp_socket_is_open(msp_sock)) {
		std::cerr << "client: error connecting msp socket..." << std::endl;
		return -1;	
	}

	msp_set_handler(msp_sock, io_handler, NULL);

	std::cerr << "client: starting client..." << std::endl;
	quit = 0;
	while (!quit) {
		now = gettime_ms();
		if (now < next_time) {
			std::cout << "client: sending msg..." << std::endl;
			msp_send(msp_sock, (uint8_t *) msg, strlen(msg));

			timeout = time_ms_to_timeval(next_time - now);
			setsockopt(mdp_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
			msp_recv(mdp_sock);
		}

		msp_processing(&next_time);
	}

	std::cout << "client: cleanup..." << std::endl;

	msp_shutdown(msp_sock);
	msp_close_all(mdp_sock);
	mdp_close(mdp_sock);
	return 0;
}
