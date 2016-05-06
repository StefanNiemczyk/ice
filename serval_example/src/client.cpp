#include <iostream>
#include <cstdio>

#include <msp_cpp.h>

#define PAYLOAD_SIZE 16
#define PORT 8042

static int quit;

size_t
io_handler(MSP_SOCKET sock, msp_state_t state,
		const uint8_t *payload, size_t len, void *context)
{
	int ret = 0;
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
	time_ms_t now;
	time_ms_t next_time = 0;

	if ((mdp_sock = mdp_socket()) < 0) {
		std::cerr << "error creating socket" << std::endl;
		return -1;
	}
	msp_sock = msp_socket(mdp_sock, 0);

	/* TODO: How to get server sid? */
	sockaddr.sid = SID_ANY;
	sockaddr.port = PORT;

	msp_connect(msp_sock, &sockaddr);
	if (!msp_socket_is_open(msp_sock)) {
		std::cerr << "error opening msp socket" << std::endl;
		return -1;
	}

	msp_set_handler(msp_sock, io_handler, NULL);

	quit = 0;
	while (!quit) {
		now = gettime_ms();
		if (now < next_time) {
			struct timeval timeout = time_ms_to_timeval(next_time - now);
			setsockopt(mdp_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
			msp_recv(mdp_sock);
		}

		msp_processing(&next_time);
	}

	msp_close_all(mdp_sock);
	mdp_close(mdp_sock);
	return 0;
}
