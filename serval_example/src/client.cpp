#include <iostream>
#include <cstdio>
#include <cstring>
#include <signal.h>

#include <msp_cpp.h>

#define PAYLOAD_SIZE 16
#define PORT 8042
#define MSP_MESSAGE_SIZE 32

static int quit;

int outlen;
char outbuf[MSP_MESSAGE_SIZE];

size_t
io_handler(MSP_SOCKET sock, msp_state_t state,
		const uint8_t *payload, size_t len, void *context)
{
	/* msp_sock is a new(connection) socket */
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

	/* eventully send response to server */
	outlen = 0; // lenght of that response
	// eventually fill outbuf[];

	if (outlen == 0) {
		msp_shutdown(sock);
	} else if (state & MSP_STATE_DATAOUT) {
		ssize_t sent = msp_send(sock, (uint8_t *)outbuf, outlen);

		/* if there was an error while sending? */
		if (sent == -1)
			msp_shutdown(sock);
	}

	/* quit program on error */
	if (state & (MSP_STATE_CLOSED | MSP_STATE_ERROR))
		quit = 1;

	return ret;
}

extern "C" {
int str_to_sid_t(sid_t *sid, const char *hex);
}

void
cleanup(int signal)
{
	quit = 1;
}

int
main(int argc, char *argv[])
{
	int mdp_sock;
	MSP_SOCKET msp_sock;
	sid_t sid;
	struct mdp_sockaddr sockaddr;
	struct timeval timeout;
	struct sigaction act;
	time_ms_t now;
	time_ms_t next_time = 0;
	char msg[] = "Hello Server!";

	/* read sid from command line parameter */
	if (argc < 2) {
		std::cerr << "usage: " << argv[0] << " SERVER_SID" << std::endl;
		return -1;
	}

	if (str_to_sid_t(&sid, argv[1]) < 0) {
		std::cerr << "error: invalid sid parameter" << std::endl;
		return -1;
	}
	sockaddr.sid = sid;
	sockaddr.port = PORT;

	if ((mdp_sock = mdp_socket()) < 0) {
		std::cerr << "client: error creating mdp socket" << std::endl;
		return -1;
	}
	msp_sock = msp_socket(mdp_sock, 0);

	msp_connect(msp_sock, &sockaddr);
	if (!msp_socket_is_open(msp_sock)) {
		std::cerr << "client: error connecting msp socket..." << std::endl;
		return -1;	
	}

	msp_set_handler(msp_sock, io_handler, NULL);

	/* cleanup on CTRL-C */
	memset(&act, 0, sizeof(act));
	act.sa_handler = &cleanup;
	if (sigaction(SIGINT, &act, NULL) < 0) {
		fprintf(stderr, "error registering signal handler\n");
		return -1;
	}
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
	msp_close_all(mdp_sock);
	mdp_close(mdp_sock);

	return 0;
}
