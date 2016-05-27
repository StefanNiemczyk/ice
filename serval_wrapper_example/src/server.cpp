#include <iostream>
#include <cstring>
#include <signal.h>

#include <serval_interface.h>

#define PORT 8042

static int quit;

void
cleanup(int sig)
{
	quit  = 1;
}

using namespace ice;
using namespace std;

int
main()
{
	int res;
	char msg[] = "Hello Client!";
	int sock;
	struct sigaction act;
	struct mdp_sockaddr addr;
	struct mdp_sockaddr remote_addr;
	time_ms_t now = 0;
	time_ms_t next_time = 0;

	if ((sock = mdp_socket()) < 0) {
		std::cerr << "error creating mdp socket" << std::endl;
		return -1;
	}

	overlay_mdp_getmyaddr(sock, 0, &addr.sid);

	MSPSocket server(sock, PORT, serval_interface::arrayToSid(addr.sid.binary));

	if (server.listen() < 0)
		return -1;

	/* cleanup on CTRL-C */
	memset(&act, 0, sizeof(act));
	act.sa_handler = &cleanup;
	if (sigaction(SIGINT, &act, NULL) < 0) {
		fprintf(stderr, "error registering signal handler\n");
		return -1;
	}

	/* main loop */
	std::cout << "server: starting server..." << std::endl;
	quit = 0;
	int len;
	uint8_t* payload;
	while (!quit && server.isOpen()) {
		server.process(10);

		for (MSPSocket *sock : *server.accept()) {
			pair<uint8_t*, int> p =  sock->read();

			payload = p.first;
			len = p.second;

			if (payload != nullptr) {
				msp_get_remote(sock->getMSPSocket(), &remote_addr);
				cout << "server received message from " << serval_interface::arrayToSid(remote_addr.sid.binary) << std::endl;
				// handle client connection
				cout << "server received message of length " << len << endl;
			}
		}

	}

	std::cout << "server: cleanup..." << std::endl;

	server.close();
	mdp_close(sock);

	return 0;
}
