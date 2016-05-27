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
main(int argc, char *argv[])
{
	int res;
	char msg[] = "Hello Server!";
	int sock;
	struct sigaction act;
	struct mdp_sockaddr addr;
	time_ms_t now = 0;
	time_ms_t next_time = 0;

	/* read sid from command line parameter */
	if (argc < 2) {
		std::cerr << "usage: " << argv[0] << " SERVER_SID" << std::endl;
		return -1;
	}

	if ((sock = mdp_socket()) < 0) {
		cerr << "error creating mdp socket" << endl;
		return -1;
	}

	overlay_mdp_getmyaddr(sock, 0, &addr.sid);

	MSPSocket client(sock, PORT, serval_interface::arrayToSid(addr.sid.binary));
	if (client.connect(std::string(argv[1])) < 0) {
		cerr << "error connecting to server" << endl;
		return -1;
	}

	/* cleanup on CTRL-C */
	memset(&act, 0, sizeof(act));
	act.sa_handler = &cleanup;
	if (sigaction(SIGINT, &act, NULL) < 0) {
		fprintf(stderr, "error registering signal handler\n");
		return -1;
	}

	/* main loop */
	std::cout << "client: starting client..." << std::endl;
	quit = 0;
	int len;
	uint8_t* payload;
	while (!quit && client.isOpen()) {
		client.process(10);

		client.write((uint8_t*) msg, strlen(msg));
	}

	std::cout << "client: cleanup..." << std::endl;

	client.close();
	mdp_close(sock);

	return 0;
}
