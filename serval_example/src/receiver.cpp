#include <iostream>
#include <cstdio>

#include <mdp_cpp.h>

#define PAYLOAD_SIZE 16
#define PORT 8042

int
main()
{
	int sock;
	int i, res;
	struct mdp_header hdr;
	struct mdp_sockaddr sockaddr;
	uint8_t payload[PAYLOAD_SIZE];
	sid_t my_sid;

	if ((sock = mdp_socket()) < 0) {
		std::cerr << "error creating socket" << std::endl;
		return -1;
	}

	sockaddr.port = PORT;
	sockaddr.sid = BIND_PRIMARY;
	if (mdp_bind(sock, &sockaddr) != 0) {
		std::cerr << "error binding port" << std::endl;
		return -1;
	}

	std::cout << "listening on port " << PORT << std::endl;
	for(;;) {
		res = mdp_recv(sock, &hdr, &payload[0], PAYLOAD_SIZE);
		
		std::cout << "package received(" << res << ")"<< std::endl;
		std::cout << "payload:" << std::endl;
		for (i = 0; i < PAYLOAD_SIZE; i++)
			printf("%d ", payload[i]);
		std::cout << std::endl;
	}

	mdp_close(sock);
	return 0;
}
