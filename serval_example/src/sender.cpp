#include <iostream>
#include <cstring>

#include <mdp_cpp.h>

#define PAYLOAD_LENGTH 16
#define PORT 8042

int
main()
{
	int sock;
	int res, i;
	uint8_t payload[PAYLOAD_LENGTH];
	struct mdp_header hdr;
	bzero(&hdr, sizeof(hdr));

	if ((sock = mdp_socket()) < 0) {
		std::cerr << "error creating socket" << std::endl;
		return -1;
	}

	hdr.local.sid = BIND_PRIMARY;
	hdr.remote.sid = SID_BROADCAST;
	hdr.remote.port = PORT;
	hdr.qos = OQ_MESH_MANAGEMENT;
	hdr.ttl = PAYLOAD_TTL_DEFAULT;
	hdr.flags |= MDP_FLAG_BIND | MDP_FLAG_NO_CRYPT;

	for (i = 0; i < PAYLOAD_LENGTH; i++)
		payload[i] = i;

	for (i = 0; i < 10; i++)
		mdp_send(sock, &hdr, &payload[0], PAYLOAD_LENGTH);
	
	mdp_close(sock);
	return 0;
}
