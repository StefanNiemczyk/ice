#include <iostream>
#include <cstring>

#include "gtest/gtest.h"
#include "serval_interface.h"

#define PAYLOAD_LENGTH 16
#define PORT 8042

static ice::serval_interface si1("/tmp/instance1", "localhost", 4110, "peter", "venkman");
static std::unique_ptr<ice::serval_identity> serverSid = si1.keyring.addIdentity();
static std::unique_ptr<ice::serval_identity> clientSid = si1.keyring.addIdentity();
static std::unique_ptr<ice::serval_identity> client2Sid = si1.keyring.addIdentity();

using ice::MSPSocket;

size_t io_handler(MSP_SOCKET sock, msp_state_t state, const uint8_t *payload, size_t len,
                void *context)
{
	MSPSocket *s = (MSPSocket *) context;
	size_t ret = 0;
	struct mdp_sockaddr addr;
	char msg[] = "Server answered";

	if (state & MSP_STATE_ERROR) {
		msp_stop(sock);
	}

	// receive messages
	if (payload && len) {
		msp_get_remote(sock, &addr);
		std::cout << "server received message from" << std::endl;

		std::cout << ice::serval_interface::arrayToSid(addr.sid.binary) << std::endl;
	}

	// send messages
	if (state & MSP_STATE_DATAOUT) {
		msp_send(sock, (uint8_t *) msg, strlen(msg));
	}

	if (state & MSP_STATE_SHUTDOWN_REMOTE) {
		// Remote party has closed the connection; no more messages will arrive.
	}

	if (state & MSP_STATE_CLOSED) {
		// Release all resources associated with this connection.
	}

	return ret;
}

size_t listen_handler(MSP_SOCKET sock, msp_state_t state, const uint8_t *payload, size_t len,
                void *context)
{
	if (state & (MSP_STATE_ERROR | MSP_STATE_CLOSED)) {
		// TODO: Handle closed?
	} else {
		msp_set_handler(sock, io_handler, NULL);
		if (payload && len)
			return io_handler(sock, state, payload, len, context);
	}

	return 0;
}


TEST(msp, server_client)
{
	auto server = si1.createMSPSocket(PORT, serverSid->sid);
	auto client = si1.createMSPSocket(PORT, clientSid->sid);
	int res;
	char msg[] = "Hello World";

	ASSERT_NE(nullptr, server);
	ASSERT_NE(nullptr, client);

	server->listen(&listen_handler);

	res = client->connect(serverSid->sid);
	ASSERT_EQ(res, 0);

	for (int i = 0; i < 10; i++) {
		client->write((uint8_t*) msg, strlen(msg));

		server->process(10);
		client->process(10);
	}

	std::pair<uint8_t*, int> p = client->read();
	for(;p.first != nullptr;) {
		p = client->read();
		std::cout << "Read message from Server: " << (char *) p.first << std::endl;
	}
	ASSERT_EQ(res, 0);

	server->close();
	client->close();
}
