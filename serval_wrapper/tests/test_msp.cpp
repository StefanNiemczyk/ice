#include <iostream>
#include <cstring>

#include "gtest/gtest.h"
#include "serval_interface.h"

#define PAYLOAD_LENGTH 16
#define PORT 8042

using ice::MSPSocket;
using namespace std;

static ice::serval_interface si1("/tmp/instance1", "localhost", 4110, "peter", "venkman");
static unique_ptr<ice::serval_identity> serverSid = si1.keyring.addIdentity();
static unique_ptr<ice::serval_identity> clientSid = si1.keyring.addIdentity();

TEST(msp, server_client)
{
	auto server = si1.createMSPSocket(PORT, serverSid->sid);
	auto client = si1.createMSPSocket(PORT, clientSid->sid);
	int res;
	char msg[] = "Hello World";
	bool connected = false;

	ASSERT_NE(nullptr, server);
	ASSERT_NE(nullptr, client);

	res = server->listen();
	ASSERT_EQ(res, 0);

	res = client->connect(serverSid->sid);
	ASSERT_EQ(res, 0);

	ASSERT_NE(server->accept(), nullptr);

	for (int i = 0; i < 100; i++) {
		server->process(10);
		client->process(10);

		client->write((uint8_t*) msg, strlen(msg));

		for (MSPSocket *sock : *server->accept()) {
			pair<uint8_t*, int> p = sock->read();
			if (p.first != nullptr) {
				// handle client connection
				cout << "server received message of length " << p.second << endl;
			}
		}
	}

	server->close();
	client->close();
}
