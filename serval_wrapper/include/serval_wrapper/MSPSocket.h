#ifndef MSPSOCKET_H_
#define MSPSOCKET_H_

#include <cstring>
#include <string>
#include <mutex>
#include <queue>
#include <set>
#include <memory>

#include <msp_cpp.h>

namespace ice {

class MSPSocket {
public:
	MSPSocket(int mdp_socket, int port, std::string const &senderSid);
	MSPSocket(MSP_SOCKET sock, MSPSocket *parent);
	virtual ~MSPSocket();

	// ================================================
	// Client/Data Socket
	// ================================================
	// connect first as client
	int connect(std::string recipientSid);

	// read and write after connecting
	std::pair<uint8_t*, int> read(); /* returns buffer with its length as pair */
	int write(uint8_t* payload, int len);

	// returns time when it wants to be called again
	time_ms_t process(time_ms_t timeout);

	// ================================================
	// Server/Listening Socket
	// ================================================
	// go into listening mode
	int listen();

	// returns set with every connection/server socket to iterate over
	std::shared_ptr<std::set<MSPSocket*>> accept();

	void close();

	int getMDPSocket();
	MSP_SOCKET getMSPSocket();
	MSPSocket *getParent();
	bool isOpen();

	// buffers all messages until retrieved using read
	std::queue<std::pair<uint8_t*, int>> recvQ;
	std::queue<std::pair<uint8_t*, int>> sendQ;

private:
	int mdp_sock;
	int port;
	MSP_SOCKET msp_sock;
	std::string const senderSid;
	MSPSocket *parent = nullptr;

	std::shared_ptr<std::set<MSPSocket*>> connectionSockets = nullptr;

	static size_t io_handler(MSP_SOCKET sock, msp_state_t state, const uint8_t *payload,
	                size_t len, void *context);
	static size_t listen_handler(MSP_SOCKET sock, msp_state_t state, const uint8_t *payload,
	                size_t len, void *context);
};

} /* namespace ice */

#endif /* MSPSOCKET_H_ */
