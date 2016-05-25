#ifndef MSPSOCKET_H_
#define MSPSOCKET_H_

#include <cstring>
#include <string>
#include <mutex>
#include <queue>
#include <memory>

#include <msp_cpp.h>

namespace ice {

class MSPSocket {
public:
	MSPSocket(int mdp_socket, int port, std::string const &senderSid);
	MSPSocket(MSP_SOCKET sock);
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

	void close();
	int getMDPSocket();
	MSP_SOCKET getMSPSocket();

	// buffers all messages until retrieved using read
	std::queue<std::pair<uint8_t*, int>> recvQ;
	std::queue<std::pair<uint8_t*, int>> sendQ;

	std::shared_ptr<std::vector<std::shared_ptr<MSPSocket>>> connectionSockets = nullptr;

private:
	int mdp_sock;
	MSP_SOCKET msp_sock;

	int port;
	std::string const senderSid;

	std::mutex _mtx;

	static size_t io_handler(MSP_SOCKET sock, msp_state_t state, const uint8_t *payload,
	                size_t len, void *context);

	static size_t listen_handler(MSP_SOCKET sock, msp_state_t state, const uint8_t *payload,
	                size_t len, void *context);

};

} /* namespace ice */

#endif /* MSPSOCKET_H_ */
