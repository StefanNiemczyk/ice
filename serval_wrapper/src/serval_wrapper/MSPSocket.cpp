#include "serval_wrapper/MSPSocket.h"

#include <iostream>
#include <sys/time.h>
#include <sys/socket.h>

#include <serval_interface.h>

namespace ice {

MSPSocket::MSPSocket(int mdp_sock, int port, std::string const &senderSid) :
		mdp_sock(mdp_sock), port(port), senderSid(senderSid), closed(false)
{
	msp_sock = msp_socket(mdp_sock, 0);
}

MSPSocket::~MSPSocket()
{
	this->close();
}

int MSPSocket::connect(std::string recipientSid)
{
	std::lock_guard<std::mutex>(this->_mtx);
	struct mdp_sockaddr addr;

	bzero(&addr, sizeof(addr));
	addr.port = port;
	serval_interface::sidToArray(recipientSid, addr.sid.binary);

	msp_connect(msp_sock, &addr);
	if (!msp_socket_is_open(msp_sock)) {
		return -1;
	}

	msp_set_handler(msp_sock, ice::MSPSocket::io_handler, (void*) this);

	return 0;
}

int MSPSocket::listen(MSP_HANDLER *handler)
{
	std::lock_guard<std::mutex>(this->_mtx);
	struct mdp_sockaddr addr;

	bzero(&addr, sizeof(addr));
	addr.port = port;
	serval_interface::sidToArray(this->senderSid, addr.sid.binary);

	msp_set_local(msp_sock, &addr);
	msp_set_handler(msp_sock, handler, (void*) this);

	return msp_listen(msp_sock);
}

int MSPSocket::write(uint8_t *payload, int len)
{
	if (closed)
		return -1;

	uint8_t *pl = new uint8_t[len];
	memcpy((void*) pl, payload, len);
	sendQ.push(std::pair<uint8_t*, int>(pl, len));

	return 0;
}

std::pair<uint8_t*, int> MSPSocket::read()
{
	if (closed || recvQ.empty())
		return std::pair<uint8_t*, int>(nullptr, 0);

	std::pair<uint8_t*, int> p = recvQ.front();
	recvQ.pop();

	return p;
}

time_t MSPSocket::process(int timeout)
{
	if (closed)
		return -1;

	time_t next;
	struct timeval timeout_val;
	timeout_val.tv_sec = 0;
	timeout_val.tv_usec = timeout;

	// this blocks for timeout microseconds
	setsockopt(mdp_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout_val, sizeof(timeout_val));
	msp_recv(mdp_sock);
	msp_processing(&next);

	return next;
}

size_t MSPSocket::listen_handler(MSP_SOCKET sock, msp_state_t state, const uint8_t *payload,
                size_t len, void *context)
{
	MSPSocket *s = (MSPSocket *) context;
	size_t ret = 0;

	return ret;
}

size_t MSPSocket::io_handler(MSP_SOCKET sock, msp_state_t state, const uint8_t *payload, size_t len,
                void *context)
{
	MSPSocket *s = (MSPSocket *) context;
	size_t ret = 0;

	if (state & MSP_STATE_ERROR) {
		msp_stop(sock);
	}

	// receive messages
	if (payload && len) {
		// Put received bytes in queue to retrieve via read
		uint8_t *pl = new uint8_t[len];
		memcpy((void*) pl, payload, len);
		s->recvQ.push(std::pair<uint8_t*, int>(pl, len));

		ret = len;
	}

	// send messages
	if (!s->sendQ.empty() && (state & MSP_STATE_DATAOUT)) {
		std::pair<uint8_t*, int> buff = s->sendQ.back();
		s->sendQ.pop();
		msp_send(sock, buff.first, buff.second);
	}

	if (state & MSP_STATE_SHUTDOWN_REMOTE) {
		// Remote party has closed the connection; no more messages will arrive.
		s->close();
	}

	if (state & MSP_STATE_CLOSED) {
		// Release all resources associated with this connection.
		s->close();
	}

	return ret;
}

int MSPSocket::getMDPSocket()
{
	return mdp_sock;
}

MSP_SOCKET MSPSocket::getMSPSocket()
{
	return msp_sock;
}

void MSPSocket::close()
{
	if (closed)
		return;
	closed = true;

	msp_close_all(this->mdp_sock);
}

} /* namespace ice */
