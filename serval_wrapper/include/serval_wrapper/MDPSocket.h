/*
 * MDPSocket.h
 *
 *  Created on: May 2, 2016
 *      Author: sni
 */

#ifndef MDPSOCKET_H_
#define MDPSOCKET_H_

#include <cstring>
#include <string>

#include <mdp_cpp.h>
#include <mutex>

namespace ice
{

class MDPSocket
{
public:
  MDPSocket(int socket, int port, std::string const &senderSid);
  virtual ~MDPSocket();

  void send(uint8_t *recipientSid, uint8_t *payload, size_t size);
  void send(std::string const &recipientSid, uint8_t *payload, size_t size);
  int receive(std::string &senderSid, uint8_t *buffer, size_t size, unsigned long timeoutMs = 1000);
  void close();

private:
  int socket;
  int port;
  std::string const senderSid;
  bool closed;
  std::mutex _mtx;
};

} /* namespace ice */

#endif /* MDPSOCKET_H_ */
