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

namespace ice
{

class MDPSocket
{
public:
  MDPSocket(int socket, int port, std::string const &recipientSid, std::string const &senderSid);
  virtual ~MDPSocket();

  void send(uint8_t *payload, size_t size);
  int receive(uint8_t *buffer, size_t size);
  void close();

private:
  int socket;
  int port;
  std::string const &recipientSid;
  std::string const &senderSid;
  struct mdp_header header;
  bool closed;
};

} /* namespace ice */

#endif /* MDPSOCKET_H_ */
