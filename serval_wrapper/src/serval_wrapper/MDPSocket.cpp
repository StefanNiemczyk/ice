/*
 * MDPSocket.cpp
 *
 *  Created on: May 2, 2016
 *      Author: sni
 */

#include "serval_wrapper/MDPSocket.h"

#include <iostream>

#include "serval_interface.h"

namespace ice
{

MDPSocket::MDPSocket(int socket, int port, std::string const &senderSid)
  : socket(socket), port(port), senderSid(senderSid), closed(false)
{

}

MDPSocket::~MDPSocket()
{
  this->close();
}

void MDPSocket::send(uint8_t *recipientSid, uint8_t *payload, size_t size)
{
  std::lock_guard<std::mutex>(this->_mtx);

  struct mdp_header header;
  bzero(&header, sizeof(header));

  serval_interface::sidToArray(this->senderSid, header.local.sid.binary);
  std::copy(recipientSid, recipientSid + 32, std::begin(header.remote.sid.binary));
  header.remote.port = port;
  header.qos = OQ_MESH_MANAGEMENT;
  header.ttl = PAYLOAD_TTL_DEFAULT;
  header.flags |= MDP_FLAG_BIND | MDP_FLAG_NO_CRYPT;

  mdp_send(this->socket, &header, payload, size);
}

void  MDPSocket::send(std::string const &recipientSid, uint8_t *payload, size_t size)
{
  std::lock_guard<std::mutex>(this->_mtx);

  struct mdp_header header;
  bzero(&header, sizeof(header));

  serval_interface::sidToArray(this->senderSid, header.local.sid.binary);
  serval_interface::sidToArray(recipientSid, header.remote.sid.binary);
  header.remote.port = port;
  header.qos = OQ_MESH_MANAGEMENT;
  header.ttl = PAYLOAD_TTL_DEFAULT;
  header.flags |= MDP_FLAG_BIND | MDP_FLAG_NO_CRYPT;

  mdp_send(this->socket, &header, payload, size);
}

int MDPSocket::receive(std::string &senderSid, uint8_t *buffer, size_t size)
{
  struct mdp_header header;
  int count = mdp_recv(this->socket, &header, buffer, size);
  senderSid = serval_interface::arrayToSid(header.remote.sid.binary);

  return count;
}

void MDPSocket::close()
{
  if (this->closed)
    return;
  this->closed = true;

  mdp_close(this->socket);
}

} /* namespace ice */
