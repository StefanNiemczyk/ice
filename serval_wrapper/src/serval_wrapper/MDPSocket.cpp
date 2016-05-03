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
  : socket(socket), port(port), recipientSid(recipientSid), senderSid(senderSid), closed(false)
{
  bzero(&this->header, sizeof(this->header));

  serval_interface::sidToArray(senderSid, this->header.local.sid.binary);
//  serval_interface::sidToArray(recipientSid, this->header.remote.sid.binary);
  this->header.remote.port = port;
  this->header.qos = OQ_MESH_MANAGEMENT;
  this->header.ttl = PAYLOAD_TTL_DEFAULT;
  this->header.flags |= MDP_FLAG_BIND | MDP_FLAG_NO_CRYPT;


  struct mdp_sockaddr sockaddr;
  sockaddr.port = port;
  serval_interface::sidToArray(senderSid, sockaddr.sid.binary);
  if (mdp_bind(this->socket, &sockaddr) != 0) {
          std::cerr << "error binding port" << std::endl;
  }
}

MDPSocket::~MDPSocket()
{
  this->close();
}

void MDPSocket::send(unsigned char *recipientSid, uint8_t *payload, size_t size)
{
  std::lock_guard<std::mutex>(this->_mtx);

  std::copy(recipientSid, recipientSid + 32, std::begin(this->header.remote.sid.binary));
//  this->header.remote.sid.binary = recipientSid;
  mdp_send(this->socket, &this->header, payload, size);
}

void  MDPSocket::send(std::string const &recipientSid, uint8_t *payload, size_t size)
{
  std::lock_guard<std::mutex>(this->_mtx);

  serval_interface::sidToArray(recipientSid, this->header.remote.sid.binary);
  mdp_send(this->socket, &this->header, payload, size);
}

int MDPSocket::receive(uint8_t *buffer, size_t size)
{
  return mdp_recv(this->socket, &this->header, buffer, size);
}

void MDPSocket::close()
{
  if (this->closed)
    return;
  this->closed = true;

  mdp_close(this->socket);
}

} /* namespace ice */
