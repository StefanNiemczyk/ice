/*
 * MDPSocket.cpp
 *
 *  Created on: May 2, 2016
 *      Author: sni
 */

#include "serval_wrapper/MDPSocket.h"

#include <iostream>
#include <sys/time.h>

#include "serval_interface.h"

namespace ice
{

MDPSocket::MDPSocket(int port, std::string const &senderSid)
  : port(port), senderSid(senderSid), closed(false), sendCounter(0)
{
  // send
  if ((this->socketSend = mdp_socket()) < 0)
  {
    std::cerr << "Error creating send socket" << std::endl;
    this->valid = false;
    return;
  }

  // receive
  if ((this->socketReceive = mdp_socket()) < 0)
  {
    std::cerr << "Error creating receive socket" << std::endl;
    this->valid = false;
    return;
  }

  // binding socket to port
  struct mdp_sockaddr sockaddr;
  sockaddr.port = port;
  serval_interface::sidToArray(senderSid, sockaddr.sid.binary);

  if (mdp_bind(this->socketReceive, &sockaddr) != 0)
  {
    std::cerr << "Error binding socket to port '" << std::to_string(port) << std::endl;
    this->valid = false;
    return;
  }

  this->valid = true;
}

MDPSocket::~MDPSocket()
{
  this->close();
}

void MDPSocket::send(uint8_t *recipientSid, uint8_t *payload, size_t size)
{
  std::lock_guard<std::mutex>(this->_mtx);
  this->checkSendCounter();

  struct mdp_header header;
  bzero(&header, sizeof(header));

  serval_interface::sidToArray(this->senderSid, header.local.sid.binary);
  std::copy(recipientSid, recipientSid + 32, std::begin(header.remote.sid.binary));
  header.remote.port = port;
  header.qos = OQ_MESH_MANAGEMENT;
  header.ttl = PAYLOAD_TTL_DEFAULT;
  header.flags |= MDP_FLAG_BIND | MDP_FLAG_NO_CRYPT;

  mdp_send(this->socketSend, &header, payload, size);
}

void MDPSocket::send(std::string const &recipientSid, uint8_t *payload, size_t size)
{
  std::lock_guard<std::mutex>(this->_mtx);
  this->checkSendCounter();

  struct mdp_header header;
  bzero(&header, sizeof(header));

  serval_interface::sidToArray(this->senderSid, header.local.sid.binary);
  serval_interface::sidToArray(recipientSid, header.remote.sid.binary);
  header.remote.port = port;
  header.qos = OQ_MESH_MANAGEMENT;
  header.ttl = PAYLOAD_TTL_DEFAULT;
  header.flags |= MDP_FLAG_BIND | MDP_FLAG_NO_CRYPT;

  mdp_send(this->socketSend, &header, payload, size);
}

int MDPSocket::receive(std::string &senderSid, uint8_t *buffer, size_t size, unsigned long timeoutMs)
{
  struct mdp_header header;
//  struct timeval tim;
//  gettimeofday(&tim, NULL);
//  uint64_t timeout = (tim.tv_sec * (uint64_t)1000) + (tim.tv_usec / 1000) + timeoutMs;

  int count = mdp_recv(this->socketReceive, &header, buffer, size);
//  int count = mdp_poll_recv(this->socket, timeoutMs, &header, buffer, size);
  senderSid = serval_interface::arrayToSid(header.remote.sid.binary);

  return count;
}

void MDPSocket::close()
{
  if (this->closed || false == this->valid)
    return;
  this->closed = true;

  mdp_close(this->socketSend);
  mdp_close(this->socketReceive);
}

bool MDPSocket::isValid()
{
  return this->valid;
}

void MDPSocket::checkSendCounter()
{
  this->sendCounter++;

  if (this->sendCounter >= 75)
  {
    mdp_close(this->socketSend);

    if ((this->socketSend = mdp_socket()) < 0)
    {
      std::cerr << "Error creating send socket" << std::endl;
      this->valid = false;
      return;
    }
  }
}

} /* namespace ice */
