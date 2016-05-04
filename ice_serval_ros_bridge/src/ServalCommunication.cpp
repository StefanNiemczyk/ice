/*
 * ServalCommunication.cpp
 *
 *  Created on: Apr 15, 2016
 *      Author: sni
 */

#include "ServalCommunication.h"

#include <chrono>
#include <serval_interface.h>
#include <serval_wrapper/MDPSocket.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "Entity.h"
#include "IceServalBridge.h"

#define LOCAL_TEST true

// Short alias for this namespace
namespace pt = boost::property_tree;

namespace ice
{

ServalCommunication::ServalCommunication(IceServalBridge *bridge, std::string configPath, std::string const host,
                                         int const port, std::string const authName, std::string const authPass) :
    CommunicationInterface(), configPath(configPath), host(host), port(port), authName(authName), authPass(authPass), serval(nullptr)
{
  _log = el::Loggers::getLogger("ServalCommunication");
  this->bridge = bridge;
  this->directory = bridge->identityDirectory;
}

ServalCommunication::~ServalCommunication()
{
  if (this->running)
  {
    this->running = false;
  }
}

std::shared_ptr<serval_interface> ServalCommunication::getServalInterface()
{
  return this->serval;
}

void ServalCommunication::setOwnSid(std::string const &sid)
{
  this->ownSid = sid;
}

void ServalCommunication::initInternal()
{
  // create interface
  this->serval = std::make_shared<serval_interface>(this->configPath, this->host, this->port, this->authName, this->authPass);
  this->self = this->directory->self;

  // get own id
#ifdef LOCAL_TEST
  this->ownSid = this->serval->keyring.addIdentity()->sid;
  this->self->addId(EntityDirectory::ID_SERVAL, this->ownSid);
#else
  if (this->ownSid == "")
  {
    auto id = this->serval->keyring.getSelf();
    if (id->size() == 0)
    {
      // error case, own id could not be determined
      throw (std::runtime_error("Own serval id could not be determined"));
    }

    this->ownSid = id->at(0).sid;
    this->self->addId(EntityDirectory::ID_SERVAL, this->ownSid);
  }
#endif

  // create socket
  this->socket = this->serval->createSocket(SERVAL_PORT, this->ownSid);

  if (this->socket == nullptr)
  {
    throw (std::runtime_error("MDP socket could not be created"));
    return;
  }

  // init read thread
  this->running = true;
  this->worker = std::thread(&ServalCommunication::read, this);
  this->worker.detach();
}

void ServalCommunication::cleanUpInternal()
{
  this->running = false;

  if (this->serval != nullptr)
  {
    this->serval = nullptr;
  }
}

void ServalCommunication::read()
{
  uint8_t buffer[1024];
  std::string sid;
  Message m;

  while (this->running)
  {
    int count;

    int recCount = this->socket->receive(sid, buffer, 1024);

    if (false == this->running)
      return;

    if (recCount == 0)
    {
      _log->info("Received empty message from sid %s", sid);
      continue;
    }

    if (recCount < 0)
    {
      _log->info("Received broken message from sid %s", sid);
      continue;
    }

    auto entity = this->directory->lookup(EntityDirectory::ID_SERVAL, sid, false);

    if (entity == nullptr)
    {
      this->_log->info("Received message from unknown serval node '%s'");

      // Create new instance and request ids
      entity = this->directory->create(EntityDirectory::ID_SERVAL, sid);
      // At the beginning each discovered node is expected to be an ice node
      entity->setAvailable(true);
      this->requestIds(entity);
    }

    m.entity = entity;
    m.command = buffer[0];
    m.payload.clear();
    m.payload.reserve(recCount-1);
    std::copy(buffer + 1, buffer + recCount, std::back_inserter(m.payload));

    this->handleMessage(m);
  }
}

void ServalCommunication::discover()
{
#ifdef LOCAL_TEST
  auto peers = this->serval->keyring.getSelf();
#else
  auto peers = this->serval->keyring.getPeerIdentities();
#endif

  for (auto &sid : *peers)
  {
    auto entity = this->directory->lookup(EntityDirectory::ID_SERVAL, sid.sid);

    if (entity == nullptr)
    {
      // Create new instance and request ids
      entity = this->directory->create(EntityDirectory::ID_SERVAL, sid.sid);
      // At the beginning each discovered node is expected to be an ice node
      entity->setAvailable(true);
      this->requestIds(entity);

      std::cout << "New ID discovered: %s" << entity->toString() << std::endl;
      _log->info("New ID discovered: %s", entity->toString());
    }
  }
}

int ServalCommunication::readMessage(std::vector<Message> &outMessages)
{
  return 0; // reading messages is done in own thread
}

void ServalCommunication::sendMessage(Message &msg)
{
  std::string sid;

  if (false == msg.entity->getId(EntityDirectory::ID_SERVAL, sid))
  {
    this->_log->error("Trying to send message with serval to none serval instance %s", msg.entity->toString());
    return;
  }

  int size = msg.payload.size() + 1;
  unsigned char buffer[size];

  buffer[0] = msg.command;
  std::copy(msg.payload.begin(), msg.payload.end(), buffer + 1);

  this->socket->send(sid, buffer, size);
}

} /* namespace ice */
