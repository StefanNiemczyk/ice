/*
 * ServalCommunication.cpp
 *
 *  Created on: Apr 15, 2016
 *      Author: sni
 */

#include "ServalCommunication.h"

#include <chrono>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <ice/communication/messages/Message.h>
#include <serval_interface.h>
#include <serval_wrapper/MDPSocket.h>

#include "ice/Entity.h"
#include "IceServalBridge.h"

// Short alias for this namespace
namespace pt = boost::property_tree;

namespace ice
{

ServalCommunication::ServalCommunication(std::weak_ptr<ICEngine> engine, std::string configPath, std::string const host,
                                         int const port, std::string const authName, std::string const authPass,
                                         bool const local) :
    CommunicationInterface(engine), configPath(configPath), host(host), port(port), authName(authName),
    authPass(authPass), serval(nullptr), local(local), running (false)
{
  _log = el::Loggers::getLogger("ServalCommunication");
  this->maxMessageSend = 2;
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

  // get own id
  if (local)
  {
    auto oid = this->serval->keyring.addIdentity();

    if (oid == nullptr)
    {
      // error case, own id could not be determined
      throw(std::runtime_error("Own serval id could not be determined"));
    }

    this->ownSid = oid->sid;
    this->self->addId(EntityDirectory::ID_SERVAL, this->ownSid);
  }
  else
  {
    if (this->ownSid == "")
    {
      auto id = this->serval->keyring.getSelf();
      if (id == nullptr || id->size() == 0)
      {
        // error case, own id could not be determined
        throw(std::runtime_error("Own serval id could not be determined"));
      }

      this->ownSid = id->at(0).sid;
      this->self->addId(EntityDirectory::ID_SERVAL, this->ownSid);
    }
  }

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
    this->serval->cleanUp();
    this->serval = nullptr;
  }
}

void ServalCommunication::read()
{
  uint8_t buffer[4096];
  std::string sid;

  while (this->running)
  {
    int recCount = this->socket->receive(sid, buffer, 4096);

    if (false == this->running)
      return;

    if (recCount == 0)
    {
      // huge amount of emtpy messages, will be skipped
//      _log->debug("Received empty message from sid %v", sid);
      continue;
    }

    if (recCount < 0)
    {
      _log->info("Received broken message from sid %v", sid);
      continue;
    }

    auto entity = this->directory->lookup(EntityDirectory::ID_SERVAL, sid, false);

    if (entity == nullptr)
    {
      this->_log->info("Received message from unknown serval node '%v'", sid);

      // Create new instance and request ids
      entity = this->directory->create(EntityDirectory::ID_SERVAL, sid);
      // At the beginning each discovered node is expected to be an ice node
      entity->setAvailable(true);
      this->discoveredEntity(entity);
    }
    else if (sid == this->ownSid)
    {
      // own message, will not be processed
      this->_log->debug("Received own message, will be skipped");
      continue;
    }

    entity->setActiveTimestamp();

    this->traffic.receivedBytes += recCount;
    ++this->traffic.messageReceivedCount;
    std::string json(buffer+3, buffer+recCount);

    auto message = Message::parse(buffer[0], json, this->containerFactory);
    message->setJobId(buffer[1]);
    message->setJobIndex(buffer[2]);

    if (message == nullptr)
    {
      this->_log->info("Received unknown or broken message from serval node '%v'", sid);

      continue;
    }

    message->setEntity(entity);
//    m.command = buffer[0];
//    m.payload.clear();
//    m.payload.reserve(recCount-1);
//    std::copy(buffer + 1, buffer + recCount, std::back_inserter(m.payload));

    this->handleMessage(message);
  }
}

void ServalCommunication::discover()
{
  std::unique_ptr<std::vector<serval_identity>> peers;
  if (this->local)
  {
    peers = this->serval->keyring.getSelf();
  }
  else
  {
    peers = this->serval->keyring.getPeerIdentities();
  }

  for (auto &sid : *peers)
  {
    auto entity = this->directory->lookup(EntityDirectory::ID_SERVAL, sid.sid);

    if (entity == nullptr)
    {
      // Create new instance and request ids
      entity = this->directory->create(EntityDirectory::ID_SERVAL, sid.sid);
      // At the beginning each discovered node is expected to be an ice node
      entity->setAvailable(true);
      this->discoveredEntity(entity);

      _log->info("New ID discovered: %v", entity->toString());
    }

    entity->setActiveTimestamp();
  }
}

int ServalCommunication::readMessage(std::vector<std::shared_ptr<Message>> &outMessages)
{
  return 0; // reading messages is done in own thread
}

void ServalCommunication::sendMessage(std::shared_ptr<Message> msg)
{
  std::string sid;

  if (false == msg->getEntity()->isAvailable())
  {
    this->_log->error("Dropped Msg: Trying to send message to not available serval instance %v", msg->getEntity()->toString());
    return;
  }

  if (false == msg->getEntity()->getId(EntityDirectory::ID_SERVAL, sid))
  {
    this->_log->error("Trying to send message with serval to none serval instance %v", msg->getEntity()->toString());
    return;
  }

  int size = 3;
  if (msg->isPayload())
  {
    std::string json = msg->toJson();
    size = json.size() + 3;

    if (size > 1024)
    {
      this->_log->error("Message could not be send to instance '%v', to large '%v' byte", msg->getEntity()->toString(), size);
      return;
    }

    std::lock_guard<std::mutex> guard(_mtxSend);
    std::copy(json.begin(), json.end(), this->buffer + 3);
  }

  buffer[0] = msg->getId();
  buffer[1] = msg->getJobId();
  buffer[2] = msg->getJobIndex();

  this->traffic.sendBytes += size;
  ++this->traffic.messageSendCount;
  this->socket->send(sid, buffer, size);
//  std::cout << size << std::endl;
}


std::shared_ptr<BaseInformationSender> ServalCommunication::createSender(std::shared_ptr<InformationCollection> collection)
{
  return nullptr;
}

std::shared_ptr<InformationReceiver> ServalCommunication::createReceiver(std::shared_ptr<InformationCollection> collection)
{
  return nullptr;
}

} /* namespace ice */
