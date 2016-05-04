/*
 * CommunicationInterface.cpp
 *
 *  Created on: Apr 15, 2016
 *      Author: sni
 */

#include "CommunicationInterface.h"

#include <iostream>
#include <map>

#include "Entity.h"
#include "IceServalBridge.h"
#include "serialize.h"

namespace ice
{

CommunicationInterface::CommunicationInterface()
{
  _log = el::Loggers::getLogger("CommunicationInterface");

}

CommunicationInterface::~CommunicationInterface()
{
  if (this->running && this->worker.joinable())
  {
    this->running = false;
    this->worker.join();
  }
}

void CommunicationInterface::init()
{
  this->initInternal();

  this->running = true;
  this->worker = std::thread(&CommunicationInterface::workerTask, this);
}

void CommunicationInterface::cleanUp()
{
  this->running = false;
  this->worker.join();

  this->cleanUpInternal();
}

void CommunicationInterface::pushMessage(Message &message)
{
  std::lock_guard<std::mutex> guard(this->_mtx);

  this->messages.push_back(message);
}

void CommunicationInterface::requestId(std::shared_ptr<Entity> const &entity, std::string const &id)
{
  // TODO
}

void CommunicationInterface::onRequestId(std::shared_ptr<Entity> const &entity, std::string const &id)
{
  // TODO
}

void CommunicationInterface::requestIds(std::shared_ptr<Entity> const &entity)
{
  std::cout << "Requesting Ids from '%s'" << entity->toString() << std::endl;
  _log->info("Requesting Ids from '%s'", entity->toString());
  Message m;
  m.entity = entity;
  m.command = IceCmd::SCMD_IDS_REQUEST;

  this->pushMessage(m);
}

void CommunicationInterface::onRequestIds(std::shared_ptr<Entity> const &entity)
{
  std::cout << "Sending Ids to '%s'" << entity->toString() << std::endl;
  _log->info("Sending Ids to '%s'", entity->toString());
  Message m;
  m.entity = entity;
  m.command = IceCmd::SCMD_IDS_RESPONSE;

  std::vector<std::tuple<std::string, std::string>> vec;
  this->self->pushIds(vec);

  serialize(vec, m.payload);

  this->pushMessage(m);
}

void CommunicationInterface::requestOfferedInformation(std::shared_ptr<Entity> const &entity)
{
  std::cout << "Requesting offered information from '%s'" << entity->toString() << std::endl;
  _log->info("Requesting offered information from '%s'", entity->toString());
  Message m;
  m.entity = entity;
  m.command = IceCmd::SCMD_INFORMATION_REQUEST;

  this->pushMessage(m);
}

void CommunicationInterface::onRequestOfferedInformation(std::shared_ptr<Entity> const &entity)
{
  std::cout << "Sending required infos to '%s'" << entity->toString() << std::endl;
  _log->info("Sending required infos to '%s'", entity->toString());
  Message m;
  m.entity = entity;
  m.command = IceCmd::SCMD_INFORMATION_RESPONSE;

  std::vector<std::tuple<std::string, std::string, std::string, std::string, std::string>> specs;
  for (auto info : this->bridge->getOfferedInfos())
  {
    std::tuple<std::string, std::string, std::string, std::string, std::string> t(info->infoSpec.getEntity(),
                                                                                  info->infoSpec.getEntityType(),
                                                                                  info->infoSpec.getScope(),
                                                                                  info->infoSpec.getRepresentation(),
                                                                                  info->infoSpec.getRelatedEntity());
    specs.push_back(t);
  }

  serialize(specs, m.payload);

  this->pushMessage(m);
}

void CommunicationInterface::handleMessage(Message &message)
{
  std::cout << "Received Message with id '%s' from %s" << std::to_string(message.command) << " "<< message.entity->toString() << std::endl;
  _log->info("Received Message with id '%s' from %s", std::to_string(message.command), message.entity->toString());

  std::vector<std::tuple<std::string, std::string>> vector;

  switch (message.command)
  {
    case (SCMD_IDS_REQUEST):
      this->onRequestIds(message.entity);
      break;

    case (SCMD_IDS_RESPONSE):
      vector = deserialize<std::vector<std::tuple<std::string, std::string>>>(message.payload);
      message.entity->fuse(vector);
      message.entity->checkIce();
      break;

    case (SCMD_ID_REQUEST):
      // TODO
      break;

    case (SCMD_ID_RESPONSE):
      // TODO
      break;

    case (SCMD_INFORMATION_REQUEST):
      this->onRequestOfferedInformation(message.entity);
      break;

    case (SCMD_INFORMATION_RESPONSE):
//      identity->onResponseOfferedInformation(message.infos);
      break;

    default:
      _log->error("Unknown command '%s', message will be skipped", std::to_string(message.command));
      break;
  }
}

void CommunicationInterface::workerTask()
{
  int counter = 0;
  std::vector<Message> msgs;

  while (this->running)
  {
    // check all n iterations for new peers
    if (counter >= 100)
    {
      this->discover();
      this->directory->checkTimeout();

      counter = 0;
    }

    // check for new messages
    msgs.clear();
    int count = this->readMessage(msgs);
    for (auto &msg : msgs)
    {
      this->handleMessage(msg);
    }

    // send messages
    {
      std::lock_guard<std::mutex> guard(this->_mtx);

      for (auto &msg : this->messages)
      {
        if (false == this->running)
        {
          return;
        }

        this->sendMessage(msg);
      }

      this->messages.clear();
    }

    ++counter;
    // sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

} /* namespace ice */
