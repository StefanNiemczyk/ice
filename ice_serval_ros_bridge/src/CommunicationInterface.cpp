/*
 * CommunicationInterface.cpp
 *
 *  Created on: Apr 15, 2016
 *      Author: sni
 */

#include "CommunicationInterface.h"

#include <iostream>
#include <map>

#include <ice/information/InformationElement.h>
#include <ice/information/InformationStore.h>
#include <ice/representation/GContainer.h>

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
  _log->info("Requesting Ids from '%v'", entity->toString());
  Message m;
  m.entity = entity;
  m.command = IceCmd::SCMD_IDS_REQUEST;

  this->pushMessage(m);
}

void CommunicationInterface::onRequestIds(std::shared_ptr<Entity> const &entity)
{
  _log->info("Sending Ids to '%v'", entity->toString());
  Message m;
  m.entity = entity;
  m.command = IceCmd::SCMD_IDS_RESPONSE;

  std::vector<std::tuple<std::string, std::string>> vec;
  this->self->pushIds(vec);

  serialize(vec, m.payload);

  this->pushMessage(m);
}

void CommunicationInterface::requestOffers(std::shared_ptr<Entity> const &entity)
{
  _log->info("Requesting offered information from '%v'", entity->toString());
  Message m;
  m.entity = entity;
  m.command = IceCmd::SCMD_OFFERS_REQUEST;

  this->pushMessage(m);
}

void CommunicationInterface::onRequestOffers(std::shared_ptr<Entity> const &entity)
{
  _log->info("Sending offered information to '%v'", entity->toString());
  Message m;
  m.entity = entity;
  m.command = IceCmd::SCMD_OFFERS_RESPONSE;

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

void CommunicationInterface::requestInformation(std::shared_ptr<Entity> const &entity,
                                std::vector<std::shared_ptr<InformationSpecification>> const &requests)
{
  _log->info("Requesting informations from '%v'", entity->toString());
  Message m;
  m.entity = entity;
  m.command = IceCmd::SCMD_INFORMATION_REQUEST;

  std::vector<std::tuple<std::string, std::string, std::string, std::string, std::string>> specs;
  for (auto &request : requests)
  {
    std::tuple<std::string, std::string, std::string, std::string, std::string> t(request->getEntity(),
                                                                                  request->getEntityType(),
                                                                                  request->getScope(),
                                                                                  request->getRepresentation(),
                                                                                  request->getRelatedEntity());
    specs.push_back(t);
  }

  serialize(specs, m.payload);

  this->pushMessage(m);
}

void CommunicationInterface::onRequestInformation(std::shared_ptr<Entity> const &entity,
                                  std::vector<std::tuple<std::string, std::string, std::string, std::string, std::string>> const &requests)
{
  _log->info("Informations request received from '%v'", entity->toString());

  std::vector<std::shared_ptr<InformationElement<std::shared_ptr<GContainer>>>> infos;
  std::vector<std::tuple<int,std::vector<std::vector<uint8_t>>>> gcontainers;

  for (int i=0; i < requests.size(); ++i)
  {
    infos.clear();
    auto t = requests.at(i);
    auto request = std::make_shared<InformationSpecification>(std::get<0>(t),
                                                              std::get<1>(t),
                                                              std::get<2>(t),
                                                              std::get<3>(t),
                                                              std::get<4>(t));

    this->bridge->informationStore->getInformation(request, infos);

    for (auto &info : infos)
    {
      std::vector<std::vector<uint8_t>> vec;
      info->getInformation()->toByte(vec);
      gcontainers.push_back(std::make_tuple(i,vec));
    }
  }

  Message m;
  m.entity = entity;
  m.command = IceCmd::SCMD_INFORMATION_RESPONSE;

  serialize(infos, m.payload);

  this->sendMessage(m);
}

void CommunicationInterface::handleMessage(Message &message)
{
  _log->info("Received Message with id '%v' from %v", std::to_string(message.command), message.entity->toString());
  message.entity->setActiveTimestamp();

  std::vector<std::tuple<std::string, std::string>> ids;
  std::vector<std::tuple<std::string, std::string, std::string, std::string, std::string>> specs;
  std::string id;
  std::vector<std::tuple<int,std::vector<std::vector<uint8_t>>>> infos;

  switch (message.command)
  {
    case (SCMD_IDS_REQUEST):
      this->onRequestIds(message.entity);
      break;

    case (SCMD_IDS_RESPONSE):
      ids = deserialize<std::vector<std::tuple<std::string, std::string>>>(message.payload);
      message.entity->fuse(ids);
      message.entity->checkIce();
      break;

    case (SCMD_ID_REQUEST):
      id = deserialize<std::string>(message.payload);
      this->onRequestId(message.entity, id);
      break;

    case (SCMD_ID_RESPONSE):
      // TODO
      break;

    case (SCMD_OFFERS_REQUEST):
      this->onRequestOffers(message.entity);
      break;

    case (SCMD_OFFERS_RESPONSE):
      specs = deserialize<std::vector<std::tuple<std::string, std::string, std::string, std::string, std::string>>>(message.payload);
      message.entity->addOfferedInformation(specs);
      break;

    case (SCMD_INFORMATION_REQUEST):
      specs = deserialize<std::vector<std::tuple<std::string, std::string, std::string, std::string, std::string>>>(message.payload);
      this->onRequestInformation(message.entity, specs);
    break;

    case (SCMD_INFORMATION_RESPONSE):
      infos = deserialize<std::vector<std::tuple<int,std::vector<std::vector<uint8_t>>>>>(message.payload);
      std::cout << "#####################" << std::endl;
      std::cout << infos.size() << std::endl;

    break;

    default:
      _log->error("Unknown command '%v', message will be skipped", std::to_string(message.command));
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
//      this->directory->checkTimeout();

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
