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

#include "messages/Message.h"
#include "messages/CommandMessage.h"
#include "messages/IdMessage.h"
#include "messages/InformationMessage.h"
#include "messages/OffersMessage.h"
#include "messages/RequestMessage.h"
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

void CommunicationInterface::pushMessage(std::shared_ptr<Message> message)
{
  std::lock_guard<std::mutex> guard(this->_messageMtx);

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
  auto m = std::make_shared<CommandMessage>(IceCmd::SCMD_IDS_REQUEST);
  m->setEntity(entity);

  this->pushMessage(m);
}

void CommunicationInterface::onRequestIds(std::shared_ptr<Entity> const &entity)
{
  _log->info("Sending Ids to '%v'", entity->toString());
  auto m = std::make_shared<IdMessage>();
  m->setEntity(entity);
  this->self->pushIds(m->getIds());

  this->pushMessage(m);
}

void CommunicationInterface::requestOffers(std::shared_ptr<Entity> const &entity)
{
  _log->info("Requesting offered information from '%v'", entity->toString());
  auto m = std::make_shared<CommandMessage>(IceCmd::SCMD_OFFERS_REQUEST);
  m->setEntity(entity);

  this->pushMessage(m);
}

void CommunicationInterface::onRequestOffers(std::shared_ptr<Entity> const &entity)
{
  _log->info("Sending offered information to '%v'", entity->toString());
  auto m = std::make_shared<OffersMessage>();
  m->setEntity(entity);

  auto &vec = m->getOfferes();
  for (auto info : this->bridge->getOfferedInfos())
  {
    vec.push_back(info->infoSpec);
  }

  this->pushMessage(m);
}

void CommunicationInterface::requestInformation(std::shared_ptr<Entity> const &entity,
                                std::vector<std::shared_ptr<InformationSpecification>> const &requests)
{
  _log->info("Requesting information from '%v'", entity->toString());
  auto m = std::make_shared<RequestMessage>();
  m->setEntity(entity);

  auto &vec = m->getRequests();
  for (auto info : requests)
  {
    vec.push_back(info);
  }

  this->pushMessage(m);
}

void CommunicationInterface::onRequestInformation(std::shared_ptr<Entity> const &entity,
                                                  std::vector<std::shared_ptr<InformationSpecification>> const &requests)
{
  _log->info("Information request received from '%v'", entity->toString());
  auto m = std::make_shared<InformationMessage>();
  m->setEntity(entity);

  for (auto &request : requests)
  {
    this->bridge->informationStore->getInformation(request, m->getInformations());
  }

  int count = m->getInformations().size();
  if (count == 0)
  {
    _log->warn("No information found for request from '%v'", entity->toString());
    return;
  }

  _log->info("Sending '%v' information to '%v'", count, entity->toString());
  this->sendMessage(m);
}

void CommunicationInterface::onInformation(std::shared_ptr<Entity> const &entity,
                                           std::vector<std::shared_ptr<InformationElement<GContainer>>> &information)
{
  _log->info("Information received from '%v'", entity->toString());

  for (auto &info : information)
  {
    this->informationStore->addInformation(info);
  }
}

void CommunicationInterface::handleMessage(std::shared_ptr<Message> message)
{
  auto entity = message->getEntity();
  _log->info("Received Message with id '%v' from %v", std::to_string(message->getId()), entity->toString());
  entity->setActiveTimestamp();

  switch (message->getId())
  {
    case (SCMD_IDS_REQUEST):
      this->onRequestIds(entity);
      break;

    case (SCMD_IDS_RESPONSE):
    {
      entity->fuse(std::static_pointer_cast<IdMessage>(message)->getIds());
      entity->checkIce();
      break;
    }

    case (SCMD_ID_REQUEST):
        // TODO
      break;

    case (SCMD_ID_RESPONSE):
      // TODO
      break;

    case (SCMD_OFFERS_REQUEST):
      this->onRequestOffers(entity);
      break;

    case (SCMD_OFFERS_RESPONSE):
        entity->addOfferedInformation(std::static_pointer_cast<OffersMessage>(message)->getOfferes());
      break;

    case (SCMD_INFORMATION_REQUEST):
        this->onRequestInformation(entity, std::static_pointer_cast<RequestMessage>(message)->getRequests());
    break;

    case (SCMD_INFORMATION_RESPONSE):
        this->onInformation(entity, std::static_pointer_cast<InformationMessage>(message)->getInformations());
    break;

    default:
      _log->error("Unknown command '%v', message will be skipped", std::to_string(message->getId()));
      break;
  }
}

void CommunicationInterface::workerTask()
{
  int counter = 0;
  std::vector<std::shared_ptr<Message>> msgs;

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
      std::lock_guard<std::mutex> guard(this->_messageMtx);

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

std::shared_ptr<GContainerFactory> CommunicationInterface::getGContainerFactory()
{
  return this->containerFactory;
}

void CommunicationInterface::setGContainerFactory(std::shared_ptr<GContainerFactory> factory)
{
  this->containerFactory = factory;
}

std::shared_ptr<InformationStore> CommunicationInterface::getInformationStore()
{
  return this->informationStore;
}

void CommunicationInterface::setInformationStore(std::shared_ptr<InformationStore> store)
{
  this->informationStore = store;
}

std::shared_ptr<OntologyInterface> CommunicationInterface::getOntologyInterface()
{
  return this->ontology;
}

void CommunicationInterface::setOntologyInterface(std::shared_ptr<OntologyInterface> ontology)
{
  this->ontology = ontology;
}

} /* namespace ice */
