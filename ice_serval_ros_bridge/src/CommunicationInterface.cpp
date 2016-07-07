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
  // TODO

//  std::vector<std::shared_ptr<InformationElement<GContainer>>> infos;
//  std::vector<comInfoElement> gcontainers;
//
//  for (int i=0; i < requests.size(); ++i)
//  {
//    infos.clear();
//    auto t = requests.at(i);
//    auto request = std::make_shared<InformationSpecification>(std::get<0>(t),
//                                                              std::get<1>(t),
//                                                              std::get<2>(t),
//                                                              std::get<3>(t),
//                                                              std::get<4>(t));
//
//    this->bridge->informationStore->getInformation(request, infos);
//
//    for (auto &info : infos)
//    {
//      std::vector<std::vector<uint8_t>> vec;
//      info->getInformation()->toByte(vec);
//      gcontainers.push_back(std::make_tuple(t, vec));
//    }
//  }
//
//  Message m;
//  m.entity = entity;
//  m.command = IceCmd::SCMD_INFORMATION_RESPONSE;
//
//  serialize(gcontainers, m.payload);
//
//  this->sendMessage(m);
}

void CommunicationInterface::onInformation(std::shared_ptr<Entity> const &entity,
                                                   std::vector<comInfoElement> const &information)
{
  _log->info("Information received from '%v'", entity->toString());

  for (auto &info : information)
  {
    comInfoSpec cSpec = std::get<0>(info);
    auto bytes = std::get<1>(info);

    auto spec = std::make_shared<InformationSpecification>(std::get<0>(cSpec),
                                                           std::get<1>(cSpec),
                                                           std::get<2>(cSpec),
                                                           std::get<3>(cSpec),
                                                           std::get<4>(cSpec));

    std::string shortIri = this->ontology->toShortIri(std::get<3>(cSpec));
    auto container = this->containerFactory->makeInstance(shortIri);

    if (nullptr == container)
    {
      _log->error("GContainer for representation '%v' could not be created, information will be skipped", std::get<3>(cSpec));
      continue;
    }

    container->fromByte(bytes);

    auto e = std::make_shared<InformationElement<GContainer>>(spec, container);

    this->informationStore->addInformation(spec, e);
  }
}

void CommunicationInterface::handleMessage(std::shared_ptr<Message> message)
{
  auto entity = message->getEntity();
  _log->info("Received Message with id '%v' from %v", std::to_string(message->getId()), entity->toString());
  entity->setActiveTimestamp();

  std::string                                           id;
  std::vector<comInfoSpec>                              specs;
  std::vector<comRequest>                               requests;
  std::vector<comInfoElement>                           infos;

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
//      infos = deserialize<std::vector<comInfoElement>>(message.payload);
//      this->onInformation(message.entity, infos);
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
